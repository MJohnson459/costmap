//! A layered costmap is a container of layers that write into a base master
//! grid.
//!
//! Each layer is processed in order and updates are based on bounds passed from
//! higher layers.
//!
//! The update loop aggregates bounds from all layers, resets the master region,
//! then calls each layer's `update_costs` in order.

use std::any::Any;

use glam::Vec2;

use crate::{
    Costmap,
    types::{Bounds, CellRegion, Footprint, MapInfo, Pose2, VoxelError},
};

/// Identifier returned by [`LayeredCostmap::add_layer`], used to fetch a layer
/// back by its concrete type with [`LayeredCostmap::layer`] /
/// [`LayeredCostmap::layer_mut`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LayerId(usize);

/// A layer is a component which acts on the base master grid. It may contain
/// its own grid but doesn't need to.
///
/// Each layer is processed in order.
///
/// Each update is limited by a set of bounds passed from higher layers.
///
/// The `Any` supertrait enables [`LayeredCostmap::layer`] /
/// [`LayeredCostmap::layer_mut`] to downcast a stored layer back to its concrete
/// type so callers can ingest into or query a layer's internal grid after it has
/// been added to the stack.
pub trait Layer: Any {
    /// Reset the layer to its initial state.
    fn reset(&mut self);

    /// Whether global "clear costmap" should call reset on this layer.
    fn is_clearable(&self) -> bool;

    /// Expand the world bounds that this layer needs to update.
    /// Called once per update; layers only expand bounds, never shrink.
    fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds);

    /// Write into the master grid only within `region`.
    fn update_costs(&mut self, master: &mut Costmap, region: CellRegion);

    /// Called when the robot footprint changes. Default: no-op.
    fn on_footprint_changed(&mut self, _footprint: &Footprint) {}

    /// Called when the master grid is resized. Default: no-op.
    fn match_size(&mut self, _info: &MapInfo) {}

    /// Called once by [`LayeredCostmap::add_layer`] with the master's layout and
    /// whether it is a rolling window. Return an error if the layer cannot work
    /// with that costmap, so a misconfigured layer fails when it is added
    /// instead of contributing nothing at run time. Default: accept.
    fn attach(&mut self, _master: &MapInfo, _rolling_window: bool) -> Result<(), VoxelError> {
        Ok(())
    }
}

/// Container of layers and a master costmap. Runs update_bounds then update_costs
/// in order each time `update_map` is called.
pub struct LayeredCostmap {
    master: Costmap,
    layers: Vec<Box<dyn Layer>>,
    rolling_window: bool,
    updated_bounds: Bounds,
}

impl LayeredCostmap {
    /// Create a layered costmap with a new master grid of the given size and default value.
    ///
    /// The master is set uniformly to `fill_value` for all cells and the updated region
    /// is reset to `fill_value` each frame before layers write.
    ///
    /// If `rolling_window` is true, the grid origin is recentered on the robot each
    /// [`update_map`](Self::update_map), so the costmap stays fixed in size but moves
    /// with the robot (local costmap). If false, the grid is fixed in world frame.
    pub fn new(info: MapInfo, fill_value: u8, rolling_window: bool) -> Self {
        let master = Costmap::new_with_value(info, fill_value);
        Self {
            master,
            layers: Vec::new(),
            rolling_window,
            updated_bounds: Bounds::empty(),
        }
    }

    /// Add a layer. Order matters: layers are updated in insertion order.
    ///
    /// Returns a [`LayerId`] that can be passed to [`Self::layer`] /
    /// [`Self::layer_mut`] to access the layer by its concrete type later, or the
    /// error from [`Layer::attach`] if the layer rejects this costmap; a rejected
    /// layer is not added.
    pub fn add_layer(&mut self, mut layer: Box<dyn Layer>) -> Result<LayerId, VoxelError> {
        layer.attach(self.master.info(), self.rolling_window)?;
        let id = LayerId(self.layers.len());
        self.layers.push(layer);
        Ok(id)
    }

    /// Borrow a previously added layer as its concrete type `T`.
    ///
    /// Returns `None` if the id is unknown or the layer is not a `T`.
    pub fn layer<T: Layer>(&self, id: LayerId) -> Option<&T> {
        let layer: &dyn Layer = self.layers.get(id.0)?.as_ref();
        (layer as &dyn Any).downcast_ref::<T>()
    }

    /// Mutably borrow a previously added layer as its concrete type `T`.
    ///
    /// Returns `None` if the id is unknown or the layer is not a `T`. This is the
    /// path used to ingest data into a layer's internal grid between updates.
    pub fn layer_mut<T: Layer>(&mut self, id: LayerId) -> Option<&mut T> {
        let layer: &mut dyn Layer = self.layers.get_mut(id.0)?.as_mut();
        (layer as &mut dyn Any).downcast_mut::<T>()
    }

    /// Immutable reference to the master grid.
    pub fn master(&self) -> &Costmap {
        &self.master
    }

    /// Mutable reference to the master grid.
    pub fn master_mut(&mut self) -> &mut Costmap {
        &mut self.master
    }

    /// Whether the costmap is configured as a rolling window (origin follows robot).
    pub fn is_rolling_window(&self) -> bool {
        self.rolling_window
    }

    /// World bounds that were updated in the last `update_map` call.
    pub fn updated_bounds(&self) -> Bounds {
        self.updated_bounds
    }

    /// Run the update loop: optionally move origin, aggregate bounds from all layers,
    /// reset the master in that region, then call each layer's update_costs.
    pub fn update_map(&mut self, robot: Pose2) {
        if self.rolling_window {
            let info = self.master.info();
            let half_w = (info.width as f32) * info.resolution * 0.5;
            let half_h = (info.height as f32) * info.resolution * 0.5;
            let new_origin = robot.position - Vec2::new(half_w, half_h);
            self.master.update_origin(new_origin);
        }

        let mut bounds = Bounds::empty();
        for layer in &mut self.layers {
            layer.update_bounds(robot, &mut bounds);
        }

        if bounds.is_empty() {
            self.updated_bounds = bounds;
            return;
        }

        let Some(region) = clip_to_cells(bounds, self.master.info()) else {
            self.updated_bounds = bounds;
            return;
        };

        self.master.reset_map(region.min, region.max);

        for layer in &mut self.layers {
            layer.update_costs(&mut self.master, region);
        }

        self.updated_bounds = bounds;
    }
}

/// Cell region of `info` covered by world `bounds`, or `None` if they do not overlap.
///
/// The cell containing `bounds.max` is included. Indices are computed in `f32`
/// and clamped before the integer conversion, so bounds far outside the grid
/// clip to its edge.
fn clip_to_cells(bounds: Bounds, info: &MapInfo) -> Option<CellRegion> {
    let size = Vec2::new(info.width as f32, info.height as f32);
    let lower = ((bounds.min - info.origin) / info.resolution).floor();
    let upper = ((bounds.max - info.origin) / info.resolution).floor() + Vec2::ONE;
    let min = lower.clamp(Vec2::ZERO, size).as_uvec2();
    let max = upper.clamp(Vec2::ZERO, size).as_uvec2();
    (min.x < max.x && min.y < max.y).then_some(CellRegion { min, max })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::MapInfo;
    use glam::UVec2;

    fn default_info() -> MapInfo {
        MapInfo {
            width: 10,
            height: 10,
            resolution: 0.1,
            ..Default::default()
        }
    }

    #[test]
    fn bounds_empty_and_expand() {
        let mut b = Bounds::empty();
        assert!(b.is_empty());

        b.expand_to_include(Vec2::new(1.0, 2.0));
        b.expand_to_include(Vec2::new(3.0, 0.0));
        assert!(!b.is_empty());
        assert_eq!(b.min, Vec2::new(1.0, 0.0));
        assert_eq!(b.max, Vec2::new(3.0, 2.0));

        b.expand_by(0.5);
        assert_eq!(b.min, Vec2::new(0.5, -0.5));
        assert_eq!(b.max, Vec2::new(3.5, 2.5));
    }

    #[test]
    fn layered_costmap_update_map_aggregates_bounds() {
        struct BoundsLayer {
            margin: f32,
        }
        impl Layer for BoundsLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds) {
                bounds.expand_to_include(robot.position);
                bounds.expand_by(self.margin);
            }
            fn update_costs(&mut self, _master: &mut Costmap, _region: CellRegion) {}
        }

        let mut layered = LayeredCostmap::new(default_info(), 0, false);
        layered
            .add_layer(Box::new(BoundsLayer { margin: 0.5 }))
            .unwrap();

        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let b = layered.updated_bounds();
        assert!(!b.is_empty());
        assert!(b.min.x <= 0.5 && b.max.x >= 0.5);
        assert!(b.min.y <= 0.5 && b.max.y >= 0.5);
    }

    fn bounds(min: Vec2, max: Vec2) -> Bounds {
        Bounds { min, max }
    }

    #[test]
    fn clip_to_cells_inside_includes_max_cell() {
        let region = clip_to_cells(
            bounds(Vec2::new(0.25, 0.25), Vec2::new(0.55, 0.35)),
            &default_info(),
        )
        .unwrap();
        assert_eq!(region.min, UVec2::new(2, 2));
        assert_eq!(region.max, UVec2::new(6, 4));
    }

    #[test]
    fn clip_to_cells_partial_overlap_clips_to_edge() {
        let region = clip_to_cells(
            bounds(Vec2::new(-5.0, 0.85), Vec2::new(0.15, 50.0)),
            &default_info(),
        )
        .unwrap();
        assert_eq!(region.min, UVec2::new(0, 8));
        assert_eq!(region.max, UVec2::new(2, 10));
    }

    #[test]
    fn clip_to_cells_no_overlap_is_none() {
        let info = default_info();
        let beyond = bounds(Vec2::new(5.0, 5.0), Vec2::new(6.0, 6.0));
        let before = bounds(Vec2::new(-6.0, -6.0), Vec2::new(-5.0, -5.0));
        assert!(clip_to_cells(beyond, &info).is_none());
        assert!(clip_to_cells(before, &info).is_none());
    }

    #[test]
    fn update_outside_master_leaves_it_untouched() {
        struct FarLayer;
        impl Layer for FarLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, _robot: Pose2, bounds: &mut Bounds) {
                bounds.expand_to_include(Vec2::new(5.0, 5.0));
                bounds.expand_to_include(Vec2::new(6.0, 6.0));
            }
            fn update_costs(&mut self, _master: &mut Costmap, _region: CellRegion) {
                panic!("no cells of the master are inside the bounds");
            }
        }

        let mut layered = LayeredCostmap::new(default_info(), 0, false);
        layered.master_mut().set(UVec2::new(3, 3), 42).unwrap();
        layered.add_layer(Box::new(FarLayer)).unwrap();
        layered.update_map(Pose2::new(Vec2::ZERO, 0.0));

        assert_eq!(layered.master().get(UVec2::new(3, 3)), Some(&42));
    }
}
