//! Layered costmap: container of layers that write into a master grid.
//!
//! Uses Rust-native types (`glam::Vec2`, `UVec2`, `Grid2d<u8>`). The update loop
//! aggregates bounds from all layers, resets the master region, then calls
//! each layer's `update_costs` in order.

use glam::{UVec2, Vec2};

use crate::grid::Grid2d;
use crate::types::MapInfo;

/// Robot pose in world coordinates (meters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    pub position: Vec2,
    pub yaw: f32,
}

/// World-axis-aligned rectangle in meters.
/// Convention: [min.x, max.x) x [min.y, max.y) in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: Vec2,
    pub max: Vec2,
}

impl Bounds {
    /// Create bounds that represent "no region" (empty). Use this as the initial
    /// value before layers expand it; layers should only expand, never shrink.
    pub fn empty() -> Self {
        Self {
            min: Vec2::new(f32::INFINITY, f32::INFINITY),
            max: Vec2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    /// Returns true if no layer has expanded the bounds (min > max in either axis).
    pub fn is_empty(&self) -> bool {
        self.min.x >= self.max.x || self.min.y >= self.max.y
    }

    /// Expand this bounds to include the point (in place).
    pub fn expand_to_include(&mut self, p: Vec2) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
    }

    /// Expand by a margin in meters in all directions (e.g. for inflation halo).
    pub fn expand_by(&mut self, margin: f32) {
        self.min.x -= margin;
        self.min.y -= margin;
        self.max.x += margin;
        self.max.y += margin;
    }
}

/// Update window in cell indices. Region is [min.x, max.x) x [min.y, max.y).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellRegion {
    pub min: UVec2,
    pub max: UVec2,
}

/// Footprint: polygon in world coordinates (meters). Used by layers for footprint callbacks.
#[derive(Debug, Clone, Default)]
pub struct Footprint {
    pub points: Vec<Vec2>,
}

/// Layer plugin interface. Layers are called in order: each may expand bounds,
/// then each writes into the master grid within the computed region.
pub trait Layer {
    /// Reset the layer to its initial state.
    fn reset(&mut self);

    /// Whether global "clear costmap" should call reset on this layer.
    fn is_clearable(&self) -> bool;

    /// Expand the world bounds that this layer needs to update.
    /// Called once per update; layers only expand bounds, never shrink.
    fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds);

    /// Write into the master grid only within `region`.
    fn update_costs(&mut self, master: &mut Grid2d<u8>, region: CellRegion);

    /// Called when the robot footprint changes. Default: no-op.
    fn on_footprint_changed(&mut self, _footprint: &Footprint) {}

    /// Called when the master grid is resized. Default: no-op.
    fn match_size(&mut self, _info: &MapInfo) {}
}

/// Container of layers and a master costmap. Runs update_bounds then update_costs
/// in order each time `update_map` is called.
pub struct LayeredGrid2d {
    master: Grid2d<u8>,
    layers: Vec<Box<dyn Layer>>,
    rolling_window: bool,
    updated_bounds: Bounds,
}

impl LayeredGrid2d {
    /// Create a layered costmap with the given master grid.
    pub fn new(master: Grid2d<u8>, rolling_window: bool) -> Self {
        Self {
            master,
            layers: Vec::new(),
            rolling_window,
            updated_bounds: Bounds::empty(),
        }
    }

    /// Add a layer. Order matters: layers are updated in insertion order.
    pub fn add_layer(&mut self, layer: Box<dyn Layer>) {
        self.layers.push(layer);
    }

    /// Immutable reference to the master grid.
    pub fn master(&self) -> &Grid2d<u8> {
        &self.master
    }

    /// Mutable reference to the master grid.
    pub fn master_mut(&mut self) -> &mut Grid2d<u8> {
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
            self.master.update_origin(&new_origin);
        }

        let mut bounds = Bounds::empty();
        for layer in &mut self.layers {
            layer.update_bounds(robot, &mut bounds);
        }

        if bounds.is_empty() {
            self.updated_bounds = bounds;
            return;
        }

        let width = self.master.width();
        let height = self.master.height();

        let min_cell = self.master.world_to_map(&bounds.min);
        let max_cell = self.master.world_to_map(&bounds.max);

        let (x0, y0) = match min_cell {
            Some(c) => (c.x.min(width), c.y.min(height)),
            None => (0, 0),
        };
        let (xn, yn) = match max_cell {
            Some(c) => ((c.x + 1).min(width), (c.y + 1).min(height)),
            None => (width, height),
        };

        if x0 >= xn || y0 >= yn {
            self.updated_bounds = bounds;
            return;
        }

        let region = CellRegion {
            min: UVec2::new(x0, y0),
            max: UVec2::new(xn, yn),
        };

        self.master.reset_map(region.min, region.max);

        for layer in &mut self.layers {
            layer.update_costs(&mut self.master, region);
        }

        self.updated_bounds = bounds;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::MapInfo;

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
    fn layered_grid2d_update_map_aggregates_bounds() {
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
            fn update_costs(&mut self, _master: &mut Grid2d<u8>, _region: CellRegion) {}
        }

        let master = Grid2d::<u8>::filled(default_info(), 0);
        let mut layered = LayeredGrid2d::new(master, false);
        layered.add_layer(Box::new(BoundsLayer { margin: 0.5 }));

        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let b = layered.updated_bounds();
        assert!(!b.is_empty());
        assert!(b.min.x <= 0.5 && b.max.x >= 0.5);
        assert!(b.min.y <= 0.5 && b.max.y >= 0.5);
    }
}
