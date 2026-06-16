//! A layer that owns a generic semantic grid and projects it into the master.
//!
//! `ProjectionLayer<T>` is the first-class way to integrate a custom `Grid2d<T>`
//! into a [`LayeredCostmap`](crate::LayeredCostmap) for either global or rolling-window
//! costmaps. The caller can ingest data via [`Self::source_mut`], read it back via
//! [`Self::source`], and the layer handles both the geometric transformation (rolling
//! window centering) and cost merging.
//!
//! This layer pattern is inspired by Nav2's `CostmapLayer`, but generalized to
//! support any cell type `T` with a semantic → cost projection closure.

use glam::Vec2;

use crate::{
    Grid2d,
    costmap::{Costmap, Layer, MergePolicy, project_into},
    types::{Bounds, CellRegion, Pose2},
};

/// A layer that owns a semantic grid and projects it into the master costmap.
///
/// Supports both global (fixed-frame) and rolling-window (robot-centered) operation.
/// When `rolling_window` is set, the source grid is re-centred on the robot during
/// [`update_bounds`](Layer::update_bounds) — before any layer's `update_costs` runs —
/// so producers that write into the source during `update_costs` see a centred grid.
#[allow(clippy::type_complexity)]
pub struct ProjectionLayer<T> {
    source: Grid2d<T>,
    policy: MergePolicy,
    project: Box<dyn Fn(&T) -> Option<u8> + Send + Sync>,
    rolling_window: bool,
    clearable: bool,
}

impl<T> ProjectionLayer<T> {
    /// Create a new projection layer from a grid and projection closure.
    ///
    /// # Arguments
    ///
    /// * `source` — the semantic grid owned by this layer.
    /// * `policy` — the merge policy (Overwrite, Max, MaxKeepUnknown).
    /// * `rolling_window` — if true, the source is centered on the robot each update.
    /// * `project` — closure that converts `&T` → `Option<u8>` cost. `None` leaves the
    ///   master cell untouched; `Some(cost)` applies the policy.
    pub fn from_grid(
        source: Grid2d<T>,
        policy: MergePolicy,
        rolling_window: bool,
        project: impl Fn(&T) -> Option<u8> + Send + Sync + 'static,
    ) -> Self {
        Self {
            source,
            policy,
            project: Box::new(project),
            rolling_window,
            clearable: true,
        }
    }

    /// Immutable reference to the source grid (query path).
    pub fn source(&self) -> &Grid2d<T> {
        &self.source
    }

    /// Mutable reference to the source grid (ingestion path).
    ///
    /// Use this to write semantic data into the grid between updates. Typically
    /// called after `layer_mut::<ProjectionLayer<T>>(id)` and before `update_map`.
    pub fn source_mut(&mut self) -> &mut Grid2d<T> {
        &mut self.source
    }
}

impl<T: Clone + 'static> Layer for ProjectionLayer<T> {
    fn reset(&mut self) {
        self.source.clear();
    }

    fn is_clearable(&self) -> bool {
        self.clearable
    }

    fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds) {
        // For a rolling window, re-centre the source on the robot here, before any
        // layer's update_costs runs. This lets producers that write into the source
        // during update_costs (e.g. the lidar example) stamp into an already-centred grid.
        if self.rolling_window {
            self.source.update_center(robot.position);
        }

        // Expand bounds to the source grid's (now current) world extent.
        let info = self.source.info();
        bounds.expand_to_include(info.origin);
        bounds.expand_to_include(info.origin + Vec2::new(info.world_width(), info.world_height()));
    }

    fn update_costs(&mut self, master: &mut Costmap, region: CellRegion) {
        project_into(master, &self.source, region, self.policy, |cell| {
            (self.project)(cell)
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{COST_FREE, COST_LETHAL, MapInfo};
    use crate::{InflationConfig, LayeredCostmap, WavefrontInflationLayer};

    fn default_info() -> MapInfo {
        MapInfo::square(5, 1.0)
    }

    #[test]
    fn test_projection_layer_via_layered_costmap() {
        // Create a layered costmap with a projection layer that projects f32 to u8.
        let mut layered = LayeredCostmap::new(default_info(), COST_FREE, false);

        let source = Grid2d::<f32>::new_with_value(default_info(), 0.0);
        let id = layered.add_layer(Box::new(ProjectionLayer::from_grid(
            source,
            MergePolicy::Overwrite,
            false,
            |v| {
                // Map 0.0 → COST_FREE, 1.0 → COST_LETHAL
                Some((v * 254.0).round() as u8)
            },
        )));

        // Write a value into the source
        {
            let proj_layer = layered
                .layer_mut::<ProjectionLayer<f32>>(id)
                .expect("layer should exist and be the right type");
            proj_layer
                .source_mut()
                .set(glam::UVec2::new(2, 2), 1.0)
                .unwrap();
        }

        // Update the master
        layered.update_map(Pose2::default());

        // Check that the master reflects the projected cost
        assert_eq!(
            layered.master().get(glam::UVec2::new(2, 2)),
            Some(&COST_LETHAL)
        );
        // Other cells should remain free
        assert_eq!(
            layered.master().get(glam::UVec2::new(0, 0)),
            Some(&COST_FREE)
        );
    }

    #[test]
    fn test_projection_layer_wrong_type_returns_none() {
        let mut layered = LayeredCostmap::new(default_info(), COST_FREE, false);

        let source = Grid2d::<f32>::new_with_value(default_info(), 0.0);
        let id = layered.add_layer(Box::new(ProjectionLayer::from_grid(
            source,
            MergePolicy::Overwrite,
            false,
            |v| Some((*v * 254.0) as u8),
        )));

        // Try to fetch as the wrong type
        let wrong_type = layered.layer::<WavefrontInflationLayer>(id);
        assert!(wrong_type.is_none());

        // Correct type should work
        let correct_type = layered.layer::<ProjectionLayer<f32>>(id);
        assert!(correct_type.is_some());
    }

    #[test]
    fn test_projection_layer_with_inflation() {
        // Test that projection layer can be stacked with inflation.
        let mut layered = LayeredCostmap::new(default_info(), COST_FREE, false);

        let source = Grid2d::<f32>::new_with_value(default_info(), 0.0);
        let proj_id = layered.add_layer(Box::new(ProjectionLayer::from_grid(
            source,
            MergePolicy::Overwrite,
            false,
            |v| Some((*v * 254.0) as u8),
        )));

        // Add inflation after projection. Radius 1.5 m (> 1 cell) so the cell adjacent
        // to a lethal seed gets an inflated halo.
        layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
            inflation_radius_m: 1.5,
            inscribed_radius_m: 0.0,
            cost_scaling_factor: 1.0,
            ..Default::default()
        })));

        // Project a lethal cell (1.0 * 254 == COST_LETHAL) so it seeds inflation.
        {
            let proj_layer = layered.layer_mut::<ProjectionLayer<f32>>(proj_id).unwrap();
            proj_layer
                .source_mut()
                .set(glam::UVec2::new(2, 2), 1.0)
                .unwrap();
        }

        layered.update_map(Pose2::default());

        // The projected lethal cost is present...
        assert_eq!(
            layered.master().get(glam::UVec2::new(2, 2)),
            Some(&COST_LETHAL)
        );
        // ...and it seeds inflation: the adjacent cell gets a halo in (FREE, LETHAL).
        let neighbour = layered
            .master()
            .get(glam::UVec2::new(3, 2))
            .copied()
            .unwrap();
        assert!(
            neighbour > COST_FREE && neighbour < COST_LETHAL,
            "expected inflated halo at (3,2), got {neighbour}"
        );
    }

    #[test]
    fn test_projection_layer_reset() {
        let mut layered = LayeredCostmap::new(default_info(), COST_FREE, false);

        let source = Grid2d::<f32>::new_with_value(default_info(), 0.0);
        let id = layered.add_layer(Box::new(ProjectionLayer::from_grid(
            source,
            MergePolicy::Overwrite,
            false,
            |v| Some((*v * 254.0) as u8),
        )));

        // Write data
        {
            let proj_layer = layered.layer_mut::<ProjectionLayer<f32>>(id).unwrap();
            proj_layer
                .source_mut()
                .set(glam::UVec2::new(1, 1), 0.5)
                .unwrap();
        }

        // Reset via layer interface
        {
            let proj_layer = layered.layer_mut::<ProjectionLayer<f32>>(id).unwrap();
            proj_layer.reset();
        }

        // Verify data is cleared to fill_value
        {
            let proj_layer = layered.layer::<ProjectionLayer<f32>>(id).unwrap();
            assert_eq!(proj_layer.source().get(glam::UVec2::new(1, 1)), Some(&0.0));
        }
    }

    #[test]
    fn test_projection_layer_is_clearable() {
        let layer = ProjectionLayer::from_grid(
            Grid2d::<f32>::new_with_value(default_info(), 0.0),
            MergePolicy::Overwrite,
            false,
            |_| Some(100),
        );
        assert!(layer.is_clearable());
    }

    #[test]
    fn update_bounds_rolling_recenters_and_covers_source_extent() {
        // 5x5 @ 1.0 m → a 5 m square, half-extent 2.5 m.
        let mut layer = ProjectionLayer::from_grid(
            Grid2d::<f32>::new_with_value(default_info(), 0.0),
            MergePolicy::Overwrite,
            true,
            |_| Some(0),
        );

        let mut bounds = Bounds::empty();
        let robot = Pose2 {
            position: Vec2::new(5.0, 5.0),
            yaw: 0.0,
        };
        layer.update_bounds(robot, &mut bounds);

        // Rolling: the source is re-centred on the robot during update_bounds. The
        // origin snaps to whole cells (update_origin shifts by integer cell offsets),
        // so centring on (5,5) with 1 m cells gives origin (2.0, 2.0), not (2.5, 2.5).
        assert_eq!(layer.source().info().origin, Vec2::new(2.0, 2.0));
        // Bounds cover exactly the (re-centred) source world extent — i.e. where the
        // layer can actually write, not some larger nominal range.
        assert_eq!(bounds.min, Vec2::new(2.0, 2.0));
        assert_eq!(bounds.max, Vec2::new(7.0, 7.0));
    }

    #[test]
    fn update_bounds_non_rolling_uses_fixed_extent() {
        let mut layer = ProjectionLayer::from_grid(
            Grid2d::<f32>::new_with_value(default_info(), 0.0),
            MergePolicy::Overwrite,
            false,
            |_| Some(0),
        );

        let mut bounds = Bounds::empty();
        layer.update_bounds(
            Pose2 {
                position: Vec2::new(5.0, 5.0),
                yaw: 0.0,
            },
            &mut bounds,
        );

        // Not rolling: the source stays put and bounds are its fixed world extent.
        assert_eq!(layer.source().info().origin, Vec2::ZERO);
        assert_eq!(bounds.min, Vec2::ZERO);
        assert_eq!(bounds.max, Vec2::new(5.0, 5.0));
    }

    #[test]
    fn rolling_window_projects_to_correct_world_cell_as_window_moves() {
        // The motivating path: a rolling master + rolling projection layer (same
        // MapInfo) driven through update_map. A cost fixed in *world* space must land
        // at the right master cell as the window moves, and master/source must stay
        // layout-aligned (project_into's debug_assert fires here, in debug test builds).
        let mut layered = LayeredCostmap::new(default_info(), COST_FREE, true);
        let id = layered.add_layer(Box::new(ProjectionLayer::from_grid(
            Grid2d::<f32>::new_with_value(default_info(), 0.0),
            MergePolicy::Max,
            true,
            |t| (*t > 0.5).then_some(COST_LETHAL),
        )));

        // Robot centred so the window origin stays at (0,0). Mark world (3.5, 3.5).
        layered
            .layer_mut::<ProjectionLayer<f32>>(id)
            .unwrap()
            .source_mut()
            .set(glam::UVec2::new(3, 3), 1.0)
            .unwrap();
        layered.update_map(Pose2 {
            position: Vec2::new(2.5, 2.5),
            yaw: 0.0,
        });
        // World (3.5,3.5) → cell (3,3) with origin (0,0).
        assert_eq!(
            layered.master().get(glam::UVec2::new(3, 3)),
            Some(&COST_LETHAL)
        );

        // Move the window +1 m in x → origin snaps to (1,0). No re-marking: the source
        // persists data by world position across the shift.
        layered.update_map(Pose2 {
            position: Vec2::new(3.5, 2.5),
            yaw: 0.0,
        });
        // World (3.5,3.5) now maps to cell (2,3) with origin (1,0).
        assert_eq!(
            layered.master().get(glam::UVec2::new(2, 3)),
            Some(&COST_LETHAL)
        );
        // The old cell no longer holds the marked world location.
        assert_eq!(
            layered.master().get(glam::UVec2::new(3, 3)),
            Some(&COST_FREE)
        );
    }
}
