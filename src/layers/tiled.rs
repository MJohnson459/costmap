//! Static-map layer backed by a [`TiledGrid`].

use std::sync::Arc;

use glam::IVec2;

use crate::{
    DenseGrid,
    costmap::{Costmap, Layer, MergePolicy, project::merge_cost},
    tiled::TiledGrid,
    types::{Bounds, COST_UNKNOWN, CellRegion, MapInfo, Pose2, VoxelError},
};

/// A layer that stamps the part of a tiled cost map under the master costmap into it.
///
/// This is the static-map layer for tiled maps. The tiled grid shares the master's
/// local frame and must have the master's resolution;
/// [`LayeredCostmap::add_layer`](crate::LayeredCostmap::add_layer) rejects a layer
/// whose resolution differs. Cells that are
/// [`COST_UNKNOWN`] in the tiles, including cells in unloaded chunks, leave the
/// master untouched. The master origin is snapped to the nearest tile cell
/// boundary, so a master whose origin is not a whole number of cells from the
/// tiled grid's origin, fixed or rolling, is stamped with up to half a cell of
/// offset.
pub struct TiledMapLayer {
    grid: Arc<TiledGrid<u8>>,
    policy: MergePolicy,
    /// Cell extent of the loaded chunks at the previous update, so cells of chunks
    /// evicted since then are refreshed too.
    previous_extent: Option<(IVec2, IVec2)>,
}

impl TiledMapLayer {
    /// Create a layer over `grid`, merging into the master with `policy`.
    pub fn new(grid: Arc<TiledGrid<u8>>, policy: MergePolicy) -> Self {
        Self {
            grid,
            policy,
            previous_extent: None,
        }
    }

    /// The tiled grid this layer reads.
    pub fn grid(&self) -> &Arc<TiledGrid<u8>> {
        &self.grid
    }

    /// Mutable access to the tiles, for loading and evicting chunks as the robot moves.
    ///
    /// Uses [`Arc::make_mut`]: if another `Arc` still points at the grid, the grid is
    /// cloned first and the other holders keep the old tiles. A layer that is the
    /// grid's only owner pages in place without copying.
    pub fn grid_mut(&mut self) -> &mut TiledGrid<u8> {
        Arc::make_mut(&mut self.grid)
    }

    /// Dense copy of the tiles under `region` of a master described by `info`.
    fn region_window(&self, info: &MapInfo, region: CellRegion) -> Option<DenseGrid<u8>> {
        let size = region.max.saturating_sub(region.min);
        if size.x == 0 || size.y == 0 {
            return None;
        }
        let window_info = MapInfo {
            width: size.x,
            height: size.y,
            resolution: info.resolution,
            origin: info.origin + region.min.as_dvec2() * info.resolution,
        };
        let mut window = DenseGrid::filled(window_info, COST_UNKNOWN);
        self.grid.read_into(&mut window).ok()?;
        Some(window)
    }
}

impl Layer for TiledMapLayer {
    fn reset(&mut self) {}

    fn is_clearable(&self) -> bool {
        false
    }

    /// Expands `bounds` to the extent of the loaded chunks, and to the extent at the
    /// previous update so that cells of evicted chunks are reset.
    ///
    /// The extent may be far larger than the master or not overlap it at all.
    /// [`LayeredCostmap`](crate::LayeredCostmap) clips the bounds to the master.
    fn update_bounds(&mut self, _robot: Pose2, bounds: &mut Bounds) {
        let current = self.grid.loaded_cell_bounds();
        for (min, max) in [current, self.previous_extent].into_iter().flatten() {
            bounds.expand_to_include(self.grid.cell_origin(min));
            bounds.expand_to_include(self.grid.cell_origin(max));
        }
        self.previous_extent = current;
    }

    fn attach(&mut self, master: &MapInfo, _rolling_window: bool) -> Result<(), VoxelError> {
        let resolution = self.grid.resolution();
        if !self.grid.resolution_matches(master.resolution) {
            return Err(VoxelError::InvalidArgument(format!(
                "tiled map resolution {resolution} m differs from the costmap's {} m",
                master.resolution
            )));
        }
        Ok(())
    }

    fn update_costs(&mut self, master: &mut Costmap, region: CellRegion) {
        let info = *master.info();
        let Some(window) = self.region_window(&info, region) else {
            return;
        };
        let size = window.info();
        let (width, region_width) = (info.width as usize, size.width as usize);
        let rows = (region.min.y..region.max.y).zip(window.as_slice().chunks_exact(region_width));
        for (y, source_row) in rows {
            let start = y as usize * width + region.min.x as usize;
            let master_row = &mut master.as_mut_slice()[start..start + region_width];
            merge_row(self.policy, master_row, source_row);
        }
    }
}

/// Merge `source` into `master` cell by cell, skipping unknown source cells.
fn merge_row(policy: MergePolicy, master: &mut [u8], source: &[u8]) {
    for (cell, &cost) in master.iter_mut().zip(source) {
        if cost == COST_UNKNOWN {
            continue;
        }
        if let Some(merged) = merge_cost(policy, *cell, cost) {
            *cell = merged;
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{DVec2, IVec2, UVec2};

    use super::*;
    use crate::{
        InflationConfig, LayeredCostmap, WavefrontInflationLayer,
        types::{COST_FREE, COST_LETHAL},
    };

    const CHUNK: u32 = 16;

    fn tiles() -> TiledGrid<u8> {
        TiledGrid::new(1.0, CHUNK, COST_UNKNOWN).unwrap()
    }

    fn fill_chunk(grid: &mut TiledGrid<u8>, coord: IVec2, value: u8) {
        let len = (CHUNK * CHUNK) as usize;
        grid.insert_chunk(coord, vec![value; len].into()).unwrap();
    }

    fn fixed_master(width: u32) -> LayeredCostmap {
        LayeredCostmap::new(MapInfo::square(width, 1.0), COST_FREE, false)
    }

    fn lethal_cells(layered: &LayeredCostmap) -> Vec<UVec2> {
        layered
            .master()
            .indexed_iter()
            .filter(|&(_, &cost)| cost == COST_LETHAL)
            .map(|(pos, _)| pos)
            .collect()
    }

    fn layer_with(grid: TiledGrid<u8>) -> Box<TiledMapLayer> {
        Box::new(TiledMapLayer::new(Arc::new(grid), MergePolicy::Overwrite))
    }

    #[test]
    fn extent_much_larger_than_master_stamps_the_overlap() {
        let mut grid = tiles();
        for cy in -6..6 {
            for cx in -6..6 {
                fill_chunk(&mut grid, IVec2::new(cx, cy), COST_FREE);
            }
        }
        grid.set(IVec2::new(5, 7), COST_LETHAL);
        grid.set(IVec2::new(-30, 7), COST_LETHAL);

        let mut layered = fixed_master(20);
        layered.add_layer(layer_with(grid)).unwrap();
        layered.update_map(Pose2::default());

        assert_eq!(lethal_cells(&layered), [UVec2::new(5, 7)]);
    }

    #[test]
    fn partial_overlap_stamps_only_loaded_cells() {
        let mut grid = tiles();
        fill_chunk(&mut grid, IVec2::new(0, 0), 40);
        fill_chunk(&mut grid, IVec2::new(1, 0), 40);
        grid.set(IVec2::new(17, 3), COST_LETHAL);

        let mut layered = fixed_master(40);
        layered.master_mut().set(UVec2::new(35, 35), 77).unwrap();
        layered.add_layer(layer_with(grid)).unwrap();
        layered.update_map(Pose2::default());

        let master = layered.master();
        assert_eq!(master[UVec2::new(0, 0)], 40);
        assert_eq!(master[UVec2::new(31, 15)], 40);
        assert_eq!(master[UVec2::new(17, 3)], COST_LETHAL);
        assert_eq!(master[UVec2::new(32, 0)], COST_FREE);
        assert_eq!(master[UVec2::new(0, 16)], COST_FREE);
    }

    #[test]
    fn no_overlap_leaves_master_content_free() {
        let mut grid = tiles();
        fill_chunk(&mut grid, IVec2::new(50, 50), COST_LETHAL);

        let mut layered = fixed_master(20);
        layered.add_layer(layer_with(grid)).unwrap();
        layered.update_map(Pose2::default());

        assert!(
            layered
                .master()
                .as_slice()
                .iter()
                .all(|&cost| cost == COST_FREE)
        );
    }

    #[test]
    fn unknown_tile_cells_leave_master_untouched() {
        let mut grid = tiles();
        fill_chunk(&mut grid, IVec2::new(0, 0), COST_UNKNOWN);
        grid.set(IVec2::new(2, 2), 10);

        let mut layered = LayeredCostmap::new(MapInfo::square(16, 1.0), 33, false);
        layered.add_layer(layer_with(grid)).unwrap();
        layered.update_map(Pose2::default());

        let master = layered.master();
        assert_eq!(master[UVec2::new(2, 2)], 10);
        assert_eq!(master[UVec2::new(3, 3)], 33);
    }

    #[test]
    fn max_policy_keeps_higher_master_cost() {
        let mut grid = tiles();
        grid.set(IVec2::new(1, 1), 50);
        grid.set(IVec2::new(2, 2), 50);

        let mut layered = fixed_master(8);
        layered.master_mut().set(UVec2::new(2, 2), 90).unwrap();
        layered
            .add_layer(Box::new(TiledMapLayer::new(
                Arc::new(grid),
                MergePolicy::Max,
            )))
            .unwrap();
        // The master is reset to free inside the update region before layers run,
        // so the pre-set 90 is gone and the layer stamps 50 everywhere known.
        layered.update_map(Pose2::default());
        assert_eq!(layered.master()[UVec2::new(2, 2)], 50);
    }

    #[test]
    fn grid_mut_pages_in_place_when_unshared() {
        let mut layered = fixed_master(16);
        let id = layered.add_layer(layer_with(tiles())).unwrap();
        let layer = layered.layer_mut::<TiledMapLayer>(id).unwrap();
        let before = Arc::as_ptr(layer.grid());

        fill_chunk(layer.grid_mut(), IVec2::ZERO, COST_LETHAL);
        assert_eq!(Arc::as_ptr(layer.grid()), before);

        layered.update_map(Pose2::default());
        assert_eq!(layered.master()[UVec2::new(3, 3)], COST_LETHAL);

        let layer = layered.layer_mut::<TiledMapLayer>(id).unwrap();
        layer.grid_mut().remove_chunk(IVec2::ZERO);
        layered.update_map(Pose2::default());
        assert_eq!(layered.master()[UVec2::new(3, 3)], COST_FREE);
    }

    #[test]
    fn grid_mut_leaves_other_holders_unchanged() {
        let shared = Arc::new(tiles());
        let mut layer = TiledMapLayer::new(Arc::clone(&shared), MergePolicy::Overwrite);
        fill_chunk(layer.grid_mut(), IVec2::ZERO, COST_LETHAL);

        assert!(!shared.is_loaded(IVec2::ZERO));
        assert!(layer.grid().is_loaded(IVec2::ZERO));
    }

    #[test]
    fn small_region_only_touches_region() {
        let mut grid = tiles();
        fill_chunk(&mut grid, IVec2::new(0, 0), 60);
        let mut layer = TiledMapLayer::new(Arc::new(grid), MergePolicy::Overwrite);
        let mut master = Costmap::filled(MapInfo::square(16, 1.0), COST_FREE);

        layer.update_costs(
            &mut master,
            CellRegion::new(UVec2::new(3, 4), UVec2::new(6, 5)),
        );

        let stamped: Vec<_> = master
            .indexed_iter()
            .filter(|&(_, &c)| c == 60)
            .map(|(p, _)| p)
            .collect();
        assert_eq!(
            stamped,
            [UVec2::new(3, 4), UVec2::new(4, 4), UVec2::new(5, 4)]
        );
    }

    #[test]
    fn rolling_window_with_inflation_tracks_obstacle_across_chunk_boundary() {
        let mut grid = tiles();
        for cx in 0..5 {
            fill_chunk(&mut grid, IVec2::new(cx, 0), COST_FREE);
        }
        let obstacle = IVec2::new(40, 3);
        grid.set(obstacle, COST_LETHAL);
        let obstacle_centre = grid.cell_center(obstacle);

        let mut layered = LayeredCostmap::new(MapInfo::square(20, 1.0), COST_FREE, true);
        layered.add_layer(layer_with(grid)).unwrap();
        layered
            .add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
                inflation_radius_m: 3.0,
                inscribed_radius_m: 0.5,
                cost_scaling_factor: 1.0,
                ..Default::default()
            })))
            .unwrap();

        // Master x ranges 25..45 then 35..55; the obstacle's chunk (x 32..48)
        // is first partly, then wholly inside, and the master crosses the x=48 edge.
        for robot_x in [35.0, 45.0] {
            layered.update_map(Pose2::new(DVec2::new(robot_x, 3.0), 0.0));

            let master = layered.master();
            let expected = master.world_to_map(obstacle_centre).unwrap();
            assert_eq!(lethal_cells(&layered), [expected], "robot at x={robot_x}");
            let beside = master[expected + UVec2::X];
            assert!(beside > COST_FREE && beside < COST_LETHAL, "halo {beside}");
            assert_eq!(master[UVec2::new(0, 0)], COST_FREE);
        }
    }

    #[test]
    fn add_layer_rejects_a_different_resolution() {
        let mut half_metre = LayeredCostmap::new(MapInfo::square(8, 0.5), COST_FREE, false);
        let layer = TiledMapLayer::new(Arc::new(tiles()), MergePolicy::Overwrite);

        assert!(matches!(
            half_metre.add_layer(Box::new(layer)),
            Err(VoxelError::InvalidArgument(_))
        ));
    }
}
