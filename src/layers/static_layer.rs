//! The static-map layer: any map, on the master's lattice or another one.

use glam::DVec2;

use crate::{
    CostWindows, Costmap,
    costmap::{
        Layer, MergePolicy,
        project::merge_window,
        resample::{Resample, resample},
    },
    types::{Bounds, CellRegion, Pose2, VoxelError},
};

/// A layer that writes the part of a map under the master costmap into it.
///
/// The map is any [`CostWindows`]: a [`Costmap`], a
/// [`TiledGrid<u8>`](crate::TiledGrid) or, with the `geotiff` feature,
/// `GeoTiffWindows`. Each update the layer asks it for the cells under the
/// master. Where the map and the master share a lattice and frame it copies them;
/// otherwise it [`resample`]s them, with
/// [`Resample::for_resolutions`] unless [`with_resample`](Self::with_resample)
/// chose a policy.
///
/// The map may be in another frame than the master, such as `map` under an `odom`
/// master. [`set_transform`](Self::set_transform) gives the pose of the map's
/// frame in the master's; the caller sets it before each
/// [`update_map`](crate::LayeredCostmap::update_map) as the transform changes.
///
/// The layer rewrites every master cell each update. Unknown map cells leave the
/// master cell as the costmap's fill value.
pub struct StaticLayer<M> {
    map: M,
    policy: MergePolicy,
    resample: Option<Resample>,
    transform: Pose2,
    coverage: Coverage,
    error: Option<VoxelError>,
}

/// The world rectangle the master covers at the next update.
enum Coverage {
    /// Not yet attached to a costmap.
    None,
    Fixed(Bounds),
    /// Centred on the robot, this far either side of it.
    Rolling(DVec2),
}

impl<M: CostWindows> StaticLayer<M> {
    /// A layer over `map`, merging into the master with `policy`. The map's frame is
    /// the master's until [`set_transform`](Self::set_transform) says otherwise.
    pub fn new(map: M, policy: MergePolicy) -> Self {
        Self {
            map,
            policy,
            resample: None,
            transform: Pose2::default(),
            coverage: Coverage::None,
            error: None,
        }
    }

    /// Resample with `resample` instead of the policy the resolutions choose.
    pub fn with_resample(mut self, resample: Resample) -> Self {
        self.resample = Some(resample);
        self
    }

    /// The map this layer reads.
    pub fn map(&self) -> &M {
        &self.map
    }

    /// Mutable access to the map, for example to load and evict the chunks of a
    /// [`TiledGrid`](crate::TiledGrid) as the robot moves.
    pub fn map_mut(&mut self) -> &mut M {
        &mut self.map
    }

    /// The pose of the map's frame in the master's frame.
    pub fn transform(&self) -> Pose2 {
        self.transform
    }

    /// Set the pose of the map's frame in the master's frame, used from the next
    /// update on.
    pub fn set_transform(&mut self, map_in_master: Pose2) {
        self.transform = map_in_master;
    }

    /// The error of the last failed read of the map, cleared by this call. A failed
    /// read leaves the master's cells at the costmap's fill value.
    pub fn take_error(&mut self) -> Option<VoxelError> {
        self.error.take()
    }

    fn window(&mut self, master: &Costmap, cells: CellRegion) -> Result<Costmap, VoxelError> {
        let lattice = master.lattice();
        if self.transform == Pose2::default() && self.map.lattice() == lattice {
            return self.map.window(cells);
        }
        let policy = self.resample.unwrap_or_else(|| {
            Resample::for_resolutions(self.map.lattice().resolution, lattice.resolution)
        });
        resample(&mut self.map, self.transform, lattice, cells, policy)
    }
}

impl<M: CostWindows + 'static> Layer for StaticLayer<M> {
    fn reset(&mut self) {}

    fn is_clearable(&self) -> bool {
        false
    }

    /// Expands `bounds` to the whole master.
    fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds) {
        let covered = match self.coverage {
            Coverage::None => return,
            Coverage::Fixed(covered) => covered,
            Coverage::Rolling(half) => Bounds {
                min: robot.position - half,
                max: robot.position + half,
            },
        };
        bounds.expand_to_include(covered.min);
        bounds.expand_to_include(covered.max);
    }

    fn attach(&mut self, master: &Costmap, rolling_window: bool) -> Result<(), VoxelError> {
        let covered = master.world_bounds();
        self.coverage = if rolling_window {
            // A rolling master moves by whole cells: one more cell each side covers it.
            let half = (covered.max - covered.min) * 0.5;
            Coverage::Rolling(half + DVec2::splat(master.lattice().resolution))
        } else {
            Coverage::Fixed(covered)
        };
        Ok(())
    }

    fn update_costs(&mut self, master: &mut Costmap, region: CellRegion) {
        let Some(offsets) = master.offsets_in(region) else {
            return;
        };
        let held = CellRegion::new(master.cell_of(offsets.min), master.cell_of(offsets.max));
        match self.window(master, held) {
            Ok(window) => merge_window(master, &window, self.policy),
            Err(error) => self.error = Some(error),
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{IVec2, UVec2};

    use super::*;
    use crate::{
        InflationConfig, LayeredCostmap, TiledGrid, WavefrontInflationLayer,
        types::{COST_FREE, COST_LETHAL, COST_UNKNOWN, Lattice},
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
        LayeredCostmap::new(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(width)),
            COST_FREE,
            false,
        )
    }

    fn lethal_cells(layered: &LayeredCostmap) -> Vec<IVec2> {
        layered
            .master()
            .indexed_iter()
            .filter(|&(_, &cost)| cost == COST_LETHAL)
            .map(|(pos, _)| pos)
            .collect()
    }

    fn layer_with<M: CostWindows>(map: M) -> Box<StaticLayer<M>> {
        Box::new(StaticLayer::new(map, MergePolicy::Overwrite))
    }

    #[test]
    fn a_map_much_larger_than_the_master_writes_the_overlap() {
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

        assert_eq!(lethal_cells(&layered), [IVec2::new(5, 7)]);
    }

    #[test]
    fn unknown_map_cells_leave_the_fill_value() {
        let mut grid = tiles();
        fill_chunk(&mut grid, IVec2::new(0, 0), 40);
        fill_chunk(&mut grid, IVec2::new(1, 0), 40);
        grid.set(IVec2::new(17, 3), COST_LETHAL);

        let mut layered = LayeredCostmap::new(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(40)),
            33,
            false,
        );
        layered.master_mut().set(IVec2::new(35, 35), 77).unwrap();
        layered.add_layer(layer_with(grid)).unwrap();
        layered.update_map(Pose2::default());

        let master = layered.master();
        assert_eq!(master[IVec2::new(0, 0)], 40);
        assert_eq!(master[IVec2::new(31, 15)], 40);
        assert_eq!(master[IVec2::new(17, 3)], COST_LETHAL);
        assert_eq!(master[IVec2::new(32, 0)], 33);
        assert_eq!(master[IVec2::new(0, 16)], 33);
        assert_eq!(master[IVec2::new(35, 35)], 33);
    }

    #[test]
    fn max_policy_keeps_the_higher_cost() {
        let mut map = Costmap::filled(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(8)),
            COST_UNKNOWN,
        );
        map.set(IVec2::new(1, 1), 50).unwrap();
        map.set(IVec2::new(2, 2), 10).unwrap();

        let mut layered = LayeredCostmap::new(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(8)),
            30,
            false,
        );
        layered
            .add_layer(Box::new(StaticLayer::new(map, MergePolicy::Max)))
            .unwrap();
        layered.update_map(Pose2::default());
        assert_eq!(layered.master()[IVec2::new(1, 1)], 50);
        assert_eq!(layered.master()[IVec2::new(2, 2)], 30);
    }

    #[test]
    fn map_mut_pages_chunks_in_and_out() {
        let mut layered = fixed_master(16);
        let id = layered.add_layer(layer_with(tiles())).unwrap();
        let layer = layered.layer_mut::<StaticLayer<TiledGrid<u8>>>(id).unwrap();
        fill_chunk(layer.map_mut(), IVec2::ZERO, COST_LETHAL);

        layered.update_map(Pose2::default());
        assert_eq!(layered.master()[IVec2::new(3, 3)], COST_LETHAL);

        let layer = layered.layer_mut::<StaticLayer<TiledGrid<u8>>>(id).unwrap();
        layer.map_mut().remove_chunk(IVec2::ZERO);
        layered.update_map(Pose2::default());
        assert_eq!(layered.master()[IVec2::new(3, 3)], COST_FREE);
    }

    #[test]
    fn a_small_region_touches_only_its_cells() {
        let mut grid = tiles();
        fill_chunk(&mut grid, IVec2::new(0, 0), 60);
        let mut layer = StaticLayer::new(grid, MergePolicy::Overwrite);
        let mut master = Costmap::filled(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(16)),
            COST_FREE,
        );

        layer.update_costs(
            &mut master,
            CellRegion::new(IVec2::new(3, 4), IVec2::new(6, 5)),
        );

        let written: Vec<_> = master
            .indexed_iter()
            .filter(|&(_, &c)| c == 60)
            .map(|(p, _)| p)
            .collect();
        assert_eq!(
            written,
            [IVec2::new(3, 4), IVec2::new(4, 4), IVec2::new(5, 4)]
        );
    }

    #[test]
    fn a_rolling_master_with_inflation_tracks_an_obstacle_across_chunks() {
        let mut grid = tiles();
        for cx in 0..5 {
            fill_chunk(&mut grid, IVec2::new(cx, 0), COST_FREE);
        }
        let obstacle = IVec2::new(40, 3);
        grid.set(obstacle, COST_LETHAL);
        let obstacle_centre = grid.lattice().cell_center(obstacle);

        let mut layered = LayeredCostmap::new(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(20)),
            COST_FREE,
            true,
        );
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
            let expected = master.lattice().cell_at(obstacle_centre);
            assert_eq!(lethal_cells(&layered), [expected], "robot at x={robot_x}");
            let beside = master[expected + IVec2::X];
            assert!(beside > COST_FREE && beside < COST_LETHAL, "halo {beside}");
            assert_eq!(master[master.cells().min], COST_FREE);
        }
    }

    #[test]
    fn a_map_of_another_resolution_is_resampled_by_worst_cost() {
        let mut map = Costmap::filled(
            Lattice::new(DVec2::ZERO, 0.25),
            CellRegion::from_size(UVec2::splat(16)),
            COST_FREE,
        );
        map.set(IVec2::new(5, 9), COST_LETHAL).unwrap();

        let mut layered = fixed_master(4);
        layered.add_layer(layer_with(map)).unwrap();
        layered.update_map(Pose2::default());

        assert_eq!(lethal_cells(&layered), [IVec2::new(1, 2)]);
    }

    #[test]
    fn a_map_in_another_frame_follows_the_transform() {
        let mut map = Costmap::filled(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::splat(10)),
            COST_FREE,
        );
        map.set(IVec2::new(2, 3), COST_LETHAL).unwrap();

        let mut layered = fixed_master(10);
        let id = layered.add_layer(layer_with(map)).unwrap();
        layered.update_map(Pose2::default());
        assert_eq!(lethal_cells(&layered), [IVec2::new(2, 3)]);

        // The map frame moves 5 m along x and turns a quarter turn about its origin:
        // map (2.5, 3.5) is now at (5 - 3.5, 2.5) in the master frame.
        let layer = layered.layer_mut::<StaticLayer<Costmap>>(id).unwrap();
        layer.set_transform(Pose2::new(
            DVec2::new(5.0, 0.0),
            std::f32::consts::FRAC_PI_2,
        ));
        layered.update_map(Pose2::default());
        assert_eq!(lethal_cells(&layered), [IVec2::new(1, 2)]);
        assert_eq!(layered.master()[IVec2::new(8, 8)], COST_FREE);
    }
}
