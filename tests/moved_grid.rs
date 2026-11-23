//! A grid that has moved holds other cells of its lattice. Every operation must
//! give the answer it gives on a grid that has not moved, with each cell named
//! [`SHIFT`] further along.

use costmap::costmap::merge_max;
use costmap::planning::Regions;
use costmap::types::{
    COST_FREE, COST_LETHAL, COST_UNKNOWN, CellRegion, FREE, Footprint, Lattice, OCCUPIED, Pose2,
};
use costmap::{
    Costmap, DenseGrid, DenseTiles, InflationConfig, Layer, OccupancyGrid, PlanOptions,
    WavefrontInflationLayer, costmap_from_occupancy, line_passable, plan,
};
use glam::{DVec2, IVec2, UVec2, Vec2};

const SHIFT: IVec2 = IVec2::new(7, -3);

fn lattice() -> Lattice {
    Lattice::new(DVec2::new(-2.0, 1.0), 0.5)
}

fn cells() -> CellRegion {
    CellRegion::from_size(UVec2::new(40, 30))
}

/// A wall across the map at x = 20 with a door at y = 10..14, and a block.
fn walled(cell: IVec2) -> bool {
    let wall = cell.x == 20 && !(10..14).contains(&cell.y);
    let block = (5..8).contains(&cell.x) && (20..23).contains(&cell.y);
    wall || block
}

fn costs() -> Costmap {
    Costmap::from_fn(lattice(), cells(), |cell| {
        if walled(cell) { COST_LETHAL } else { COST_FREE }
    })
}

fn occupancy() -> OccupancyGrid {
    OccupancyGrid::from_fn(lattice(), cells(), |cell| {
        if walled(cell) { OCCUPIED } else { FREE }
    })
}

/// The values of `grid` at the same places in the world, in a grid that started
/// [`SHIFT`] cells away and moved there: its cell `c + SHIFT` is `grid`'s cell `c`.
fn moved<T: Clone + Default>(grid: &DenseGrid<T>) -> DenseGrid<T> {
    let lattice = grid.lattice();
    let start = Lattice::new(
        lattice.anchor - SHIFT.as_dvec2() * lattice.resolution,
        lattice.resolution,
    );
    let mut moved = DenseGrid::<T>::new(start, CellRegion::from_size(grid.size()));
    moved.update_origin(grid.world_bounds().min + DVec2::splat(0.25 * lattice.resolution));
    moved.as_mut_slice().clone_from_slice(grid.as_slice());

    assert_eq!(moved.cells().min, SHIFT);
    assert_eq!(moved.world_bounds(), grid.world_bounds());
    assert_eq!(moved.lattice().resolution, grid.lattice().resolution);
    assert_eq!(moved.size(), grid.size());
    moved
}

fn shifted(cells: impl IntoIterator<Item = IVec2>) -> Vec<IVec2> {
    cells.into_iter().map(|cell| cell + SHIFT).collect()
}

#[test]
fn cells_are_read_and_written_by_name() {
    let grid = costs();
    let mut moved = moved(&grid);

    assert_eq!(
        moved.cells(),
        CellRegion::new(SHIFT, SHIFT + IVec2::new(40, 30))
    );
    for (cell, cost) in grid.indexed_iter() {
        assert_eq!(moved.get(cell + SHIFT), Some(cost), "at {cell}");
        assert_eq!(
            moved.lattice().cell_center(cell + SHIFT),
            grid.lattice().cell_center(cell)
        );
    }
    assert!(!moved.contains(IVec2::ZERO));
    assert!(moved.get(SHIFT - IVec2::X).is_none());
    assert!(moved.set(SHIFT + IVec2::new(40, 0), 1).is_err());

    let named: Vec<IVec2> = moved.indexed_iter().map(|(cell, _)| cell).collect();
    assert_eq!(named, shifted(grid.indexed_iter().map(|(cell, _)| cell)));

    moved.set(SHIFT + IVec2::new(3, 2), 9).unwrap();
    assert_eq!(moved.as_slice()[2 * 40 + 3], 9);
    assert_eq!(moved[SHIFT + IVec2::new(3, 2)], 9);
}

#[test]
fn reset_map_takes_cells() {
    let mut grid = costs();
    let mut moved = moved(&grid);
    let (lower, upper) = (IVec2::new(4, 19), IVec2::new(9, 22));

    grid.reset_map(lower, upper);
    moved.reset_map(lower + SHIFT, upper + SHIFT);

    assert_eq!(grid[IVec2::new(6, 20)], COST_FREE);
    assert_eq!(grid[IVec2::new(6, 22)], COST_LETHAL);
    assert_eq!(moved.as_slice(), grid.as_slice());
}

#[test]
fn a_line_crosses_the_same_cells() {
    let grid = costs();
    let moved = moved(&grid);
    let origin = DVec2::new(-0.3, 2.1);

    for dir in [
        Vec2::new(1.0, 0.4),
        Vec2::new(-1.0, 0.2),
        Vec2::new(0.3, -1.0),
    ] {
        let steps: Vec<_> = grid.line(origin, dir, 30.0).unwrap().collect();
        let moved_steps: Vec<_> = moved.line(origin, dir, 30.0).unwrap().collect();

        assert!(steps.len() > 3);
        assert_eq!(
            shifted(steps.iter().map(|step| step.cell)),
            moved_steps.iter().map(|step| step.cell).collect::<Vec<_>>()
        );
        assert!(steps.iter().zip(&moved_steps).all(|(a, b)| a.t == b.t));
        assert!(moved_steps.iter().all(|step| moved.contains(step.cell)));
    }
}

#[test]
fn a_polygon_covers_the_same_cells() {
    let grid = costs();
    let moved = moved(&grid);
    // Partly off the grid's lower-left corner.
    let points = [
        DVec2::new(-3.0, 0.2),
        DVec2::new(4.3, 1.4),
        DVec2::new(5.1, 6.6),
        DVec2::new(-1.2, 5.0),
    ];

    let cells: Vec<IVec2> = grid.polygon(&points).unwrap().collect();
    let moved_cells: Vec<IVec2> = moved.polygon(&points).unwrap().collect();

    assert!(cells.len() > 20);
    assert_eq!(shifted(cells), moved_cells);
    assert!(moved_cells.iter().all(|&cell| moved.contains(cell)));
}

#[test]
fn a_footprint_costs_the_same() {
    let grid = costs();
    let moved = moved(&grid);
    let footprint = Footprint::rectangle(1.2, 0.8);
    let on_block = Pose2::new(DVec2::new(1.2, 11.6), 0.4);
    let clear = Pose2::new(DVec2::new(3.0, 4.0), 0.4);

    assert_eq!(grid.footprint_cost(on_block, &footprint), COST_LETHAL);
    assert_eq!(moved.footprint_cost(on_block, &footprint), COST_LETHAL);
    assert_eq!(grid.footprint_cost(clear, &footprint), COST_FREE);
    assert_eq!(moved.footprint_cost(clear, &footprint), COST_FREE);
}

#[test]
fn a_ray_hits_the_same_cell() {
    let grid = occupancy();
    let moved = moved(&grid);
    let origin = DVec2::new(-0.3, 2.1);
    let dir = Vec2::new(1.0, 0.1);

    let hit = grid.raycast_dda(origin, dir, 30.0).unwrap();
    let moved_hit = moved.raycast_dda(origin, dir, 30.0).unwrap();
    assert_eq!(hit.cell.x, 20);
    assert_eq!(moved_hit.cell, hit.cell + SHIFT);
    assert_eq!(moved_hit.hit_distance, hit.hit_distance);

    let hit = grid.raycast_grid_step(origin, dir, 30.0).unwrap();
    let moved_hit = moved.raycast_grid_step(origin, dir, 30.0).unwrap();
    assert_eq!(hit.cell.x, 20);
    assert_eq!(moved_hit.cell, hit.cell + SHIFT);
    assert_eq!(moved_hit.hit_distance, hit.hit_distance);

    let outside = DVec2::new(-50.0, 2.0);
    assert!(moved.raycast_dda(outside, dir, 30.0).is_none());
    assert!(moved.raycast_grid_step(outside, dir, 30.0).is_none());
}

#[test]
fn inflation_gives_the_same_costs() {
    let mut grid = costs();
    let mut moved = moved(&grid);

    grid.inflate(2.0, 0.6, 3.0);
    moved.inflate(2.0, 0.6, 3.0);

    assert!(
        grid.as_slice()
            .iter()
            .any(|&c| c > COST_FREE && c < COST_LETHAL)
    );
    assert_eq!(moved.as_slice(), grid.as_slice());
}

#[test]
fn inflating_a_region_takes_cells() {
    let mut grid = costs();
    let mut moved = moved(&grid);
    let region = CellRegion::new(IVec2::new(15, 4), IVec2::new(26, 18));
    let moved_region = CellRegion::new(region.min + SHIFT, region.max + SHIFT);
    let config = InflationConfig {
        inflation_radius_m: 2.0,
        inscribed_radius_m: 0.6,
        cost_scaling_factor: 3.0,
        ..Default::default()
    };

    WavefrontInflationLayer::new(config.clone()).update_costs(&mut grid, region);
    WavefrontInflationLayer::new(config).update_costs(&mut moved, moved_region);

    assert!(grid[IVec2::new(19, 5)] > COST_FREE);
    assert_eq!(grid[IVec2::new(19, 20)], COST_FREE);
    assert_eq!(moved.as_slice(), grid.as_slice());
}

#[test]
fn a_plan_is_the_same_path() {
    let grid = costs();
    let moved = moved(&grid);
    let options = PlanOptions::default();
    let (start, goal) = (IVec2::new(2, 2), IVec2::new(37, 27));

    let path = plan(&grid, start, goal, &options).unwrap();
    let moved_path = plan(&moved, start + SHIFT, goal + SHIFT, &options).unwrap();

    assert!(path.cells.iter().any(|cell| cell.x == 20));
    assert_eq!(moved_path.cells, shifted(path.cells.iter().copied()));
    assert_eq!(moved_path.cost, path.cost);
    assert_eq!(moved_path.to_world(&moved), path.to_world(&grid));
    assert_eq!(
        moved_path.shortcut(&moved, &options),
        shifted(path.shortcut(&grid, &options))
    );

    // The same cell names on the moved grid are other places: (2, 2) is outside it.
    assert!(plan(&moved, start, goal + SHIFT, &options).is_none());

    let (a, b) = (IVec2::new(18, 12), IVec2::new(23, 12));
    assert!(line_passable(&grid, a, b, &options));
    assert!(line_passable(&moved, a + SHIFT, b + SHIFT, &options));
    let (a, b) = (IVec2::new(18, 20), IVec2::new(23, 20));
    assert!(!line_passable(&grid, a, b, &options));
    assert!(!line_passable(&moved, a + SHIFT, b + SHIFT, &options));
}

#[test]
fn regions_are_the_same_cells() {
    // Close the door, so the wall splits the map in two.
    let mut grid = costs();
    for y in 10..14 {
        grid.set(IVec2::new(20, y), COST_LETHAL).unwrap();
    }
    let moved = moved(&grid);
    let options = PlanOptions::default();

    let regions = Regions::new(&grid, &options);
    let moved_regions = Regions::new(&moved, &options);
    let largest = regions.largest().unwrap();
    let moved_largest = moved_regions.largest().unwrap();

    assert_eq!(moved_regions.size(moved_largest), regions.size(largest));
    assert_eq!(
        moved_regions.cells(moved_largest).collect::<Vec<_>>(),
        shifted(regions.cells(largest))
    );
    for (cell, _) in grid.indexed_iter() {
        assert_eq!(
            moved_regions.region(cell + SHIFT).is_some(),
            regions.region(cell).is_some(),
            "at {cell}"
        );
    }
    let (left, right) = (IVec2::new(3, 3), IVec2::new(30, 3));
    assert_ne!(regions.region(left), regions.region(right));
    assert_ne!(
        moved_regions.region(left + SHIFT),
        moved_regions.region(right + SHIFT)
    );
    assert!(moved_regions.region(SHIFT - IVec2::ONE).is_none());
}

#[test]
fn a_costmap_from_occupancy_keeps_the_cells() {
    let moved = moved(&occupancy());
    let costs = costmap_from_occupancy(&moved).unwrap();

    assert!(costs.layout_matches(&moved));
    assert_eq!(costs[SHIFT + IVec2::new(20, 0)], COST_LETHAL);
}

#[test]
fn grids_holding_different_cells_of_one_lattice_merge_where_they_overlap() {
    let mut master = Costmap::new(lattice(), cells());
    let mut source = Costmap::filled(lattice(), cells(), COST_UNKNOWN);
    source.update_origin(master.lattice().cell_corner(IVec2::new(10, 5)));
    assert_eq!(source.lattice(), master.lattice());
    assert_eq!(source.cells().min, IVec2::new(10, 5));

    let shared = IVec2::new(12, 6);
    let beyond_master = IVec2::new(45, 6);
    source.set(shared, COST_LETHAL).unwrap();
    source.set(beyond_master, COST_LETHAL).unwrap();

    let region = master.cells();
    merge_max(&mut master, &source, region);

    let lethal: Vec<IVec2> = master
        .indexed_iter()
        .filter(|&(_, &cost)| cost == COST_LETHAL)
        .map(|(cell, _)| cell)
        .collect();
    assert_eq!(lethal, [shared]);
}

#[test]
fn a_tile_graph_refuses_a_moved_grid() {
    let grid = costs();
    assert!(DenseTiles::new(&grid, 16).is_ok());
    assert!(DenseTiles::new(moved(&grid), 16).is_err());
}
