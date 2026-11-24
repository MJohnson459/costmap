//! Every deprecated name still compiles and forwards to its replacement. Deleting
//! an alias by accident fails this build; deleting one on purpose means deleting
//! its use here and listing it in MIGRATION.md.
#![allow(deprecated)]

use costmap::types::{CellRegion, Lattice};
use costmap::{CostmapError, DenseGrid, Grid2d, VoxelError};
use glam::{DVec2, IVec2, UVec2};

fn grid() -> DenseGrid<u8> {
    DenseGrid::filled(
        Lattice::new(DVec2::ZERO, 1.0),
        CellRegion::from_size(UVec2::new(4, 3)),
        0,
    )
}

#[test]
fn renamed_types_are_aliases() {
    let error: VoxelError = CostmapError::InvalidArgument("x".into());
    let _: costmap::types::VoxelError = error;
    let _: Grid2d<u8> = grid();
}

#[test]
fn dense_grid_accessors_forward() {
    let mut grid = grid();
    assert_eq!(grid.width(), grid.size().x);
    assert_eq!(grid.height(), grid.size().y);
    grid.iter_mut().for_each(|v| *v = 1);
    assert!(grid.iter().all(|&v| v == 1));
}

#[test]
fn dense_grid_moves_forward() {
    let mut old = grid();
    let mut new = grid();

    old.update_origin(DVec2::new(1.0, 2.0));
    new.move_to(DVec2::new(1.0, 2.0));
    assert_eq!(old.cells(), new.cells());

    old.update_center(DVec2::new(-3.0, 5.0));
    new.center_on(DVec2::new(-3.0, 5.0));
    assert_eq!(old.cells(), new.cells());

    old.set(new.cells().min, 7).unwrap();
    old.reset_map(new.cells().min, new.cells().min + IVec2::ONE);
    assert_eq!(old.get(new.cells().min), Some(&0));

    old.resize_map(UVec2::new(2, 2), 0.5, DVec2::new(1.0, 1.0));
    new.reset_to(
        Lattice::new(DVec2::new(1.0, 1.0), 0.5),
        CellRegion::from_size(UVec2::new(2, 2)),
    );
    assert_eq!(old.cells(), new.cells());
    assert_eq!(old.lattice(), new.lattice());
}
