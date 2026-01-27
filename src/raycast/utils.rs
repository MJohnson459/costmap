use glam::{IVec2, UVec2};

use crate::{
    OccupancyGrid,
    types::{FREE, OCCUPIED},
};

#[inline]
pub fn in_bounds(cell: &IVec2, bounds: &UVec2) -> bool {
    (cell.x as u32) < bounds.x && (cell.y as u32) < bounds.y
}

#[inline]
pub fn is_occupied(grid: &OccupancyGrid, cell: &IVec2) -> bool {
    let value = grid.get(&cell.as_uvec2()).unwrap_or(&FREE);
    value >= &OCCUPIED
}
