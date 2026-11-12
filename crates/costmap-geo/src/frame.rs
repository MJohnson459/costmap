use costmap::MapInfo;
use glam::{DVec2, UVec2, Vec2};

/// Layout of a working grid of `size` cells in cell space: one unit per cell, origin at
/// the corner of cell `(0, 0)`. Projected positions come from a [`CellFrame`], which
/// keeps them in `f64`.
pub fn cells(size: UVec2) -> MapInfo {
    MapInfo {
        width: size.x,
        height: size.y,
        resolution: 1.0,
        origin: Vec2::ZERO,
    }
}

/// Where a grid sits in projected coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellFrame {
    /// Projected position of the lower-left corner of cell `(0, 0)`.
    pub origin: DVec2,
    /// Cell edge length in metres.
    pub resolution: f64,
}

impl CellFrame {
    /// Projected position of the centre of cell `(x, y)`; the cell may lie outside the grid.
    pub fn centre(&self, x: i64, y: i64) -> DVec2 {
        self.origin + DVec2::new(x as f64 + 0.5, y as f64 + 0.5) * self.resolution
    }
}
