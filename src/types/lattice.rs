//! Placement of cells in the world.

use glam::{DVec2, IVec2};

/// Where cells sit in the world: cell `(0, 0)` has its lower-left corner at
/// `anchor`, and cells are `resolution` metres square.
///
/// A cell holds its lower and left edges, so a position on the edge between two
/// cells belongs to the upper or right one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lattice {
    pub anchor: DVec2,
    pub resolution: f64,
}

impl Lattice {
    #[inline]
    pub fn new(anchor: DVec2, resolution: f64) -> Self {
        Self { anchor, resolution }
    }

    /// The cell containing `position`. Every position has one: coordinates beyond
    /// the `i32` range saturate, and NaN gives 0.
    #[inline]
    pub fn cell_at(&self, position: DVec2) -> IVec2 {
        let cells = self.to_cells(position);
        IVec2::new(floor_i32(cells.x), floor_i32(cells.y))
    }

    /// World position of the centre of `cell`.
    #[inline]
    pub fn cell_center(&self, cell: IVec2) -> DVec2 {
        self.anchor + (cell.as_dvec2() + DVec2::splat(0.5)) * self.resolution
    }

    /// World position of the lower-left corner of `cell`.
    #[inline]
    pub fn cell_corner(&self, cell: IVec2) -> DVec2 {
        self.anchor + cell.as_dvec2() * self.resolution
    }

    /// `position` in cell units from the anchor, not rounded.
    #[inline]
    pub fn to_cells(&self, position: DVec2) -> DVec2 {
        (position - self.anchor) / self.resolution
    }

    /// The world position `cells` cell units from the anchor.
    #[inline]
    pub fn from_cells(&self, cells: DVec2) -> DVec2 {
        self.anchor + cells * self.resolution
    }
}

/// `v` rounded toward negative infinity, saturating at the `i32` limits; NaN gives 0.
/// Truncates and steps down when truncation rounded up, because `f64::floor` is a
/// libm call on the baseline x86-64 target.
#[inline]
fn floor_i32(v: f64) -> i32 {
    let t = v as i32;
    t.saturating_sub(i32::from(v < f64::from(t)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lattice() -> Lattice {
        Lattice::new(DVec2::new(-1.0, 2.0), 0.25)
    }

    #[test]
    fn cell_round_trips_through_its_centre_and_corner() {
        let l = lattice();
        for cell in [
            IVec2::ZERO,
            IVec2::new(7, -3),
            IVec2::new(-40, 12),
            IVec2::new(1_000_000, -1_000_000),
        ] {
            assert_eq!(l.cell_at(l.cell_center(cell)), cell);
            assert_eq!(l.cell_at(l.cell_corner(cell)), cell);
        }
        let position = DVec2::new(3.3125, -7.0625);
        assert_eq!(l.from_cells(l.to_cells(position)), position);
    }

    #[test]
    fn negative_positions_round_down() {
        let l = Lattice::new(DVec2::ZERO, 0.5);
        assert_eq!(l.cell_at(DVec2::new(-0.01, -0.49)), IVec2::new(-1, -1));
        assert_eq!(l.cell_at(DVec2::new(-0.51, 0.49)), IVec2::new(-2, 0));
        assert_eq!(l.cell_at(DVec2::new(-0.0, -0.0)), IVec2::ZERO);
        let anchored = lattice();
        assert_eq!(
            anchored.cell_at(anchored.anchor - DVec2::splat(1e-9)),
            IVec2::new(-1, -1)
        );
    }

    #[test]
    fn edge_belongs_to_the_cell_above_and_right() {
        let l = Lattice::new(DVec2::ZERO, 0.5);
        assert_eq!(l.cell_at(DVec2::new(1.0, 0.5)), IVec2::new(2, 1));
        assert_eq!(l.cell_at(DVec2::new(-1.0, -0.5)), IVec2::new(-2, -1));
        let anchored = lattice();
        assert_eq!(anchored.cell_at(anchored.anchor), IVec2::ZERO);
    }

    #[test]
    fn centre_is_half_a_cell_past_the_corner() {
        let l = lattice();
        let cell = IVec2::new(-2, 3);
        assert_eq!(l.cell_corner(cell), DVec2::new(-1.5, 2.75));
        assert_eq!(l.cell_center(cell), DVec2::new(-1.375, 2.875));
        assert_eq!(l.to_cells(l.cell_center(cell)), DVec2::new(-1.5, 3.5));
    }

    #[test]
    fn cell_at_saturates_and_maps_nan_to_zero() {
        let l = Lattice::new(DVec2::ZERO, 1.0);
        assert_eq!(
            l.cell_at(DVec2::new(f64::INFINITY, f64::NEG_INFINITY)),
            IVec2::new(i32::MAX, i32::MIN)
        );
        assert_eq!(
            l.cell_at(DVec2::new(-3.0e9, 3.0e9)),
            IVec2::new(i32::MIN, i32::MAX)
        );
        assert_eq!(l.cell_at(DVec2::NAN), IVec2::ZERO);
    }
}
