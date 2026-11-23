//! The one request every cost map answers: the cells of a rectangle as a dense window.

use glam::IVec2;

use crate::{
    Costmap,
    types::{COST_UNKNOWN, CellRegion, Lattice, VoxelError},
};

/// A cost map that answers one request: the cells of a rectangle, as a dense window
/// on the map's lattice.
///
/// Implemented for [`Costmap`], [`TiledGrid<u8>`](crate::TiledGrid) and, with the
/// `geotiff` feature, `GeoTiffWindows`. Algorithms take the returned window; none
/// is generic over the map it came from.
///
/// ```
/// use costmap::{CostWindows, Costmap, Lattice, types::{COST_UNKNOWN, CellRegion}};
/// use glam::{DVec2, IVec2, UVec2};
///
/// let mut map = Costmap::filled(
///     Lattice::new(DVec2::ZERO, 0.1),
///     CellRegion::from_size(UVec2::splat(10)),
///     7,
/// );
/// let cells = CellRegion::sized(IVec2::new(8, 8), UVec2::splat(4));
/// let window = map.window(cells).unwrap();
/// assert_eq!(window.cells(), cells);
/// assert_eq!(window.get(IVec2::new(9, 9)), Some(&7));
/// assert_eq!(window.get(IVec2::new(10, 10)), Some(&COST_UNKNOWN));
/// ```
pub trait CostWindows {
    /// The lattice every window is cut on.
    fn lattice(&self) -> Lattice;

    /// The cells of `cells`: a [`Costmap`] on [`lattice`](Self::lattice) whose
    /// [`cells`](crate::DenseGrid::cells) equal `cells`. Cells the map does not hold
    /// are [`COST_UNKNOWN`]. Fails only if the map's storage cannot be read.
    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError>;
}

impl CostWindows for Costmap {
    fn lattice(&self) -> Lattice {
        Costmap::lattice(self)
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        Ok(dense_window(self, cells))
    }
}

impl CostWindows for &Costmap {
    fn lattice(&self) -> Lattice {
        Costmap::lattice(self)
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        Ok(dense_window(self, cells))
    }
}

impl<W: CostWindows + ?Sized> CostWindows for &mut W {
    fn lattice(&self) -> Lattice {
        (**self).lattice()
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        (**self).window(cells)
    }
}

impl<W: CostWindows + ?Sized> CostWindows for Box<W> {
    fn lattice(&self) -> Lattice {
        (**self).lattice()
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        (**self).window(cells)
    }
}

/// Copies the rows `map` and `cells` share into an unknown window over `cells`.
fn dense_window(map: &Costmap, cells: CellRegion) -> Costmap {
    let mut window = Costmap::filled(map.lattice(), cells, COST_UNKNOWN);
    let Some(shared) = map.offsets_in(cells) else {
        return window;
    };
    // A map offset plus this is a window offset; both are in `[0, size)`.
    let to_window = map.cells().min.as_i64vec2() - cells.min.as_i64vec2();
    let (from_width, to_width) = (map.size().x as usize, window.size().x as usize);
    let columns = (shared.max.x - shared.min.x) as usize;
    for y in shared.min.y..shared.max.y {
        let from = IVec2::new(shared.min.x, y);
        let to = (from.as_i64vec2() + to_window).as_uvec2();
        let from = y as usize * from_width + from.x as usize;
        let to = to.y as usize * to_width + to.x as usize;
        window.as_mut_slice()[to..to + columns]
            .copy_from_slice(&map.as_slice()[from..from + columns]);
    }
    window
}

#[cfg(test)]
mod tests {
    use glam::{DVec2, UVec2};

    use super::*;

    /// 10 x 8 cells at 0.5 m from cell (-3, 4), each holding a distinct value.
    fn numbered_map() -> Costmap {
        let lattice = Lattice::new(DVec2::new(-2.0, 1.0), 0.5);
        let cells = CellRegion::sized(IVec2::new(-3, 4), UVec2::new(10, 8));
        Costmap::from_fn(lattice, cells, |c| (10 * (c.y - 4) + c.x + 3) as u8)
    }

    /// The map's value of each window cell, or unknown where the map has none.
    fn expected(map: &Costmap, window: &Costmap) -> Vec<u8> {
        window
            .indexed_iter()
            .map(|(cell, _)| map.get(cell).copied().unwrap_or(COST_UNKNOWN))
            .collect()
    }

    fn check(cells: CellRegion) -> Costmap {
        let mut map = numbered_map();
        let window = map.window(cells).unwrap();
        assert_eq!(window.lattice(), map.lattice());
        assert_eq!(window.cells(), cells);
        assert_eq!(window.as_slice(), expected(&map, &window).as_slice());
        window
    }

    #[test]
    fn interior_window_copies_its_cells() {
        let window = check(CellRegion::sized(IVec2::new(-1, 6), UVec2::new(4, 3)));
        assert!(window.as_slice().iter().all(|&c| c != COST_UNKNOWN));
    }

    #[test]
    fn window_over_the_edge_is_unknown_outside_the_map() {
        let window = check(CellRegion::sized(IVec2::new(-6, 9), UVec2::new(6, 6)));
        assert!(window.as_slice().contains(&COST_UNKNOWN));
        assert!(window.as_slice().iter().any(|&c| c != COST_UNKNOWN));
    }

    #[test]
    fn window_larger_than_the_map_holds_all_of_it() {
        let window = check(CellRegion::sized(IVec2::new(-20, -20), UVec2::splat(50)));
        assert_eq!(
            window
                .as_slice()
                .iter()
                .filter(|&&c| c != COST_UNKNOWN)
                .count(),
            80
        );
    }

    #[test]
    fn window_off_the_map_is_all_unknown() {
        let window = check(CellRegion::sized(IVec2::new(100, -50), UVec2::splat(5)));
        assert!(window.as_slice().iter().all(|&c| c == COST_UNKNOWN));
    }

    #[test]
    fn window_at_the_ends_of_the_cell_range_is_all_unknown() {
        let map = numbered_map();
        let size = UVec2::new(5, 4);
        for centre in [
            DVec2::new(1.0e8, 1.0e8),
            DVec2::new(-1.0e8, 5.0),
            DVec2::new(1.0e10, -1.0e10),
            DVec2::new(f64::MAX, f64::MIN),
        ] {
            let cells = CellRegion::centered(map.lattice().cell_at(centre), size);
            let window = check(cells);
            assert_eq!(window.size(), size, "{centre}");
            assert!(
                window.as_slice().iter().all(|&c| c == COST_UNKNOWN),
                "{centre}"
            );
        }
    }

    #[test]
    fn a_centred_region_keeps_its_size_at_the_ends_of_the_cell_range() {
        let size = UVec2::new(5, 4);
        let middle = CellRegion::centered(IVec2::new(10, -10), size);
        assert_eq!(middle, CellRegion::sized(IVec2::new(8, -12), size));
        for centre in [IVec2::MAX, IVec2::MIN, IVec2::new(i32::MAX, i32::MIN)] {
            assert_eq!(CellRegion::centered(centre, size).size(), size, "{centre}");
        }
        let top = CellRegion::centered(IVec2::MAX, size);
        assert_eq!(top.max, IVec2::MAX);
        let wide = CellRegion::centered(IVec2::ZERO, UVec2::splat(u32::MAX));
        assert_eq!(wide, CellRegion::new(IVec2::MIN, IVec2::MAX));
    }

    #[test]
    fn a_borrowed_map_and_a_boxed_one_answer_alike() {
        let map = numbered_map();
        let cells = CellRegion::sized(IVec2::new(-2, 5), UVec2::new(3, 3));
        let borrowed = (&map).window(cells).unwrap();
        let mut boxed: Box<dyn CostWindows> = Box::new(map.clone());
        assert_eq!(boxed.lattice(), map.lattice());
        assert_eq!(boxed.window(cells).unwrap(), borrowed);
    }
}
