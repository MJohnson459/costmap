//! The one request every cost map answers: the cells of a rectangle as a dense window.

use glam::{DVec2, IVec2, UVec2};

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

    /// The window of `size` cells whose middle cell holds `center`, a position in
    /// the lattice's frame.
    ///
    /// A centre beyond the `i32` cell range gives the window at the edge of that
    /// range, which is all unknown. Fails if `center` is not finite or the map
    /// cannot be read.
    fn window_around(&mut self, center: DVec2, size: UVec2) -> Result<Costmap, VoxelError> {
        if !center.is_finite() {
            return Err(VoxelError::InvalidArgument(format!(
                "window centre {center} is not finite"
            )));
        }
        let cell = self.lattice().cell_at(center);
        self.window(CellRegion::centered(cell, size))
    }
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

/// A map whose windows come inflated.
///
/// Each window is read from the map with a margin as wide as inflation reaches,
/// so every obstacle that raises a cost in the window is in what is read, then
/// inflated as [`Costmap::inflate`] inflates and cut back to the cells asked for.
/// A window of an inflated map is therefore the same cells as the matching window
/// of the whole map inflated at once.
///
/// ```
/// use costmap::{CostWindows, Costmap, InflatedWindows, Lattice, types::{COST_FREE, COST_LETHAL, CellRegion}};
/// use glam::{DVec2, IVec2, UVec2};
///
/// let lattice = Lattice::new(DVec2::ZERO, 0.1);
/// let map = Costmap::from_fn(lattice, CellRegion::from_size(UVec2::splat(40)), |cell| {
///     if cell == IVec2::new(20, 20) { COST_LETHAL } else { COST_FREE }
/// });
/// let mut whole = map.clone();
/// whole.inflate(0.5, 0.2, 2.0);
///
/// let mut inflated = InflatedWindows::new(map, 0.5, 0.2, 2.0);
/// let cells = CellRegion::sized(IVec2::new(22, 18), UVec2::splat(4));
/// assert_eq!(inflated.window(cells).unwrap(), (&whole).window(cells).unwrap());
/// ```
pub struct InflatedWindows<M> {
    map: M,
    /// Cells read beyond each edge of a window.
    margin: u32,
    radius_m: f32,
    inscribed_radius_m: f32,
    cost_scaling_factor: f32,
}

impl<M: CostWindows> InflatedWindows<M> {
    /// Inflate the windows of `map` as [`Costmap::inflate`] does with these
    /// arguments. Cells within `inscribed_radius_m` of an obstacle are inscribed
    /// even where `radius_m` is smaller.
    pub fn new(map: M, radius_m: f32, inscribed_radius_m: f32, cost_scaling_factor: f32) -> Self {
        let radius_m = radius_m.max(inscribed_radius_m);
        let resolution = map.lattice().resolution;
        Self {
            margin: (f64::from(radius_m) / resolution).ceil() as u32 + 1,
            map,
            radius_m,
            inscribed_radius_m,
            cost_scaling_factor,
        }
    }

    /// The map the windows are read from.
    pub fn map(&self) -> &M {
        &self.map
    }
}

impl<M: CostWindows> CostWindows for InflatedWindows<M> {
    fn lattice(&self) -> Lattice {
        self.map.lattice()
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        let mut costs = self.map.window(cells.grown(self.margin))?;
        costs.inflate(
            self.radius_m,
            self.inscribed_radius_m,
            self.cost_scaling_factor,
        );
        (&costs).window(cells)
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

    #[test]
    fn a_window_around_a_position_is_centred_on_the_cell_holding_it() {
        let mut map = numbered_map();
        // Inside cell (2, 6); the 4 x 3 window then starts at (0, 5).
        let window = map
            .window_around(DVec2::new(-0.7, 4.4), UVec2::new(4, 3))
            .unwrap();
        assert_eq!(
            window.cells(),
            CellRegion::sized(IVec2::new(0, 5), UVec2::new(4, 3))
        );
        assert_eq!(window.get(IVec2::new(0, 5)), Some(&13));
        let same_cell = map
            .window_around(DVec2::new(-0.51, 4.26), UVec2::new(4, 3))
            .unwrap();
        assert_eq!(same_cell, window);
    }

    #[test]
    fn a_window_around_a_far_position_is_unknown_and_a_non_finite_one_is_an_error() {
        let mut map: Box<dyn CostWindows> = Box::new(numbered_map());
        let size = UVec2::new(5, 4);
        for centre in [DVec2::new(1.0e10, -1.0e10), DVec2::new(f64::MAX, f64::MIN)] {
            let window = map.window_around(centre, size).unwrap();
            assert_eq!(window.size(), size, "{centre}");
            assert!(window.as_slice().iter().all(|&c| c == COST_UNKNOWN));
        }
        for centre in [
            DVec2::new(f64::NAN, 5.0),
            DVec2::new(5.0, f64::INFINITY),
            DVec2::new(f64::NEG_INFINITY, f64::NAN),
        ] {
            assert!(map.window_around(centre, UVec2::ONE).is_err(), "{centre}");
        }
    }

    #[test]
    fn an_inflated_window_matches_the_whole_map_inflated() {
        use crate::types::{COST_FREE, COST_LETHAL};
        let lattice = Lattice::new(DVec2::new(-2.0, 1.0), 0.1);
        let cells = CellRegion::sized(IVec2::new(-30, 10), UVec2::new(60, 50));
        let map = Costmap::from_fn(lattice, cells, |c| {
            let wall = c.x == 0 && c.y < 40;
            let post = c == IVec2::new(-20, 50) || c == IVec2::new(25, 15);
            if wall || post { COST_LETHAL } else { COST_FREE }
        });
        for (radius_m, inscribed_m) in [(0.8f32, 0.3f32), (0.2, 0.5), (0.0, 0.0)] {
            let mut whole = map.clone();
            whole.inflate(radius_m.max(inscribed_m), inscribed_m, 2.0);
            let mut inflated = InflatedWindows::new(map.clone(), radius_m, inscribed_m, 2.0);
            for window in [
                CellRegion::sized(IVec2::new(-4, 30), UVec2::new(8, 8)),
                CellRegion::sized(IVec2::new(-35, 5), UVec2::new(12, 9)),
                CellRegion::sized(IVec2::new(20, 52), UVec2::new(16, 16)),
                cells,
            ] {
                assert_eq!(
                    inflated.window(window).unwrap(),
                    (&whole).window(window).unwrap(),
                    "{window:?} at {radius_m} m"
                );
            }
        }
    }
}
