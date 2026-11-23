//! Windows of the served map around the robot.

use costmap::types::{CellRegion, VoxelError};
use costmap::{CostWindows, Costmap};
use glam::{DVec2, UVec2};

mod geotiff;

pub use geotiff::GeoTiffSource;

/// The window of `size` cells of `map` whose middle cell holds `center`, in the
/// map frame.
///
/// A centre beyond the `i32` cell range gives the window at the edge of that range,
/// which is all unknown. Fails if `center` is not finite or `map` cannot be read.
pub fn window_around(
    map: &mut (impl CostWindows + ?Sized),
    center: DVec2,
    size: UVec2,
) -> Result<Costmap, VoxelError> {
    if !center.is_finite() {
        return Err(VoxelError::InvalidArgument(format!(
            "window centre {center} is not finite"
        )));
    }
    let cell = map.lattice().cell_at(center);
    map.window(CellRegion::centered(cell, size))
}

#[cfg(test)]
mod tests {
    use costmap::Lattice;
    use costmap::types::COST_UNKNOWN;
    use glam::IVec2;

    use super::*;

    /// 10 x 8 map at 0.5 m whose cell (x, y) holds `10 * y + x`.
    fn numbered_map() -> Costmap {
        let lattice = Lattice::new(DVec2::new(-2.0, 1.0), 0.5);
        let cells = CellRegion::from_size(UVec2::new(10, 8));
        Costmap::from_fn(lattice, cells, |cell| (10 * cell.y + cell.x) as u8)
    }

    #[test]
    fn the_window_is_centred_on_the_cell_holding_the_centre() {
        let mut map = numbered_map();
        let window = window_around(&mut map, DVec2::new(0.3, 2.9), UVec2::new(4, 3)).unwrap();
        // centre cell (4, 3); lower-left (2, 2)
        assert_eq!(window.cells().min, IVec2::new(2, 2));
        assert_eq!(window.world_bounds().min, DVec2::new(-1.0, 2.0));
        assert_eq!(window.get(IVec2::new(2, 2)), Some(&22));
    }

    #[test]
    fn a_centre_far_outside_the_map_gives_an_unknown_window() {
        let mut map = numbered_map();
        let size = UVec2::new(5, 4);
        for centre in [
            DVec2::new(1.0e8, 1.0e8),
            DVec2::new(-1.0e8, 5.0),
            DVec2::new(1.0e10, -1.0e10),
            DVec2::new(f64::MAX, f64::MIN),
        ] {
            let window = window_around(&mut map, centre, size).unwrap();
            assert_eq!(window.size(), size, "{centre}");
            assert!(
                window.as_slice().iter().all(|&c| c == COST_UNKNOWN),
                "{centre}"
            );
        }
    }

    #[test]
    fn a_non_finite_centre_is_an_error() {
        let mut map = numbered_map();
        for centre in [
            DVec2::new(f64::NAN, 5.0),
            DVec2::new(5.0, f64::INFINITY),
            DVec2::new(f64::NEG_INFINITY, f64::NAN),
        ] {
            assert!(
                window_around(&mut map, centre, UVec2::ONE).is_err(),
                "{centre}"
            );
        }
    }

    #[test]
    fn nearby_centres_in_one_cell_give_the_same_window() {
        let mut map = numbered_map();
        let a = window_around(&mut map, DVec2::new(0.51, 2.01), UVec2::new(4, 4)).unwrap();
        let b = window_around(&mut map, DVec2::new(0.99, 2.49), UVec2::new(4, 4)).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn a_boxed_map_answers() {
        let mut map: Box<dyn CostWindows> = Box::new(numbered_map());
        let window = window_around(&mut *map, DVec2::ZERO, UVec2::ONE).unwrap();
        assert_eq!(window.size(), UVec2::ONE);
    }
}
