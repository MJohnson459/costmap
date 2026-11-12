//! Sources of costmap windows.

use std::ops::Range;

use costmap::types::{COST_UNKNOWN, VoxelError};
use costmap::{Costmap, MapInfo};
use glam::{DVec2, I64Vec2, UVec2};
use thiserror::Error;

mod geotiff;

pub use geotiff::{GeoTiffSource, TileStats};

/// A map could not supply a window.
#[derive(Debug, Error)]
#[error("reading map data: {0}")]
pub struct SourceError(#[from] pub VoxelError);

/// A map that hands out rectangular windows of itself.
///
/// Implementations place every window on their own cell grid, so a cell keeps
/// its world position and value from one window to the next.
pub trait MapSource {
    /// Window of `size_cells` cells around `center` (world metres).
    ///
    /// Cells with no data are `COST_UNKNOWN`. Fails if the backing store
    /// cannot be read.
    fn window(&mut self, center: DVec2, size_cells: UVec2) -> Result<Costmap, SourceError>;

    /// Cell size in metres.
    fn resolution(&self) -> f64;
}

/// A dense costmap held in memory; cells outside it are `COST_UNKNOWN`.
impl MapSource for Costmap {
    /// A window that does not touch the map, however far away its centre, is all
    /// unknown. Fails if `center` is not finite.
    fn window(&mut self, center: DVec2, size_cells: UVec2) -> Result<Costmap, SourceError> {
        let map = *self.info();
        let cells = WindowCells::around(center, map.origin, map.resolution, size_cells)?;
        let mut window = cells.unknown_window(map.origin, map.resolution);
        copy_overlap(self, &mut window, cells.min);
        Ok(window)
    }

    fn resolution(&self) -> f64 {
        self.info().resolution
    }
}

/// Largest magnitude, in cells, of a window centre. Farther centres are clamped to it,
/// which keeps every cell-rectangle sum far from `i64` overflow.
const MAX_CENTER_CELL: i64 = 1 << 40;

/// The cells of a window in the cell coordinates of the map it is cut from, where cell
/// (0, 0) has its lower-left corner at the map's origin. Computed in `i64`, so a window
/// may lie far outside the map.
#[derive(Debug, Clone, Copy, PartialEq)]
struct WindowCells {
    /// The cell holding the requested centre, clamped to [`MAX_CENTER_CELL`].
    center: I64Vec2,
    /// First cell of the window, inclusive.
    min: I64Vec2,
    /// Last cell of the window, exclusive.
    max: I64Vec2,
}

impl WindowCells {
    /// The window of `size` cells whose middle cell contains `center`, on the grid of
    /// cells of `resolution` metres whose cell (0, 0) has its lower-left corner at
    /// `origin`. Fails if `center` is not finite.
    fn around(
        center: DVec2,
        origin: DVec2,
        resolution: f64,
        size: UVec2,
    ) -> Result<Self, SourceError> {
        if !center.is_finite() {
            return Err(SourceError(VoxelError::InvalidArgument(format!(
                "window centre {center} is not finite"
            ))));
        }
        let limit = I64Vec2::splat(MAX_CENTER_CELL);
        let center = ((center - origin) / resolution)
            .floor()
            .as_i64vec2()
            .clamp(-limit, limit);
        let min = center - (size / 2).as_i64vec2();
        Ok(Self {
            center,
            min,
            max: min + size.as_i64vec2(),
        })
    }

    /// Whether the window shares a cell with a map of `size` cells.
    fn overlaps(&self, size: UVec2) -> bool {
        self.min.cmplt(size.as_i64vec2()).all() && self.max.cmpgt(I64Vec2::ZERO).all()
    }

    /// An all-unknown costmap covering the window, on the grid described at
    /// [`around`](Self::around).
    fn unknown_window(&self, origin: DVec2, resolution: f64) -> Costmap {
        let size = (self.max - self.min).as_uvec2();
        let info = MapInfo {
            width: size.x,
            height: size.y,
            resolution,
            origin: origin + self.min.as_dvec2() * resolution,
        };
        Costmap::filled(info, COST_UNKNOWN)
    }
}

/// Copies the cells `map` and `window` share, one row slice at a time.
/// `lower_left` is the window's cell (0, 0) in map cell coordinates.
fn copy_overlap(map: &Costmap, window: &mut Costmap, lower_left: I64Vec2) {
    let (Some(cols), Some(rows)) = (
        overlap(lower_left.x, window.size().x, map.size().x),
        overlap(lower_left.y, window.size().y, map.size().y),
    ) else {
        return;
    };
    let map_width = map.size().x as usize;
    let window_width = window.size().x as usize;
    // Offsets of the overlap's first column and row inside the window.
    let col_offset = (cols.start as i64 - lower_left.x) as usize;
    let row_offset = (rows.start as i64 - lower_left.y) as usize;

    let src_rows = map.as_slice().chunks_exact(map_width).skip(rows.start);
    let dst_rows = window
        .as_mut_slice()
        .chunks_exact_mut(window_width)
        .skip(row_offset);
    for (src, dst) in src_rows.zip(dst_rows).take(rows.len()) {
        dst[col_offset..col_offset + cols.len()].copy_from_slice(&src[cols.clone()]);
    }
}

/// Map indices covered by `len` cells starting at map index `start`, clipped
/// to `0..limit`. `None` if nothing overlaps.
fn overlap(start: i64, len: u32, limit: u32) -> Option<Range<usize>> {
    let lo = start.max(0);
    let hi = (start + i64::from(len)).min(i64::from(limit));
    (lo < hi).then_some(lo as usize..hi as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 10 x 8 map at 0.5 m whose cell (x, y) holds `10 * y + x`.
    fn numbered_map() -> Costmap {
        let info = MapInfo {
            width: 10,
            height: 8,
            resolution: 0.5,
            origin: DVec2::new(-2.0, 1.0),
        };
        Costmap::from_fn(info, |x, y| (10 * y + x) as u8)
    }

    /// Value of the map cell under the centre of each window cell, or
    /// `COST_UNKNOWN` where the window lies outside the map.
    fn expected_by_world_position(map: &Costmap, window: &Costmap) -> Vec<u8> {
        window
            .indexed_iter()
            .map(|(cell, _)| {
                let world = window.map_to_world(cell);
                map.world_to_map(world)
                    .and_then(|m| map.get(m).copied())
                    .unwrap_or(COST_UNKNOWN)
            })
            .collect()
    }

    #[test]
    fn interior_window_copies_cells_at_their_world_position() {
        let mut map = numbered_map();
        let window = map.window(DVec2::new(0.3, 2.9), UVec2::new(4, 3)).unwrap();
        assert_eq!(window.info().resolution, 0.5);
        assert_eq!(window.size(), UVec2::new(4, 3));
        // centre cell (4, 3); lower-left (2, 2)
        assert_eq!(window.info().origin, DVec2::new(-1.0, 2.0));
        assert_eq!(
            window.as_slice(),
            expected_by_world_position(&map, &window).as_slice()
        );
        assert_eq!(window.get(UVec2::ZERO), Some(&22));
    }

    #[test]
    fn window_over_the_edge_is_unknown_outside_the_map() {
        let mut map = numbered_map();
        let window = map.window(DVec2::new(-2.2, 4.8), UVec2::new(6, 6)).unwrap();
        let expected = expected_by_world_position(&map, &window);
        assert_eq!(window.as_slice(), expected.as_slice());
        assert!(expected.contains(&COST_UNKNOWN));
        assert!(expected.iter().any(|&c| c != COST_UNKNOWN));
    }

    #[test]
    fn window_off_the_map_is_all_unknown() {
        let mut map = numbered_map();
        let window = map
            .window(DVec2::new(100.0, -50.0), UVec2::new(5, 5))
            .unwrap();
        assert!(window.as_slice().iter().all(|&c| c == COST_UNKNOWN));
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
            let window = map.window(centre, size).unwrap();
            assert_eq!(window.size(), UVec2::new(5, 4), "{centre}");
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
            assert!(map.window(centre, UVec2::ONE).is_err(), "{centre}");
        }
    }

    #[test]
    fn window_larger_than_the_map_contains_all_of_it() {
        let mut map = numbered_map();
        let window = map
            .window(map.info().world_center(), UVec2::new(30, 30))
            .unwrap();
        let known = window
            .as_slice()
            .iter()
            .filter(|&&c| c != COST_UNKNOWN)
            .count();
        assert_eq!(known, 80);
        assert_eq!(
            window.as_slice(),
            expected_by_world_position(&map, &window).as_slice()
        );
    }

    #[test]
    fn origin_snaps_to_the_map_grid() {
        let mut map = numbered_map();
        let a = map
            .window(DVec2::new(0.51, 2.01), UVec2::new(4, 4))
            .unwrap();
        let b = map
            .window(DVec2::new(0.99, 2.49), UVec2::new(4, 4))
            .unwrap();
        assert_eq!(a.info(), b.info());
        let offset = (a.info().origin - map.info().origin) / map.info().resolution;
        assert_eq!(offset, offset.round());
    }

    #[test]
    fn trait_is_object_safe() {
        let mut source: Box<dyn MapSource> = Box::new(numbered_map());
        assert_eq!(source.resolution(), 0.5);
        assert_eq!(source.window(DVec2::ZERO, UVec2::ONE).unwrap().size().x, 1);
    }
}
