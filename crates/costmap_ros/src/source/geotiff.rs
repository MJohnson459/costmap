//! A map read on demand from a cost GeoTIFF.

use std::path::Path;

use costmap::types::{COST_UNKNOWN, VoxelError};
use costmap::{CostGeoTiff, Costmap, TiledGrid};
use glam::{DVec2, I64Vec2, IVec2, UVec2};
use tracing::{debug, info};

use super::{MapSource, SourceError, WindowCells};

/// Edge of a held chunk, in cells.
const CHUNK_CELLS: u32 = 64;
/// Distance beyond the requested window that is loaded, in metres, so a robot
/// moving less than this between windows does not touch the file.
const LOAD_MARGIN_M: f64 = 5.0;
/// Distance beyond the loaded region that chunks are kept, in metres. Chunks
/// further out are dropped, so a robot oscillating at the edge of the loaded
/// region does not reload the same chunks.
const EVICT_MARGIN_M: f64 = 25.0;

/// Chunk traffic of a [`GeoTiffSource`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileStats {
    /// Chunks read from the file since the source was opened.
    pub loaded: u64,
    /// Chunks dropped from memory since the source was opened.
    pub evicted: u64,
    /// Chunks in memory now.
    pub held: usize,
}

/// Rectangle of cells, `min` inclusive and `max` exclusive.
#[derive(Debug, Clone, Copy)]
struct CellRect {
    min: IVec2,
    max: IVec2,
}

impl CellRect {
    fn contains(&self, other: &CellRect) -> bool {
        self.min.cmple(other.min).all() && self.max.cmpge(other.max).all()
    }

    fn grown(&self, cells: i32) -> CellRect {
        CellRect {
            min: self.min - IVec2::splat(cells),
            max: self.max + IVec2::splat(cells),
        }
    }
}

/// A cost GeoTIFF of which only the chunks near the requested window are held.
///
/// Windows are placed on the file's cell grid in its local frame: metres east
/// and north of the raster's lower-left corner. Cells outside the raster are
/// `COST_UNKNOWN`.
///
/// A window not inside the region loaded last time loads the chunks under the
/// window plus `LOAD_MARGIN_M` on every side, after dropping chunks more than
/// `EVICT_MARGIN_M` beyond that region. Windows inside the loaded region do
/// not read the file.
pub struct GeoTiffSource {
    file: CostGeoTiff,
    grid: TiledGrid<u8>,
    loaded_region: Option<CellRect>,
    loaded: u64,
    evicted: u64,
}

impl GeoTiffSource {
    /// Open `path` and log its size and georeference. No cells are read yet.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, SourceError> {
        let file = CostGeoTiff::open(path)?;
        let grid = TiledGrid::new(file.resolution(), CHUNK_CELLS, COST_UNKNOWN)?;
        let size = file.size();
        let georef = file.georeference();
        info!(
            "GeoTIFF {}x{} cells at {} m ({:.1} x {:.1} m); map frame origin is the lower-left \
             corner, at {} easting {:.3}, northing {:.3}",
            size.x,
            size.y,
            file.resolution(),
            f64::from(size.x) * file.resolution(),
            f64::from(size.y) * file.resolution(),
            georef.epsg.map_or_else(
                || "projected (no EPSG code)".into(),
                |c| format!("EPSG:{c}")
            ),
            georef.anchor.x,
            georef.anchor.y,
        );
        Ok(Self {
            file,
            grid,
            loaded_region: None,
            loaded: 0,
            evicted: 0,
        })
    }

    /// Chunk counters.
    pub fn stats(&self) -> TileStats {
        TileStats {
            loaded: self.loaded,
            evicted: self.evicted,
            held: self.grid.loaded_chunks().count(),
        }
    }

    fn cells_in(&self, metres: f64) -> i32 {
        (metres / self.grid.resolution()).ceil() as i32
    }

    /// Drop distant chunks, then load the chunks under `wanted` plus the load margin.
    fn refill(&mut self, center_cell: IVec2, wanted: CellRect) -> Result<(), SourceError> {
        let region = wanted.grown(self.cells_in(LOAD_MARGIN_M));
        // Square around the centre that holds `region` plus the evict margin.
        let reach = (region.max - region.min).max_element() / 2 + 1;
        let radius = reach + self.cells_in(EVICT_MARGIN_M);
        // Cleared first: if the load fails, the evicted chunks must not still count as loaded.
        self.loaded_region = None;
        self.evicted += self.grid.evict_outside(center_cell, radius.unsigned_abs()) as u64;
        self.loaded += self
            .file
            .load_region(&mut self.grid, region.min, region.max)? as u64;
        self.loaded_region = Some(region);
        Ok(())
    }
}

impl MapSource for GeoTiffSource {
    /// A window that does not touch the raster, however far away its centre, is all
    /// unknown and reads no chunks. Fails if `center` is not finite.
    fn window(&mut self, center: DVec2, size_cells: UVec2) -> Result<Costmap, SourceError> {
        let resolution = self.grid.resolution();
        let cells = WindowCells::around(center, DVec2::ZERO, resolution, size_cells)?;
        if !cells.overlaps(self.file.size()) {
            return Ok(cells.unknown_window(DVec2::ZERO, resolution));
        }
        let to_cell = |cell: I64Vec2| {
            IVec2::try_from(cell).map_err(|_| {
                SourceError(VoxelError::InvalidArgument(format!(
                    "window of {size_cells} cells around {center} is too large"
                )))
            })
        };
        let wanted = CellRect {
            min: to_cell(cells.min)?,
            max: to_cell(cells.max)?,
        };
        let (min, center_cell) = (wanted.min, to_cell(cells.center)?);
        if !self.loaded_region.is_some_and(|r| r.contains(&wanted)) {
            self.refill(center_cell, wanted)?;
        }
        let stats = self.stats();
        debug!(
            "chunks loaded {}, evicted {}, held {}",
            stats.loaded, stats.evicted, stats.held
        );
        Ok(self.grid.window(min, size_cells))
    }

    fn resolution(&self) -> f64 {
        self.grid.resolution()
    }
}

impl Drop for GeoTiffSource {
    fn drop(&mut self) {
        let stats = self.stats();
        info!(
            "GeoTIFF chunks: {} loaded, {} evicted, {} held",
            stats.loaded, stats.evicted, stats.held
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::{GeoReference, write_cost_geotiff_region};

    const RESOLUTION: f64 = 0.5;
    const MAP_CELLS: UVec2 = UVec2::new(1000, 200);
    const WINDOW: UVec2 = UVec2::new(20, 20);
    const EPSG: u32 = 32630;
    const ANCHOR: DVec2 = DVec2::new(500_000.0, 6_200_000.0);

    /// Asymmetric in x and y, never `COST_UNKNOWN`.
    fn pattern(x: i32, y: i32) -> u8 {
        ((x * 7 + y * 13).rem_euclid(200)) as u8
    }

    /// Writes the pattern as a GeoTIFF and opens it.
    fn open_pattern_map(dir: &tempfile::TempDir) -> GeoTiffSource {
        let mut grid = TiledGrid::new(RESOLUTION, CHUNK_CELLS, COST_UNKNOWN).unwrap();
        for y in 0..MAP_CELLS.y as i32 {
            for x in 0..MAP_CELLS.x as i32 {
                grid.set(IVec2::new(x, y), pattern(x, y));
            }
        }
        let georef = GeoReference {
            epsg: Some(EPSG),
            anchor: ANCHOR,
        };
        let path = dir.path().join("map.tif");
        write_cost_geotiff_region(&path, &grid, &georef, IVec2::ZERO, MAP_CELLS.as_ivec2())
            .unwrap();
        GeoTiffSource::open(path).unwrap()
    }

    /// Pattern value under the centre of every window cell, or unknown off the map.
    fn expected(window: &Costmap) -> Vec<u8> {
        let info = window.info();
        let first = (info.origin / info.resolution).round().as_ivec2();
        window
            .indexed_iter()
            .map(|(cell, _)| {
                let c = first + cell.as_ivec2();
                let inside =
                    c.x >= 0 && c.y >= 0 && c.x < MAP_CELLS.x as i32 && c.y < MAP_CELLS.y as i32;
                if inside {
                    pattern(c.x, c.y)
                } else {
                    COST_UNKNOWN
                }
            })
            .collect()
    }

    fn world(cell_x: f64, cell_y: f64) -> DVec2 {
        DVec2::new(cell_x, cell_y) * RESOLUTION
    }

    #[test]
    fn interior_window_matches_the_pattern_at_its_world_position() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        let window = source.window(world(300.5, 100.5), WINDOW).unwrap();
        // centre cell (300, 100); lower-left (290, 90)
        assert_eq!(window.info().origin, DVec2::new(145.0, 45.0));
        assert_eq!(window.info().resolution, RESOLUTION);
        assert_eq!(window.as_slice(), expected(&window).as_slice());
        assert_eq!(window.get(UVec2::ZERO), Some(&pattern(290, 90)));
    }

    #[test]
    fn failed_load_does_not_leave_evicted_chunks_marked_loaded() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        let near = world(100.5, 100.5);
        source.window(near, WINDOW).unwrap();
        std::fs::File::create(dir.path().join("map.tif")).unwrap();
        assert!(source.window(world(900.5, 100.5), WINDOW).is_err());
        assert!(source.window(near, WINDOW).is_err());
    }

    #[test]
    fn window_over_the_edge_is_unknown_outside() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        let window = source.window(world(2.5, 197.5), WINDOW).unwrap();
        let want = expected(&window);
        assert!(want.contains(&COST_UNKNOWN));
        assert!(want.iter().any(|&c| c != COST_UNKNOWN));
        assert_eq!(window.as_slice(), want.as_slice());
    }

    #[test]
    fn held_chunks_stay_bounded_and_a_return_trip_gives_the_same_window() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        let start = world(100.5, 100.5);
        let first = source.window(start, WINDOW).unwrap();
        let mut most_held = 0;
        for x in (100..900).step_by(10) {
            let window = source
                .window(world(f64::from(x) + 0.5, 100.5), WINDOW)
                .unwrap();
            assert_eq!(window.as_slice(), expected(&window).as_slice());
            most_held = most_held.max(source.stats().held);
        }
        let total_chunks =
            (MAP_CELLS.x.div_ceil(CHUNK_CELLS) * MAP_CELLS.y.div_ceil(CHUNK_CELLS)) as usize;
        assert!(
            most_held * 3 < total_chunks,
            "held {most_held} of {total_chunks}"
        );
        assert!(source.stats().evicted > 0);

        let again = source.window(start, WINDOW).unwrap();
        assert_eq!(again.info(), first.info());
        assert_eq!(again.as_slice(), first.as_slice());
    }

    #[test]
    fn a_one_cell_move_loads_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        source.window(world(300.5, 100.5), WINDOW).unwrap();
        let before = source.stats();
        assert!(before.loaded > 0);
        source.window(world(301.5, 100.5), WINDOW).unwrap();
        source.window(world(301.5, 101.5), WINDOW).unwrap();
        assert_eq!(source.stats(), before);
    }

    #[test]
    fn a_centre_far_outside_the_map_gives_an_unknown_window() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        let centres = [
            DVec2::new(1.0e8, 1.0e8),
            DVec2::new(-1.0e8, 5.0),
            DVec2::new(1.0e10, -1.0e10),
            DVec2::new(f64::MAX, f64::MIN),
        ];
        for centre in centres {
            let window = source.window(centre, WINDOW).unwrap();
            assert_eq!(window.info().width, WINDOW.x, "{centre}");
            assert_eq!(window.info().height, WINDOW.y, "{centre}");
            assert!(
                window.as_slice().iter().all(|&c| c == COST_UNKNOWN),
                "{centre}"
            );
        }
        assert_eq!(source.stats().loaded, 0);

        let window = source.window(world(300.5, 100.5), WINDOW).unwrap();
        assert_eq!(window.as_slice(), expected(&window).as_slice());
    }

    #[test]
    fn a_non_finite_centre_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        for centre in [
            DVec2::new(f64::NAN, 5.0),
            DVec2::new(5.0, f64::INFINITY),
            DVec2::new(f64::NEG_INFINITY, f64::NAN),
        ] {
            assert!(source.window(centre, WINDOW).is_err(), "{centre}");
        }
        assert_eq!(source.stats().loaded, 0);
    }

    #[test]
    fn resolution_comes_from_the_file() {
        let dir = tempfile::tempdir().unwrap();
        assert_eq!(open_pattern_map(&dir).resolution(), RESOLUTION);
    }
}
