//! A cost GeoTIFF served a window at a time, with its chunk traffic logged.

use std::path::Path;

use costmap::types::{CellRegion, VoxelError};
use costmap::{ChunkStats, CostWindows, Costmap, GeoTiffWindows, Lattice};
use tracing::{debug, info};

/// Edge of a held chunk, in cells.
const CHUNK_CELLS: u32 = 64;
/// Distance beyond the requested window that is loaded, in metres, so a robot
/// moving less than this between windows does not touch the file.
const LOAD_MARGIN_M: f64 = 5.0;
/// Distance beyond the loaded region that chunks are kept, in metres. Chunks
/// further out are dropped, so a robot oscillating at the edge of the loaded
/// region does not reload the same chunks.
const KEEP_MARGIN_M: f64 = 25.0;

/// A cost GeoTIFF of which only the chunks near the last window are held.
///
/// Windows are on the file's lattice: metres east and north of the raster's
/// lower-left corner. [`GeoTiffWindows`] reads them, loading `LOAD_MARGIN_M`
/// beyond each window and keeping chunks up to `KEEP_MARGIN_M` beyond that.
pub struct GeoTiffSource {
    windows: GeoTiffWindows,
}

impl GeoTiffSource {
    /// Open `path` and log its size and georeference. No cells are read yet.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, VoxelError> {
        let windows = GeoTiffWindows::open(path, CHUNK_CELLS)?;
        let file = windows.file();
        let (size, resolution, georef) = (file.size(), file.resolution(), file.georeference());
        info!(
            "GeoTIFF {}x{} cells at {} m ({:.1} x {:.1} m); map frame origin is the lower-left \
             corner, at {} easting {:.3}, northing {:.3}",
            size.x,
            size.y,
            resolution,
            f64::from(size.x) * resolution,
            f64::from(size.y) * resolution,
            georef.epsg.map_or_else(
                || "projected (no EPSG code)".into(),
                |c| format!("EPSG:{c}")
            ),
            georef.anchor.x,
            georef.anchor.y,
        );
        let cells = |metres: f64| (metres / resolution).ceil() as u32;
        Ok(Self {
            windows: windows.with_margins(cells(LOAD_MARGIN_M), cells(KEEP_MARGIN_M)),
        })
    }

    /// Chunk counters.
    pub fn stats(&self) -> ChunkStats {
        self.windows.stats()
    }
}

impl CostWindows for GeoTiffSource {
    fn lattice(&self) -> Lattice {
        self.windows.lattice()
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        let window = self.windows.window(cells)?;
        let stats = self.stats();
        debug!(
            "chunks loaded {}, evicted {}, held {}",
            stats.loaded, stats.evicted, stats.held
        );
        Ok(window)
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
    use costmap::types::COST_UNKNOWN;
    use costmap::{GeoReference, TiledGrid, write_cost_geotiff_region};
    use glam::{DVec2, IVec2, UVec2};

    use super::*;
    use crate::source::window_around;

    const RESOLUTION: f64 = 0.5;
    const MAP_CELLS: UVec2 = UVec2::new(1000, 200);
    const WINDOW: UVec2 = UVec2::new(20, 20);

    fn pattern(x: i32, y: i32) -> u8 {
        ((x * 7 + y * 13).rem_euclid(200)) as u8
    }

    fn open_pattern_map(dir: &tempfile::TempDir) -> GeoTiffSource {
        let mut grid = TiledGrid::new(RESOLUTION, CHUNK_CELLS, COST_UNKNOWN).unwrap();
        for y in 0..MAP_CELLS.y as i32 {
            for x in 0..MAP_CELLS.x as i32 {
                grid.set(IVec2::new(x, y), pattern(x, y));
            }
        }
        let georef = GeoReference {
            epsg: Some(32630),
            anchor: DVec2::new(500_000.0, 6_200_000.0),
        };
        let path = dir.path().join("map.tif");
        write_cost_geotiff_region(&path, &grid, &georef, IVec2::ZERO, MAP_CELLS.as_ivec2())
            .unwrap();
        GeoTiffSource::open(path).unwrap()
    }

    fn world(cell_x: f64, cell_y: f64) -> DVec2 {
        DVec2::new(cell_x, cell_y) * RESOLUTION
    }

    #[test]
    fn a_window_lies_on_the_file_lattice() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        assert_eq!(source.lattice(), Lattice::new(DVec2::ZERO, RESOLUTION));
        let window = window_around(&mut source, world(300.5, 100.5), WINDOW).unwrap();
        assert_eq!(window.world_bounds().min, DVec2::new(145.0, 45.0));
        assert_eq!(window.get(IVec2::new(300, 100)), Some(&pattern(300, 100)));
    }

    #[test]
    fn a_move_within_the_load_margin_loads_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        window_around(&mut source, world(300.5, 100.5), WINDOW).unwrap();
        let before = source.stats();
        assert!(before.loaded > 0);
        // 4.5 m east and north: inside the 5 m load margin.
        window_around(&mut source, world(309.5, 109.5), WINDOW).unwrap();
        assert_eq!(source.stats(), before);
    }
}
