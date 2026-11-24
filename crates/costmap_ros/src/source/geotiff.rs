//! A cost GeoTIFF served a window at a time, with its chunk traffic logged.

use std::path::Path;

use costmap::types::{CellRegion, CostmapError};
use costmap::{ChunkStats, CostWindows, Costmap, GeoTiffReading, GeoTiffWindows, Lattice, Margin};
use tracing::{debug, info};

/// A cost GeoTIFF of which only the chunks near the last window are held.
///
/// Windows are on the file's lattice: metres east and north of the raster's
/// lower-left corner. [`GeoTiffWindows`] reads them, as [`READING`](Self::READING)
/// sets out.
pub struct GeoTiffSource {
    windows: GeoTiffWindows,
}

impl GeoTiffSource {
    /// 64-cell chunks. A robot moving less than the 5 m load margin between
    /// windows does not touch the file, and chunks within 25 m of the loaded
    /// region are kept, so a robot oscillating at its edge does not reload them.
    pub const READING: GeoTiffReading = GeoTiffReading {
        chunk_cells: 64,
        load_margin: Margin::Metres(5.0),
        keep_margin: Margin::Metres(25.0),
    };

    /// Open `path` as [`READING`](Self::READING) sets out, and log its size and
    /// georeference. No cells are read yet.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, CostmapError> {
        Ok(Self::new(Self::READING.open(path)?))
    }

    /// Serve windows from `windows`, and log its size and georeference.
    pub fn new(windows: GeoTiffWindows) -> Self {
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
        Self { windows }
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

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, CostmapError> {
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

    const RESOLUTION: f64 = 0.5;
    const MAP_CELLS: UVec2 = UVec2::new(1000, 200);
    const WINDOW: UVec2 = UVec2::new(20, 20);

    fn pattern(x: i32, y: i32) -> u8 {
        ((x * 7 + y * 13).rem_euclid(200)) as u8
    }

    fn open_pattern_map(dir: &tempfile::TempDir) -> GeoTiffSource {
        let mut grid =
            TiledGrid::new(RESOLUTION, GeoTiffSource::READING.chunk_cells, COST_UNKNOWN).unwrap();
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
        let window = source.window_around(world(300.5, 100.5), WINDOW).unwrap();
        assert_eq!(window.world_bounds().min, DVec2::new(145.0, 45.0));
        assert_eq!(window.get(IVec2::new(300, 100)), Some(&pattern(300, 100)));
    }

    #[test]
    fn a_move_within_the_load_margin_loads_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let mut source = open_pattern_map(&dir);
        source.window_around(world(300.5, 100.5), WINDOW).unwrap();
        let before = source.stats();
        assert!(before.loaded > 0);
        // 4.5 m east and north: inside the 5 m load margin.
        source.window_around(world(309.5, 109.5), WINDOW).unwrap();
        assert_eq!(source.stats(), before);
    }
}
