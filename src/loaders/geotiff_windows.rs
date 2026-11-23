//! Windows of a cost GeoTIFF, read on demand.

use std::path::Path;

use crate::{
    CostGeoTiff, CostWindows, Costmap, TiledGrid,
    types::{COST_UNKNOWN, CellRegion, Lattice, VoxelError},
};

/// Chunk traffic of a [`GeoTiffWindows`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkStats {
    /// Chunks read from the file since the reader was made.
    pub loaded: u64,
    /// Chunks dropped from memory since the reader was made.
    pub evicted: u64,
    /// Chunks in memory now.
    pub held: usize,
}

/// A cost GeoTIFF of which only the chunks near the last window are held.
///
/// Windows are on the file's lattice: cell (0, 0) is the raster's lower-left pixel,
/// anchored at the origin of its local frame. Cells outside the raster are
/// [`COST_UNKNOWN`].
///
/// A window not inside the region loaded last time drops the chunks more than
/// `keep` cells beyond the new region, then loads the chunks under the window plus
/// `load` cells on every side (see [`with_margins`](Self::with_margins)). Windows
/// inside the loaded region do not read the file, and a window that does not touch
/// the raster reads nothing.
pub struct GeoTiffWindows {
    file: CostGeoTiff,
    chunks: TiledGrid<u8>,
    load_margin: u32,
    keep_margin: u32,
    loaded_region: Option<CellRegion>,
    loaded: u64,
    evicted: u64,
}

impl GeoTiffWindows {
    /// Read windows of `file` through chunks of `chunk_cells` cells a side, with no
    /// margins. Fails if `chunk_cells` is not a valid chunk size.
    pub fn new(file: CostGeoTiff, chunk_cells: u32) -> Result<Self, VoxelError> {
        let chunks = TiledGrid::new(file.resolution(), chunk_cells, COST_UNKNOWN)?;
        Ok(Self {
            file,
            chunks,
            load_margin: 0,
            keep_margin: 0,
            loaded_region: None,
            loaded: 0,
            evicted: 0,
        })
    }

    /// Open the GeoTIFF at `path`; see [`new`](Self::new). No cells are read yet.
    pub fn open(path: impl AsRef<Path>, chunk_cells: u32) -> Result<Self, VoxelError> {
        Self::new(CostGeoTiff::open(path)?, chunk_cells)
    }

    /// Load `load` cells beyond each window, so a window that moves less than this
    /// does not read the file; keep chunks up to `keep` cells beyond the loaded
    /// region, so a window moving back and forth at its edge does not reload them.
    pub fn with_margins(mut self, load: u32, keep: u32) -> Self {
        self.load_margin = load;
        self.keep_margin = keep;
        self
    }

    /// The file windows are read from.
    pub fn file(&self) -> &CostGeoTiff {
        &self.file
    }

    /// The raster's cells.
    pub fn cells(&self) -> CellRegion {
        CellRegion::from_size(self.file.size())
    }

    /// Chunk counters.
    pub fn stats(&self) -> ChunkStats {
        ChunkStats {
            loaded: self.loaded,
            evicted: self.evicted,
            held: self.chunks.loaded_chunks().count(),
        }
    }

    /// Drop distant chunks, then load the chunks under `wanted` plus the load margin.
    fn refill(&mut self, wanted: CellRegion) -> Result<(), VoxelError> {
        let region = wanted.grown(self.load_margin);
        let size = region.size();
        // Square around the middle of the region that holds it plus the keep margin.
        let middle = (region.min.as_i64vec2() + (size / 2).as_i64vec2()).as_ivec2();
        let radius = (size.max_element() / 2 + 1).saturating_add(self.keep_margin);
        // Cleared first: if the load fails, the evicted chunks must not still count as loaded.
        self.loaded_region = None;
        self.evicted += self.chunks.evict_outside(middle, radius) as u64;
        self.loaded += self
            .file
            .load_region(&mut self.chunks, region.min, region.max)? as u64;
        self.loaded_region = Some(region);
        Ok(())
    }
}

impl CostWindows for GeoTiffWindows {
    fn lattice(&self) -> Lattice {
        self.chunks.lattice()
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, VoxelError> {
        if !cells.overlaps(&self.cells()) {
            return Ok(Costmap::filled(self.lattice(), cells, COST_UNKNOWN));
        }
        if !self
            .loaded_region
            .is_some_and(|loaded| loaded.contains_region(&cells))
        {
            self.refill(cells)?;
        }
        Ok(self.chunks.window(cells))
    }
}

#[cfg(test)]
mod tests {
    use glam::{DVec2, IVec2, UVec2};

    use super::*;
    use crate::{GeoReference, write_cost_geotiff_region};

    const RESOLUTION: f64 = 0.5;
    const CHUNK_CELLS: u32 = 64;
    const MAP_CELLS: UVec2 = UVec2::new(1000, 200);
    const WINDOW: UVec2 = UVec2::new(20, 20);
    /// The node's margins: 5 m loaded and 25 m kept.
    const LOAD: u32 = 10;
    const KEEP: u32 = 50;

    /// Asymmetric in x and y, never `COST_UNKNOWN`.
    fn pattern(cell: IVec2) -> u8 {
        ((cell.x * 7 + cell.y * 13).rem_euclid(200)) as u8
    }

    /// Writes the pattern as a GeoTIFF and opens it.
    fn open_pattern_map(dir: &tempfile::TempDir) -> GeoTiffWindows {
        let mut grid = TiledGrid::new(RESOLUTION, CHUNK_CELLS, COST_UNKNOWN).unwrap();
        for y in 0..MAP_CELLS.y as i32 {
            for x in 0..MAP_CELLS.x as i32 {
                grid.set(IVec2::new(x, y), pattern(IVec2::new(x, y)));
            }
        }
        let georef = GeoReference {
            epsg: Some(32630),
            anchor: DVec2::new(500_000.0, 6_200_000.0),
        };
        let path = dir.path().join("map.tif");
        write_cost_geotiff_region(&path, &grid, &georef, IVec2::ZERO, MAP_CELLS.as_ivec2())
            .unwrap();
        GeoTiffWindows::open(path, CHUNK_CELLS)
            .unwrap()
            .with_margins(LOAD, KEEP)
    }

    /// The pattern at every window cell, or unknown off the raster.
    fn expected(window: &Costmap) -> Vec<u8> {
        let raster = CellRegion::from_size(MAP_CELLS);
        window
            .indexed_iter()
            .map(|(cell, _)| {
                if raster.contains(cell) {
                    pattern(cell)
                } else {
                    COST_UNKNOWN
                }
            })
            .collect()
    }

    fn at(x: i32, y: i32) -> CellRegion {
        CellRegion::centered(IVec2::new(x, y), WINDOW)
    }

    #[test]
    fn interior_window_holds_the_pattern() {
        let dir = tempfile::tempdir().unwrap();
        let mut map = open_pattern_map(&dir);
        let window = map.window(at(300, 100)).unwrap();
        assert_eq!(window.cells().min, IVec2::new(290, 90));
        assert_eq!(window.lattice(), Lattice::new(DVec2::ZERO, RESOLUTION));
        assert_eq!(window.world_bounds().min, DVec2::new(145.0, 45.0));
        assert_eq!(window.as_slice(), expected(&window).as_slice());
    }

    #[test]
    fn failed_load_does_not_leave_evicted_chunks_marked_loaded() {
        let dir = tempfile::tempdir().unwrap();
        let mut map = open_pattern_map(&dir);
        map.window(at(100, 100)).unwrap();
        std::fs::File::create(dir.path().join("map.tif")).unwrap();
        assert!(map.window(at(900, 100)).is_err());
        assert!(map.window(at(100, 100)).is_err());
    }

    #[test]
    fn window_over_the_edge_is_unknown_outside() {
        let dir = tempfile::tempdir().unwrap();
        let mut map = open_pattern_map(&dir);
        let window = map.window(at(2, 197)).unwrap();
        let want = expected(&window);
        assert!(want.contains(&COST_UNKNOWN));
        assert!(want.iter().any(|&c| c != COST_UNKNOWN));
        assert_eq!(window.as_slice(), want.as_slice());
    }

    #[test]
    fn held_chunks_stay_bounded_and_a_return_trip_gives_the_same_window() {
        let dir = tempfile::tempdir().unwrap();
        let mut map = open_pattern_map(&dir);
        let first = map.window(at(100, 100)).unwrap();
        let mut most_held = 0;
        for x in (100..900).step_by(10) {
            let window = map.window(at(x, 100)).unwrap();
            assert_eq!(window.as_slice(), expected(&window).as_slice());
            most_held = most_held.max(map.stats().held);
        }
        let total_chunks =
            (MAP_CELLS.x.div_ceil(CHUNK_CELLS) * MAP_CELLS.y.div_ceil(CHUNK_CELLS)) as usize;
        assert!(
            most_held * 3 < total_chunks,
            "held {most_held} of {total_chunks}"
        );
        assert!(map.stats().evicted > 0);
        assert_eq!(map.window(at(100, 100)).unwrap(), first);
    }

    #[test]
    fn a_one_cell_move_loads_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let mut map = open_pattern_map(&dir);
        map.window(at(300, 100)).unwrap();
        let before = map.stats();
        assert!(before.loaded > 0);
        map.window(at(301, 100)).unwrap();
        map.window(at(301, 101)).unwrap();
        assert_eq!(map.stats(), before);
    }

    #[test]
    fn a_window_off_the_raster_is_unknown_and_reads_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let mut map = open_pattern_map(&dir);
        for centre in [
            IVec2::new(-5000, 100),
            IVec2::splat(i32::MAX),
            IVec2::splat(i32::MIN),
            IVec2::new(i32::MAX, i32::MIN),
        ] {
            let window = map.window(CellRegion::centered(centre, WINDOW)).unwrap();
            assert_eq!(window.size(), WINDOW, "{centre}");
            assert!(
                window.as_slice().iter().all(|&c| c == COST_UNKNOWN),
                "{centre}"
            );
        }
        assert_eq!(map.stats().loaded, 0);
        let window = map.window(at(300, 100)).unwrap();
        assert_eq!(window.as_slice(), expected(&window).as_slice());
    }
}
