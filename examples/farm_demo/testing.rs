//! Maps written to temporary GeoTIFFs for the tests.

use std::{
    fs,
    path::{Path, PathBuf},
    sync::atomic::{AtomicU32, Ordering},
};

use costmap::{
    DenseGrid, GeoReference, MapInfo, TiledGrid, types::COST_UNKNOWN, write_cost_geotiff_region,
};
use glam::{DVec2, IVec2, UVec2};

const GEOREFERENCE: GeoReference = GeoReference {
    epsg: Some(32630),
    anchor: DVec2::new(500_000.0, 6_200_000.0),
};
const WRITE_CHUNK_CELLS: u32 = 256;

/// A directory under the system temp directory, removed on drop.
pub struct TempDir(PathBuf);

impl TempDir {
    pub fn new(name: &str) -> Self {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let dir = std::env::temp_dir().join(format!(
            "farm_demo_{name}_{}_{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&dir).unwrap();
        Self(dir)
    }

    pub fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

/// Write a `size_m` map at `resolution` whose cell centred at `p` has cost `cost(p)`,
/// with this crate's max-pooled overviews, and return its path.
pub fn write_map(
    dir: &TempDir,
    name: &str,
    size_m: DVec2,
    resolution: f64,
    cost: impl Fn(DVec2) -> u8,
) -> PathBuf {
    let size = (size_m / resolution).round().as_uvec2();
    let info = MapInfo {
        width: size.x,
        height: size.y,
        resolution,
        origin: DVec2::ZERO,
    };
    let raster = DenseGrid::from_fn(info, |x, y| {
        cost(info.origin + (UVec2::new(x, y).as_dvec2() + 0.5) * resolution)
    });
    let mut tiles = TiledGrid::new(resolution, WRITE_CHUNK_CELLS, COST_UNKNOWN).unwrap();
    tiles.write_window(&raster).unwrap();
    let path = dir.path().join(name);
    write_cost_geotiff_region(&path, &tiles, &GEOREFERENCE, IVec2::ZERO, size.as_ivec2()).unwrap();
    path
}
