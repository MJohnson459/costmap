//! Converts a ROS map YAML to a cost GeoTIFF for `costmap_window_server --map`.
//!
//! Usage: `yaml_to_cost_geotiff <map.yaml> <out.tif>`
//!
//! Cells take the cost Nav2 gives their occupancy. The raster's lower-left
//! corner is the YAML origin, as projected coordinates with no EPSG code.

use std::path::PathBuf;

use anyhow::{Context, anyhow};
use clap::Parser;
use costmap::types::COST_UNKNOWN;
use costmap::{GeoReference, RosMapLoader, TiledGrid, write_cost_geotiff_region};
use costmap_ros::costmap_from_occupancy;
use glam::IVec2;

const CHUNK_CELLS: u32 = 256;

#[derive(Debug, Parser)]
struct Args {
    /// ROS map YAML.
    yaml: PathBuf,
    /// GeoTIFF to write.
    tif: PathBuf,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let occupancy = RosMapLoader::load_from_yaml(&args.yaml)
        .with_context(|| format!("loading {}", args.yaml.display()))?;
    let map = costmap_from_occupancy(&occupancy).context("converting map to costs")?;
    let info = map.info();

    let mut grid = TiledGrid::new(info.resolution, CHUNK_CELLS, COST_UNKNOWN)?;
    for (cell, &cost) in map.indexed_iter() {
        grid.set(cell.as_ivec2(), cost);
    }
    let georef = GeoReference {
        epsg: None,
        anchor: info.origin,
    };
    let size = IVec2::new(
        i32::try_from(info.width).map_err(|e| anyhow!("map width: {e}"))?,
        i32::try_from(info.height).map_err(|e| anyhow!("map height: {e}"))?,
    );
    write_cost_geotiff_region(&args.tif, &grid, &georef, IVec2::ZERO, size)
        .with_context(|| format!("writing {}", args.tif.display()))?;
    Ok(())
}
