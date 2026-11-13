use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;
use costmap::types::COST_UNKNOWN;
use costmap::{
    GeoReference, RosMapLoader, TiledGrid, costmap_from_occupancy, write_cost_geotiff_region,
};
use glam::IVec2;

const CHUNK_CELLS: u32 = 256;

/// Convert a ROS map (YAML and image) to a cost GeoTIFF.
///
/// A ROS map image cannot be read in part; a GeoTIFF can, so a large map
/// converted with this tool can be served chunk by chunk. Each cell takes the
/// cost Nav2's StaticLayer gives its occupancy: occupied 254, free 0, unknown
/// 255. The raster's lower-left corner is the YAML origin, written as
/// projected coordinates with no EPSG code.
#[derive(Debug, Parser)]
#[command(version)]
struct Args {
    /// ROS map YAML.
    yaml: PathBuf,
    /// Cost GeoTIFF to write.
    out: PathBuf,
}

fn convert(args: &Args) -> Result<(), Box<dyn Error>> {
    let occupancy = RosMapLoader::load_from_yaml(&args.yaml)?;
    let map = costmap_from_occupancy(&occupancy)?;
    let info = map.info();

    let mut grid = TiledGrid::new(info.resolution, CHUNK_CELLS, COST_UNKNOWN)?;
    for (cell, &cost) in map.indexed_iter() {
        grid.set(cell.as_ivec2(), cost);
    }
    let georef = GeoReference {
        epsg: None,
        anchor: info.origin,
    };
    let size = IVec2::new(i32::try_from(info.width)?, i32::try_from(info.height)?);
    write_cost_geotiff_region(&args.out, &grid, &georef, IVec2::ZERO, size)?;
    println!(
        "{}: {} x {} cells at {} m",
        args.out.display(),
        info.width,
        info.height,
        info.resolution
    );
    Ok(())
}

fn main() -> ExitCode {
    let args = Args::parse();
    match convert(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("error: {}: {err}", args.yaml.display());
            ExitCode::FAILURE
        }
    }
}
