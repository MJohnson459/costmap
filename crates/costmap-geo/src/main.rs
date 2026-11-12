use std::{path::PathBuf, process::ExitCode, time::Instant};

use clap::Parser;
use costmap_geo::{CostRules, Job, run};

/// Build a navigation cost GeoTIFF from OpenDroneMap DSM, DTM and orthophoto rasters.
///
/// All inputs must be in the same projected CRS. Cost values: 0 free, 1-252 graded,
/// 253 inscribed, 254 lethal, 255 unknown. The summary counts 253 and 254 together.
#[derive(Debug, Parser)]
#[command(version)]
struct Args {
    /// Digital surface model (odm_dem/dsm.tif).
    #[arg(long)]
    dsm: PathBuf,
    /// Digital terrain model (odm_dem/dtm.tif); defines the output extent.
    #[arg(long)]
    dtm: PathBuf,
    /// Output cost GeoTIFF.
    #[arg(long)]
    out: PathBuf,
    /// RGB or RGBA orthophoto (odm_orthophoto/odm_orthophoto.tif); alpha 0 marks unknown cells.
    #[arg(long)]
    ortho: Option<PathBuf>,
    /// GeoJSON polygons with an integer `cost` property, in the rasters' CRS.
    #[arg(long)]
    overrides: Option<PathBuf>,
    /// Output cell size in metres. Defaults to the DTM's pixel size.
    #[arg(long)]
    resolution: Option<f64>,
    /// Height above ground in metres from which a cell is lethal.
    #[arg(long, default_value_t = CostRules::default().obstacle_height)]
    obstacle_height: f32,
    /// Largest edge in metres of the box the DTM is averaged over for slope; the box is an odd
    /// number of cells, at least 3.
    #[arg(long, default_value_t = CostRules::default().slope_window_m)]
    slope_window_m: f64,
    /// Slope in degrees up to which a cell costs nothing.
    #[arg(long, default_value_t = CostRules::default().slope_free_deg)]
    slope_free_deg: f32,
    /// Slope in degrees from which a cell is lethal.
    #[arg(long, default_value_t = CostRules::default().slope_lethal_deg)]
    slope_lethal_deg: f32,
    /// Minimum cost (0-254) of vegetated cells; 254 is lethal, 0 turns the rule off. Needs --ortho.
    #[arg(long, default_value_t = CostRules::default().vegetation_cost)]
    vegetation_cost: u8,
    /// Excess-green index above which an orthophoto pixel counts as green.
    #[arg(long, default_value_t = CostRules::default().exg_threshold)]
    exg_threshold: f32,
    /// Edge in metres of the box around a cell in which green pixels are counted; the box is an
    /// odd number of cells, at least 1.
    #[arg(long, default_value_t = CostRules::default().vegetation_window_m)]
    vegetation_window_m: f64,
    /// Fraction (above 0, at most 1) of the known pixels in the box that must be green for the
    /// cell to count as vegetation.
    #[arg(long, default_value_t = CostRules::default().vegetation_min_fraction)]
    vegetation_min_fraction: f64,
}

impl Args {
    fn into_job(self) -> Job {
        Job {
            dsm: self.dsm,
            dtm: self.dtm,
            ortho: self.ortho,
            overrides: self.overrides,
            out: self.out,
            rules: CostRules {
                resolution: self.resolution,
                obstacle_height: self.obstacle_height,
                slope_window_m: self.slope_window_m,
                slope_free_deg: self.slope_free_deg,
                slope_lethal_deg: self.slope_lethal_deg,
                vegetation_cost: self.vegetation_cost,
                exg_threshold: self.exg_threshold,
                vegetation_window_m: self.vegetation_window_m,
                vegetation_min_fraction: self.vegetation_min_fraction,
            },
        }
    }
}

fn main() -> ExitCode {
    let job = Args::parse().into_job();
    let start = Instant::now();
    match run(&job) {
        Ok(summary) => {
            println!("{summary}");
            println!(
                "wrote {} in {:.1} s",
                job.out.display(),
                start.elapsed().as_secs_f64()
            );
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}
