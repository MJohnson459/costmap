//! # Farm-scale navigation
//!
//! A robot crosses a map far larger than its costmap. The map is a cost GeoTIFF whose
//! full-resolution raster is held in memory only as chunks around the robot; the whole
//! map is held only as a coarse overview. For a map of cells coarser than 0.25 m the
//! overview factor is 1 and the overview is the full map; the summary states the
//! overview's size.
//!
//! 1. The global route is planned on a coarse overview of the map at about 0.5 m per
//!    cell: one of the file's stored overview images if the file records it as
//!    max-pooled, otherwise a max-pooled overview built by streaming the raster one strip
//!    or tile at a time. Stored levels built by averaging or subsampling can drop thin
//!    obstacles such as fences, so they are not used.
//! 2. While driving, tile chunks within a margin of the robot are loaded from the file
//!    and chunks beyond a larger radius are evicted.
//! 3. A 30 m rolling local costmap stacks the paged aerial prior (`TiledMapLayer`),
//!    simulated lidar with decay (`ObstacleLayer`) and inflation
//!    (`WavefrontInflationLayer`).
//! 4. The robot follows local A* plans to the furthest free route point in the window.
//!    Two obstacles that the aerial map does not show stand on the route; the lidar
//!    finds them and the local plan goes round them. If the furthest route point is
//!    out of reach, the robot plans to the furthest one it can reach; if there is none,
//!    it marks the obstacles its lidar holds on the overview and replans the route. It
//!    stops, blocked, when that route fails too or nothing new has been observed.
//! 5. Safety is measured against ground truth, not the local costmap: the robot's
//!    0.4 m disc, checked every half cell along each step's path, against the lethal
//!    cells of the full-resolution map and the surprise obstacles.
//!
//! Without `--map`, a synthetic 400 m x 300 m farm at 0.1 m is generated, written to
//! a temporary GeoTIFF and read back through the same path as a real map.
//!
//! ```text
//! cargo run --release --example farm_demo --features rerun,geotiff
//! cargo run --release --example farm_demo --features rerun,geotiff -- --map cost.tif
//! cargo run --release --example farm_demo --features rerun,geotiff -- --headless --frames frames
//! ffmpeg -framerate 30 -i frames/frame_%05d.png -pix_fmt yuv420p farm_demo.mp4
//! ```
//!
//! `--help` lists every option. The tests run with
//! `cargo test --release --example farm_demo --features rerun,geotiff`.

mod cli;
mod frames;
mod render;
mod route;
mod safety;
mod sim;
mod synthetic;
#[cfg(test)]
mod testing;
mod viz;
mod world;

#[cfg(test)]
#[path = "../../tests/common/gdal.rs"]
mod gdal;

use std::{
    error::Error,
    fmt,
    process::ExitCode,
    time::{Duration, Instant},
};

use costmap::CostGeoTiff;
use glam::UVec2;

use crate::{
    cli::{Command, Options, USAGE},
    frames::{FrameWriter, VIDEO_FPS},
    route::Overview,
    safety::{CLEARANCE_HORIZON_M, SafetyMonitor},
    sim::{DETOUR_EXTRA_LENGTH_M, STEP_S, Simulation, Stats, Status},
    viz::RerunLog,
};

/// Radius of the robot's circular footprint: the inscribed radius of both inflations
/// and the footprint of the ground-truth safety check.
pub const ROBOT_RADIUS_M: f32 = 0.4;
const MIB: f64 = 1024.0 * 1024.0;

fn main() -> ExitCode {
    let options = match cli::parse(std::env::args().skip(1)) {
        Ok(Command::Run(options)) => options,
        Ok(Command::Help) => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(message) => {
            eprint!("error: {message}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    match run(&options) {
        Ok(summary) => {
            print!("{summary}");
            if summary.succeeded() {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Run the whole demo and return its summary.
fn run(options: &Options) -> Result<Summary, Box<dyn Error>> {
    // Kept alive until the run ends: dropping it deletes the generated file.
    let synthetic;
    let path = match &options.map {
        Some(path) => path.clone(),
        None => {
            synthetic = synthetic::write_temp()
                .map_err(|e| format!("cannot write the synthetic farm: {e}"))?;
            println!(
                "Generated a synthetic farm at {}",
                synthetic.path().display()
            );
            synthetic.path().to_path_buf()
        }
    };
    let mut map =
        CostGeoTiff::open(&path).map_err(|e| format!("cannot open {}: {e}", path.display()))?;
    let epsg = map
        .georeference()
        .epsg
        .map_or_else(|| "unknown".to_string(), |code| code.to_string());
    println!(
        "Map {}: {} x {} cells at {} m, EPSG {epsg}",
        path.display(),
        map.size().x,
        map.size().y,
        map.resolution(),
    );

    let overview = Overview::load(&mut map, options.overview_m)
        .map_err(|e| format!("cannot read an overview of {}: {e}", path.display()))?;
    let info = overview.costs.info();
    println!(
        "Overview: {} x {} cells at {} m, {}",
        info.width, info.height, info.resolution, overview.source
    );
    let (start, goal) = match options.endpoints {
        Some(endpoints) => endpoints,
        None => overview.default_endpoints()?,
    };
    let route = overview.plan_route(start, goal)?;
    println!(
        "Route: {:.1} m from ({:.1}, {:.1}) to ({:.1}, {:.1})",
        route.length_m(),
        start.x,
        start.y,
        goal.x,
        goal.y
    );
    let surprises = world::place_surprises(&route, &overview.costs);
    for obstacle in &surprises {
        println!(
            "Surprise obstacle: {:.1} m x {:.1} m at ({:.1}, {:.1})",
            2.0 * obstacle.half_size.x,
            2.0 * obstacle.half_size.y,
            obstacle.centre.x,
            obstacle.centre.y
        );
    }

    let route_length_m = route.length_m();
    let mut sim = Simulation::new(map, overview, route, surprises, options.speed_mps)?;
    let chunk_size = sim.tiles().chunk_size();
    let data_chunks = sim.overview().chunks_under_known_cells(
        chunk_size,
        sim.tiles().resolution(),
        sim.map_size(),
    );
    let mut log = RerunLog::open(&options.viewer)?;
    if let Some(log) = &log {
        log.log_static(&sim)?;
    }
    let mut frames = options
        .frames
        .as_deref()
        .map(|dir| FrameWriter::new(dir, route_length_m, options.speed_mps))
        .transpose()?;

    let started = Instant::now();
    let mut stepping = Duration::ZERO;
    while sim.status() == Status::Driving {
        let step_started = Instant::now();
        sim.step()?;
        stepping += step_started.elapsed();
        if let Some(log) = &mut log {
            log.log_step(&sim)?;
        }
        if let Some(frames) = &mut frames {
            frames.after_step(&sim)?;
        }
    }
    if let Some(frames) = &frames {
        println!(
            "Wrote {} frames; make a video with:\n  ffmpeg -framerate {VIDEO_FPS} -i {}/frame_%05d.png -pix_fmt yuv420p farm_demo.mp4",
            frames.written(),
            frames.dir().display()
        );
    }
    Ok(Summary {
        route_length_m,
        status: sim.status(),
        stats: sim.stats(),
        safety: sim.safety(),
        steps: sim.steps(),
        map_size: sim.map_size(),
        overview_size: sim.overview().size(),
        chunk_size,
        extent_chunks: sim.extent_chunks(),
        data_chunks,
        wall: started.elapsed(),
        stepping,
    })
}

/// What the run achieved and what it cost.
struct Summary {
    route_length_m: f32,
    status: Status,
    stats: Stats,
    safety: SafetyMonitor,
    steps: u64,
    map_size: UVec2,
    overview_size: UVec2,
    chunk_size: u32,
    /// Chunks in the raster's extent.
    extent_chunks: usize,
    /// Chunks under known overview cells: see [`Overview::chunks_under_known_cells`].
    data_chunks: usize,
    /// Wall-clock time of the drive, including logging and frame writing.
    wall: Duration,
    /// Wall-clock time spent inside simulation steps.
    stepping: Duration,
}

impl Summary {
    fn succeeded(&self) -> bool {
        self.status == Status::Reached && self.safety.violations() == 0
    }
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = &self.stats;
        let cells = u64::from(self.map_size.x) * u64::from(self.map_size.y);
        let chunk_mib = f64::from(self.chunk_size).powi(2) / MIB;
        let per_step_ms = self.stepping.as_secs_f64() * 1e3 / self.steps.max(1) as f64;
        writeln!(
            f,
            "Route length            {:.1} m as first planned",
            self.route_length_m
        )?;
        writeln!(f, "Distance driven         {:.1} m", s.driven_m)?;
        writeln!(
            f,
            "Simulated time          {:.1} s ({} steps of {STEP_S} s)",
            self.steps as f32 * STEP_S,
            self.steps
        )?;
        writeln!(
            f,
            "Drive wall-clock time   {:.2} s ({per_step_ms:.2} ms per simulation step)",
            self.wall.as_secs_f64()
        )?;
        writeln!(
            f,
            "Map                     {} x {} cells, {:.1} MiB if dense",
            self.map_size.x,
            self.map_size.y,
            cells as f64 / MIB
        )?;
        writeln!(
            f,
            "Overview                {} x {} cells, {:.1} MiB held for the raw and inflated copies",
            self.overview_size.x,
            self.overview_size.y,
            2.0 * (u64::from(self.overview_size.x) * u64::from(self.overview_size.y)) as f64 / MIB
        )?;
        writeln!(
            f,
            "Tile chunks             {0} x {0} cells; {1} in the raster extent, {2} of them under known overview cells",
            self.chunk_size, self.extent_chunks, self.data_chunks
        )?;
        writeln!(
            f,
            "Peak chunks loaded      {} ({:.2} MiB), counted after loading and before evicting",
            s.peak_chunks,
            s.peak_chunks as f64 * chunk_mib
        )?;
        writeln!(
            f,
            "Tile loads / evictions  {} / {}",
            s.tile_loads, s.tile_evictions
        )?;
        writeln!(f, "Local plans             {}", s.replans)?;
        writeln!(
            f,
            "Route replans           {} on the overview around observed obstacles",
            s.global_replans
        )?;
        writeln!(
            f,
            "Detours                 {} plans over {DETOUR_EXTRA_LENGTH_M} m longer than the route, {} with a surprise obstacle in lidar range",
            s.detours, s.detours_near_surprise
        )?;
        writeln!(
            f,
            "Safety violations       {} steps on which the {ROBOT_RADIUS_M} m footprint overlapped a lethal map cell or surprise obstacle",
            self.safety.violations()
        )?;
        writeln!(
            f,
            "Minimum clearance       {:.2} m from the footprint to a lethal map cell or surprise obstacle (searched to {CLEARANCE_HORIZON_M} m)",
            self.safety.min_clearance_m()
        )?;
        let outcome = match self.status {
            Status::Reached => "reached".to_string(),
            Status::Blocked { at, reason } => {
                format!(
                    "not reached: blocked at ({:.1}, {:.1}): {reason}",
                    at.x, at.y
                )
            }
            Status::TimedOut => "not reached: timed out".to_string(),
            Status::Driving => "not reached: still driving".to_string(),
        };
        writeln!(f, "Goal                    {outcome}")
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use glam::Vec2;

    use super::*;
    use crate::{
        cli::Viewer,
        gdal::{gdal_available, run_gdal},
        route::{OverviewSource, StreamReason},
        testing::{self, TempDir},
    };

    const FENCE_RESOLUTION_M: f32 = 0.1;
    const FENCE_X_M: f32 = 200.0;
    const GATE_Y_M: (f32, f32) = (140.0, 145.0);

    /// Free ground with a one-cell lethal fence at x = 200 m and a 5 m gate in it.
    fn fence(p: Vec2) -> u8 {
        let fence_column = (FENCE_X_M / FENCE_RESOLUTION_M) as i32;
        let in_gate = (GATE_Y_M.0..GATE_Y_M.1).contains(&p.y);
        if (p.x / FENCE_RESOLUTION_M).floor() as i32 == fence_column && !in_gate {
            costmap::types::COST_LETHAL
        } else {
            costmap::types::COST_FREE
        }
    }

    #[test]
    fn averaged_stored_overviews_are_streamed_and_the_route_uses_the_gate() {
        if !gdal_available("averaged_stored_overviews_are_streamed_and_the_route_uses_the_gate") {
            return;
        }
        let dir = TempDir::new("fence");
        let size = Vec2::new(400.0, 200.0);
        let ours = testing::write_map(&dir, "fence.tif", size, FENCE_RESOLUTION_M, fence);
        let averaged = dir.path().join("averaged.tif");
        run_gdal("gdal_translate", &[&ours, &averaged]);
        let gdaladdo = ["-q", "-r", "average"].map(Path::new);
        let factors = ["2", "4", "8"].map(Path::new);
        let args: Vec<&Path> = gdaladdo
            .into_iter()
            .chain([averaged.as_path()])
            .chain(factors)
            .collect();
        run_gdal("gdaladdo", &args);

        let mut map = CostGeoTiff::open(&averaged).unwrap();
        let overview = Overview::load(&mut map, route::DEFAULT_OVERVIEW_RESOLUTION_M).unwrap();
        let reason = StreamReason::NotMaxPooled {
            resampling: Some("AVERAGE".into()),
        };
        assert_eq!(
            overview.source,
            OverviewSource::Streamed { factor: 4, reason }
        );
        let (start, goal) = (Vec2::new(180.0, 60.0), Vec2::new(220.0, 60.0));
        let route = overview.plan_route(start, goal).unwrap();
        let crossing = route
            .points()
            .windows(2)
            .find(|pair| pair[0].x < FENCE_X_M && pair[1].x >= FENCE_X_M)
            .unwrap();
        assert!(
            (GATE_Y_M.0..GATE_Y_M.1).contains(&crossing[0].y),
            "route crosses the fence at {crossing:?}"
        );

        let options = Options {
            map: Some(averaged),
            endpoints: Some((start, goal)),
            viewer: Viewer::Headless,
            ..Options::default()
        };
        let summary = run(&options).unwrap();
        print!("{summary}");
        assert_eq!(summary.status, Status::Reached);
        assert_eq!(summary.safety.violations(), 0);
    }

    #[test]
    fn synthetic_farm_reaches_goal_around_surprises_with_bounded_paging() {
        let options = Options {
            viewer: Viewer::Headless,
            ..Options::default()
        };
        let summary = run(&options).unwrap();
        print!("{summary}");
        assert_eq!(summary.status, Status::Reached);
        assert_eq!(summary.safety.violations(), 0);
        assert!(summary.safety.min_clearance_m() > 0.0);
        assert!(summary.stats.detours_near_surprise >= 1);
        assert!(summary.data_chunks <= summary.extent_chunks);
        assert!(
            summary.stats.peak_chunks * 10 < summary.data_chunks,
            "peak {} of {} chunks with data",
            summary.stats.peak_chunks,
            summary.data_chunks
        );
    }
}
