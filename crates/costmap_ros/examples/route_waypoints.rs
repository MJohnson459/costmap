//! Plans a route across a whole cost GeoTIFF and prints waypoints along it.
//!
//! Usage: `route_waypoints --map cost.tif --start X,Y --goal X,Y`
//!
//! Nav2 holds a window of the map, so it cannot plan to a goal outside it.
//! This plans on a max-pooled overview of the whole raster with `costmap`'s
//! A* and prints one waypoint every `--spacing-m` along the route, as
//! `x,y x,y ...` in the map frame, for Nav2 to drive leg by leg.

use anyhow::{Context, anyhow};
use clap::Parser;
use costmap::types::COST_UNKNOWN;
use costmap::{CostGeoTiff, Costmap, PlanOptions, plan};
use glam::{DVec2, IVec2};
use std::path::PathBuf;

const CHUNK_CELLS: u32 = 256;
const COST_SCALING: f32 = 2.0;

#[derive(Debug, Parser)]
struct Args {
    /// Cost GeoTIFF; the map frame is at its lower-left corner.
    #[arg(long)]
    map: PathBuf,
    /// Route start as `x,y` in map metres.
    #[arg(long, value_parser = parse_point)]
    start: DVec2,
    /// Route goal as `x,y` in map metres.
    #[arg(long, value_parser = parse_point)]
    goal: DVec2,
    /// Distance along the route between waypoints, in metres.
    #[arg(long, default_value_t = 15.0)]
    spacing_m: f64,
    /// Largest overview cell the route is planned on, in metres.
    #[arg(long, default_value_t = 0.25)]
    overview_m: f64,
    /// Robot radius in metres.
    #[arg(long, default_value_t = 0.3)]
    robot_radius_m: f32,
    /// Distance from obstacles over which the route is pushed away, in metres.
    #[arg(long, default_value_t = 1.5)]
    inflation_m: f32,
}

fn parse_point(text: &str) -> Result<DVec2, String> {
    let (x, y) = text
        .split_once(',')
        .ok_or_else(|| format!("expected x,y, got {text}"))?;
    let parse = |v: &str| v.trim().parse::<f64>().map_err(|e| format!("{v}: {e}"));
    Ok(DVec2::new(parse(x)?, parse(y)?))
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let mut map =
        CostGeoTiff::open(&args.map).with_context(|| format!("opening {}", args.map.display()))?;

    let mut factor = 1;
    while map.resolution() * f64::from(2 * factor) <= args.overview_m {
        factor *= 2;
    }
    let tiles = map.read_overview_max(factor, CHUNK_CELLS)?;
    let size = map.size().map(|s| s.div_ceil(factor));
    let window = tiles.window(IVec2::ZERO, size);
    let mut overview =
        Costmap::from_vec(*window.info(), window.as_slice().to_vec())?.with_fill(COST_UNKNOWN);
    overview.inflate(args.inflation_m, args.robot_radius_m, COST_SCALING);

    let cell = |point: DVec2, name: &str| {
        overview
            .world_to_map(point)
            .ok_or_else(|| anyhow!("{name} {point} is outside the map"))
    };
    let path = plan(
        &overview,
        cell(args.start, "start")?,
        cell(args.goal, "goal")?,
        &PlanOptions::default(),
    )
    .ok_or_else(|| anyhow!("no route from {} to {}", args.start, args.goal))?;
    let route = path.to_world(&overview);

    let mut waypoints = Vec::new();
    let mut since_last = 0.0;
    for pair in route.windows(2) {
        since_last += pair[0].distance(pair[1]);
        if since_last >= args.spacing_m {
            waypoints.push(pair[1]);
            since_last = 0.0;
        }
    }
    // A last waypoint close to the goal is replaced by the goal itself.
    if since_last < args.spacing_m / 3.0 {
        waypoints.pop();
    }
    waypoints.push(args.goal);

    eprintln!(
        "route {:.1} m on a {:.2} m overview, {} waypoints",
        path.length_m(overview.info().resolution),
        overview.info().resolution,
        waypoints.len()
    );
    let text: Vec<String> = waypoints
        .iter()
        .map(|p| format!("{:.2},{:.2}", p.x, p.y))
        .collect();
    println!("{}", text.join(" "));
    Ok(())
}
