//! Plans one route across a whole map and reports what it took.
//!
//! ```text
//! cargo run --release --features geotiff --example plan_route -- \
//!     cost.tif 20,20 140,170 [repeats]
//! ```
//!
//! The map is a cost GeoTIFF or a ROS map YAML. The start and goal are `x,y` in
//! metres in the map's frame: for a GeoTIFF, from the raster's lower-left corner.
//! The route is planned `repeats` times, 5 by default; the first plan reads its
//! tiles from the file.

use std::error::Error;
use std::time::Instant;

use costmap::{GraphOrigin, RouteOptions, RoutePlanner};
use glam::DVec2;

const USAGE: &str = "usage: plan_route MAP START_X,START_Y GOAL_X,GOAL_Y [REPEATS]";

fn point(text: &str) -> Result<DVec2, Box<dyn Error>> {
    let (x, y) = text
        .split_once(',')
        .ok_or_else(|| format!("expected x,y but got {text:?}"))?;
    Ok(DVec2::new(x.trim().parse()?, y.trim().parse()?))
}

/// Peak resident memory of this process in megabytes, from `/proc`.
fn peak_rss_mb() -> Option<f64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|line| line.starts_with("VmHWM:"))?;
    let kb: f64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kb / 1024.0)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [map, start, goal, rest @ ..] = args.as_slice() else {
        return Err(USAGE.into());
    };
    let (start, goal) = (point(start)?, point(goal)?);
    let repeats: u32 = match rest {
        [] => 5,
        [repeats] => repeats.parse()?,
        _ => return Err(USAGE.into()),
    };

    let opened = Instant::now();
    let planner = RoutePlanner::open(map, RouteOptions::default())
        .map_err(|e| format!("opening {map}: {e}"))?;
    let graph = planner.graph();
    println!(
        "{} x {} cells at {} m: {} crossings in {:.1} MB, ready in {:.2} s",
        graph.layout().cells().size().x,
        graph.layout().cells().size().y,
        graph.lattice().resolution,
        graph.node_count() / 2,
        graph.memory_bytes() as f64 / 1e6,
        opened.elapsed().as_secs_f64()
    );
    match planner.origin() {
        GraphOrigin::Read(path) => println!("graph read from {}", path.display()),
        GraphOrigin::Stored { took, path } => println!(
            "graph built in {:.1} s and stored at {}",
            took.as_secs_f64(),
            path.display()
        ),
        GraphOrigin::NotStored { took, path, error } => println!(
            "graph built in {:.1} s; storing it at {} failed: {error}",
            took.as_secs_f64(),
            path.display()
        ),
        GraphOrigin::Built { took } => println!("graph built in {:.1} s", took.as_secs_f64()),
    }

    for i in 0..repeats.max(1) {
        let planned = Instant::now();
        let route = planner.plan(start, goal)?;
        println!(
            "plan {}: {:.1} m, {} points, {:.1} ms",
            i + 1,
            route.length_m,
            route.points.len(),
            planned.elapsed().as_secs_f64() * 1e3
        );
    }
    if let Some(mb) = peak_rss_mb() {
        println!("peak resident memory {mb:.0} MB");
    }
    Ok(())
}
