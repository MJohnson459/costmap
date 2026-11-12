//! Plans one route across a cost GeoTIFF and reports what it took, without ROS.
//!
//! ```text
//! cargo run --release -p costmap_ros --example plan_route -- \
//!     --map cost.tif --start 20,20 --goal 140,170
//! ```

use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context, bail};
use clap::Parser;
use costmap_ros::{RouteOptions, RoutePlanner};
use glam::DVec2;

#[derive(Debug, Parser)]
struct Args {
    /// Single-band u8 cost GeoTIFF.
    #[arg(long)]
    map: PathBuf,
    /// Start as `x,y` in metres from the raster's lower-left corner.
    #[arg(long)]
    start: String,
    /// Goal, as the start.
    #[arg(long)]
    goal: String,
    /// Times the route is planned; the first plan reads its tiles from the file.
    #[arg(long, default_value_t = 5)]
    repeats: u32,
}

fn point(text: &str) -> anyhow::Result<DVec2> {
    let Some((x, y)) = text.split_once(',') else {
        bail!("expected x,y but got {text:?}");
    };
    Ok(DVec2::new(x.trim().parse()?, y.trim().parse()?))
}

/// Peak resident memory of this process in megabytes, from `/proc`.
fn peak_rss_mb() -> Option<f64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|line| line.starts_with("VmHWM:"))?;
    let kb: f64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kb / 1024.0)
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt().init();
    let args = Args::parse();
    let (start, goal) = (point(&args.start)?, point(&args.goal)?);

    let opened = Instant::now();
    let planner = RoutePlanner::open(&args.map, RouteOptions::default())
        .with_context(|| format!("opening {}", args.map.display()))?;
    let graph = planner.graph();
    println!(
        "{} x {} cells at {} m: {} crossings in {:.1} MB, ready in {:.2} s",
        graph.layout().size().x,
        graph.layout().size().y,
        graph.resolution(),
        graph.node_count() / 2,
        graph.memory_bytes() as f64 / 1e6,
        opened.elapsed().as_secs_f64()
    );

    for i in 0..args.repeats.max(1) {
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
