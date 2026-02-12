//! EDT profiling harness
//!
//! Runs the Euclidean Distance Transform many times on a large grid for `perf` or other
//! profilers. Criterion's statistical sampling adds noise; this binary does a simple
//! loop so the profiler sees a clean signal.
//!
//! # Usage
//!
//! ```bash
//! cargo build --release --example profile_edt
//! perf record -g ./target/release/examples/profile_edt
//! perf report
//! ```

use std::hint::black_box;
use std::time::Instant;

use costmap::layers::inflation::DistanceField;

const WIDTH: usize = 4000;
const HEIGHT: usize = 4000;
const ITERS: u32 = 100;
const INSCRIBED_RADIUS_CELLS: f32 = 2.0;
const INFLATION_RADIUS_CELLS: f32 = 10.0;

fn main() {
    let occupied = make_sparse_occupancy(WIDTH, HEIGHT);
    let n = WIDTH * HEIGHT;
    let cells_m = n as f64 / 1_000_000.0;
    eprintln!(
        "profile_edt: {}×{} = {:.1}M cells, {} iterations",
        WIDTH, HEIGHT, cells_m, ITERS
    );

    let start = Instant::now();
    for _ in 0..ITERS {
        let df = DistanceField::from_occupancy(WIDTH, HEIGHT, &occupied);
        let costmap = df.to_costmap(INSCRIBED_RADIUS_CELLS, INFLATION_RADIUS_CELLS);
        black_box(costmap);
    }
    let elapsed = start.elapsed();

    let per_run = elapsed.as_secs_f64() / ITERS as f64;
    let per_run_ms = per_run * 1000.0;
    eprintln!(
        "Done: {:.2}s total, {:.2}ms per run ({:.1} runs/s)",
        elapsed.as_secs_f64(),
        per_run_ms,
        1.0 / per_run
    );
}

fn make_sparse_occupancy(width: usize, height: usize) -> Vec<bool> {
    let step = (width.max(height) / 8).max(1);
    let mut occupied = vec![false; width * height];
    for y in (0..height).step_by(step) {
        for x in (0..width).step_by(step) {
            occupied[y * width + x] = true;
        }
    }
    occupied
}
