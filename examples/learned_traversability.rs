//! # Local Costmap with Learned Traversability
//!
//! This example demonstrates a rolling-window local costmap where a mock "learned traversability"
//! model fills a semantic grid each frame. The model predicts terrain traversability (0.0 = free,
//! 1.0 = impassable), which is projected into the master costmap and then inflated for safe planning.
//!
//! The pipeline is: **semantic traversability layer** (mock learned model output) → **projection**
//! (conversion to cost space) → **inflation layer** → single master grid.
//!
//! ## Robotics Context
//! This pattern demonstrates how learned/perception-based traversability models can be integrated
//! into a costmap pipeline:
//! - **Semantic segmentation**: A neural network predicts traversability for each cell (e.g., 0.9
//!   for rocky terrain, 0.0 for smooth grass).
//! - **Projection**: Convert semantic values to costmap costs via a configurable closure.
//! - **Rolling window**: Keep the semantic grid centered on the robot as it moves.
//! - **Inflation**: Expand obstacles to account for robot size and safety margins.
//!
//! The robot in this example drives across a high-cost terrain band and the control decision is
//! printed when high-traversability costs are detected underneath the robot.

use std::error::Error;
use std::time::Duration;

use costmap::rerun_viz::{log_costmap, log_point3d};
use costmap::types::{COST_FREE, Pose2};
use costmap::{Grid2d, MapInfo, MergePolicy, ProjectionLayer, cost_from_unit};
use costmap::{InflationConfig, LayeredCostmap, WavefrontInflationLayer};
use glam::{UVec2, Vec2, Vec3};

// Simulation parameters
const DELAY_MS: u64 = 100;
const WAYPOINT_SPEED_MPS: f32 = 1.1;
const WAYPOINTS: &[(f32, f32)] = &[(2.0, 3.0), (7.0, 3.0), (7.0, 5.0), (2.0, 5.0)];

// Local costmap configuration
/// Size of the robot-centered rolling window costmap (square, in cells)
const LOCAL_SIZE_CELLS: u32 = 120;
const RESOLUTION: f32 = 0.1;

// Visualization Z-heights for layering elements in Rerun
const Z_LOCAL: f32 = 0.12;
const Z_ROBOT: f32 = 0.35;

fn main() -> Result<(), Box<dyn Error>> {
    let resolution = RESOLUTION;
    let local_info = MapInfo::square(LOCAL_SIZE_CELLS, resolution);
    let rec = rerun::RecordingStreamBuilder::new("costmap_learned_traversability").spawn()?;

    let mut layered = LayeredCostmap::new(local_info, COST_FREE, true);
    let id = layered.add_layer(Box::new(ProjectionLayer::from_grid(
        Grid2d::<f32>::new_with_value(local_info, 0.0),
        MergePolicy::Max,
        true, // rolling window
        |t| Some(cost_from_unit(*t)),
    )));
    layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
        inflation_radius_m: 0.4,
        inscribed_radius_m: 0.1,
        cost_scaling_factor: 3.0,
        ..Default::default()
    })));

    let waypoints: Vec<Vec2> = WAYPOINTS.iter().map(|(x, y)| Vec2::new(*x, *y)).collect();
    let (segment_lengths, total_length) = build_segments(&waypoints);
    let dt = DELAY_MS as f32 / 1000.0;

    let mut frame_idx: i64 = 0;
    loop {
        rec.set_time_sequence("frame", frame_idx);
        let distance = (frame_idx as f32) * WAYPOINT_SPEED_MPS * dt;
        let (robot_pos, heading_dir) =
            sample_path(&waypoints, &segment_lengths, total_length, distance);
        let heading = heading_dir.y.atan2(heading_dir.x);
        let robot = Pose2::new(robot_pos, heading);

        // Mock "learned traversability model": fill the source world-anchored. The
        // rolling-window ProjectionLayer re-centres the grid on the robot during
        // update_map, and update_origin preserves data by world position, so we just
        // write each cell's cost from its current world location.
        {
            let proj = layered.layer_mut::<ProjectionLayer<f32>>(id).unwrap();
            let src = proj.source_mut();
            let (w, h) = (src.width(), src.height());
            for y in 0..h {
                for x in 0..w {
                    let cell = UVec2::new(x, y);
                    let world = src.map_to_world(cell);
                    // Impassable terrain band in world coords (x in [4.0, 4.5]).
                    // Traversability 1.0 projects to COST_LETHAL, which seeds inflation.
                    let t = if world.x > 4.0 && world.x < 4.5 {
                        1.0
                    } else {
                        0.0
                    };
                    let _ = src.set(cell, t);
                }
            }
        }

        layered.update_map(robot);

        // Control decision: read traversability back under the robot.
        {
            let proj = layered.layer::<ProjectionLayer<f32>>(id).unwrap();
            let src = proj.source();
            if let Some(cell) = src.world_to_map(robot_pos) {
                let t = src.get(cell).copied().unwrap_or(0.0);
                if t > 0.5 {
                    println!(
                        "[frame {frame_idx}] high-cost terrain (t={t:.2}) under robot -> SLOW DOWN"
                    );
                }
            }
        }

        log_costmap(&rec, "world/local_costmap", layered.master(), Z_LOCAL)?;
        log_point3d(
            &rec,
            "world/robot",
            Vec3::new(robot_pos.x, robot_pos.y, Z_ROBOT),
            Some(rerun::Color::from_rgb(0, 200, 255)),
            Some(5.0),
        )?;

        if DELAY_MS > 0 {
            std::thread::sleep(Duration::from_millis(DELAY_MS));
        }
        frame_idx = frame_idx.wrapping_add(1);
    }
}

// Helper functions for path following - not core library APIs

fn build_segments(waypoints: &[Vec2]) -> (Vec<f32>, f32) {
    let mut lengths = Vec::with_capacity(waypoints.len());
    let mut total = 0.0;

    if waypoints.len() < 2 {
        return (lengths, total);
    }

    for idx in 0..waypoints.len() {
        let a = waypoints[idx];
        let b = waypoints[(idx + 1) % waypoints.len()];
        let len = (b - a).length();
        lengths.push(len);
        total += len;
    }

    (lengths, total)
}

fn sample_path(
    waypoints: &[Vec2],
    segment_lengths: &[f32],
    total_length: f32,
    distance: f32,
) -> (Vec2, Vec2) {
    if waypoints.is_empty() {
        return (Vec2::ZERO, Vec2::X);
    }

    if waypoints.len() == 1 || total_length <= 0.0 {
        return (waypoints[0], Vec2::X);
    }

    let mut remaining = distance.rem_euclid(total_length);

    for (idx, seg_len) in segment_lengths.iter().enumerate() {
        if *seg_len <= 0.0 {
            continue;
        }
        if remaining <= *seg_len {
            let a = waypoints[idx];
            let b = waypoints[(idx + 1) % waypoints.len()];
            let dir = (b - a).normalize_or_zero();
            let pos = a + dir * remaining;
            let heading = if dir.length_squared() == 0.0 {
                Vec2::X
            } else {
                dir
            };
            return (pos, heading);
        }
        remaining -= *seg_len;
    }

    (waypoints[0], Vec2::X)
}
