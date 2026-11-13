//! # Local Costmap with Simulated Lidar
//!
//! This example demonstrates a complete local costmap workflow using the layered costmap API:
//! - A robot moving along a predefined path
//! - Simulated 2D lidar scanning the environment (sensor layer)
//! - Local costmap generation from lidar data (rolling window following the robot)
//! - Costmap inflation for safe navigation planning (inflation layer)
//!
//! The pipeline is: **obstacle layer** (fed with simulated laser scans) → **inflation layer** → single
//! master grid. Only the final master costmap (obstacles + inflation) is visualized.
//!
//! ## Robotics Context
//! This workflow is fundamental to mobile robot navigation. The key concepts shown:
//! - **Global map**: Static occupancy grid of the environment
//! - **Local costmap**: Dynamic, robot-centered map updated from sensors
//! - **Rolling window**: The local costmap moves with the robot, keeping it centered
//! - **Ray clearing**: Marking cells as free along sensor beams (inverse sensor model)
//! - **Inflation**: Expanding obstacles to account for robot size and safety margins
//!
//! This pattern allows robots to navigate using both prior maps and real-time sensor data.

use std::error::Error;
use std::f32::consts::TAU;
use std::time::Duration;

use costmap::rerun_viz::{log_costmap, log_occupancy_grid_static, log_point3d};
use costmap::types::{COST_FREE, Pose2};
use costmap::{InflationConfig, LayeredCostmap, ObstacleConfig, ObstacleLayer};
use costmap::{MapInfo, OccupancyGrid, RosMapLoader, WavefrontInflationLayer};
use glam::{DVec2, Vec2};

const DEFAULT_YAML_PATH: &str = "tests/fixtures/warehouse.yaml";

// Simulation parameters
const DELAY_MS: u64 = 100;
const WAYPOINT_SPEED_MPS: f32 = 1.1;
const WAYPOINTS: &[(f64, f64)] = &[(258.0, 98.0), (229.0, 98.0), (229.0, 62.0), (258.0, 62.0)];

// Lidar configuration
const N_BEAMS: usize = 60; // 360° / 60 = 6° angular resolution
const MAX_RANGE_M: f32 = 12.0;

// Local costmap configuration
/// Size of the robot-centered rolling window costmap (square, in cells)
const LOCAL_SIZE_CELLS: u32 = 240;

// Visualization Z-heights for layering elements in Rerun
const Z_GLOBAL: f32 = 0.0;
const Z_LOCAL: f32 = 0.12;
const Z_ROBOT: f32 = 0.35;

/// Simulates a lidar by raycasting the global map from `pose`.
///
/// Returns one range per beam: the distance to the centre of the first occupied
/// cell, or infinity for a beam that hits nothing within [`MAX_RANGE_M`]. A real
/// robot would receive these from a laser scan topic.
fn simulate_scan(global_grid: &OccupancyGrid, pose: Pose2) -> Vec<f32> {
    let beam_step = TAU / N_BEAMS as f32;
    (0..N_BEAMS)
        .map(|i| {
            let angle = pose.yaw + beam_step * i as f32;
            let dir = Vec2::new(angle.cos(), angle.sin());
            global_grid
                .raycast_dda(pose.position, dir, MAX_RANGE_M)
                .map_or(f32::INFINITY, |hit| {
                    global_grid
                        .lattice()
                        .cell_center(hit.cell)
                        .distance(pose.position) as f32
                })
        })
        .collect()
}

fn main() -> Result<(), Box<dyn Error>> {
    // Step 1: Load the global map (static environment representation)
    let global_grid = RosMapLoader::load_from_yaml(DEFAULT_YAML_PATH)?;
    let info = *global_grid.info();

    // Step 2: Set up visualization (optional - Rerun is not required to use the library)
    let rec = rerun::RecordingStreamBuilder::new("costmap_local_costmap_lidar").spawn()?;
    // Static: the global map never changes, so log it on all timelines rather than at
    // a single timepoint — otherwise it would only be visible before frame 0.
    log_occupancy_grid_static(&rec, "world/global_map", &global_grid, Z_GLOBAL)?;

    let dt = DELAY_MS as f32 / 1000.0;
    let waypoints: Vec<DVec2> = WAYPOINTS.iter().map(|(x, y)| DVec2::new(*x, *y)).collect();

    // Step 3: Create layered costmap (rolling window, sensor layer + inflation layer)
    let local_info = MapInfo::square(LOCAL_SIZE_CELLS, info.resolution);
    let mut layered = LayeredCostmap::new(local_info, COST_FREE, true);
    let obstacle_layer = layered.add_layer(Box::new(ObstacleLayer::new(
        local_info,
        ObstacleConfig {
            rolling_window: true,
            obstacle_range_m: MAX_RANGE_M,
            raytrace_range_m: MAX_RANGE_M,
            ..Default::default()
        },
    )))?;
    layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
        inflation_radius_m: 0.9,
        inscribed_radius_m: 0.15,
        cost_scaling_factor: 3.0,
        ..Default::default()
    })))?;

    let (segment_lengths, total_length) = build_segments(&waypoints);

    // Step 4: Main loop - simulate robot moving and sensing
    let mut frame_idx: i64 = 0;
    loop {
        rec.set_time_sequence("frame", frame_idx);

        let distance = (frame_idx as f32) * WAYPOINT_SPEED_MPS * dt;
        let (robot_pos, heading_dir) =
            sample_path(&waypoints, &segment_lengths, total_length, distance);
        let heading = heading_dir.y.atan2(heading_dir.x);

        // Feed the simulated scan to the obstacle layer, then run the layered update:
        // the layer re-centres on the robot and writes into the master.
        let pose = Pose2::new(robot_pos, heading);
        let ranges = simulate_scan(&global_grid, pose);
        if let Some(layer) = layered.layer_mut::<ObstacleLayer>(obstacle_layer) {
            layer.insert_laser_scan(pose, 0.0, TAU / N_BEAMS as f32, &ranges, 0.0);
        }
        layered.update_map(pose);

        log_point3d(
            &rec,
            "world/robot",
            robot_pos.as_vec2().extend(Z_ROBOT),
            Some(rerun::Color::from_rgb(0, 200, 255)),
            Some(5.0),
        )?;

        log_costmap(&rec, "world/local_costmap", layered.master(), Z_LOCAL)?;

        if DELAY_MS > 0 {
            std::thread::sleep(Duration::from_millis(DELAY_MS));
        }

        frame_idx = frame_idx.wrapping_add(1);
    }
}

// Helper functions for path following - not core library APIs

fn build_segments(waypoints: &[DVec2]) -> (Vec<f32>, f32) {
    let mut lengths = Vec::with_capacity(waypoints.len());
    let mut total = 0.0;

    if waypoints.len() < 2 {
        return (lengths, total);
    }

    for idx in 0..waypoints.len() {
        let a = waypoints[idx];
        let b = waypoints[(idx + 1) % waypoints.len()];
        let len = (b - a).length() as f32;
        lengths.push(len);
        total += len;
    }

    (lengths, total)
}

fn sample_path(
    waypoints: &[DVec2],
    segment_lengths: &[f32],
    total_length: f32,
    distance: f32,
) -> (DVec2, Vec2) {
    if waypoints.is_empty() {
        return (DVec2::ZERO, Vec2::X);
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
            let pos = a + dir * f64::from(remaining);
            let heading = if dir.length_squared() == 0.0 {
                Vec2::X
            } else {
                dir.as_vec2()
            };
            return (pos, heading);
        }
        remaining -= *seg_len;
    }

    (waypoints[0], Vec2::X)
}
