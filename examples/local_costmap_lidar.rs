//! # Local Costmap with Simulated Lidar
//!
//! This example demonstrates a complete local costmap workflow using the layered costmap API:
//! - A robot moving along a predefined path
//! - Simulated 2D lidar scanning the environment (sensor layer)
//! - Local costmap generation from lidar data (rolling window following the robot)
//! - Costmap inflation for safe navigation planning (inflation layer)
//!
//! The pipeline is: **sensor layer** (ray clearing from global map) → **inflation layer** → single
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
use std::sync::Arc;
use std::time::Duration;

use costmap::raycast::RayHit2D;
use costmap::rerun_viz::{log_costmap, log_occupancy_grid, log_point3d};
use costmap::types::{COST_FREE, COST_LETHAL, COST_UNKNOWN};
use costmap::{Bounds, CellRegion, Layer, LayeredGrid2d, Pose2};
use costmap::{Grid2d, MapInfo, OccupancyGrid, RosMapLoader, WavefrontInflationLayer};
use costmap::{InflationConfig, grid::merge_overwrite};
use glam::{Vec2, Vec3};

const DEFAULT_YAML_PATH: &str = "tests/fixtures/warehouse.yaml";

// Simulation parameters
const DELAY_MS: u64 = 100;
const WAYPOINT_SPEED_MPS: f32 = 1.1;
const WAYPOINTS: &[(f32, f32)] = &[(258.0, 98.0), (229.0, 98.0), (229.0, 62.0), (258.0, 62.0)];

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

/// Example-only layer: simulates lidar by raycasting on a global occupancy grid.
/// Keeps an internal obstacle grid (like Nav2's layer costmap_) so observations
/// persist across frames; each update we shift it, draw new rays, then write to master.
struct SimLidarLayer {
    global_grid: Arc<OccupancyGrid>,
    /// Internal costmap that persists between updates (Nav2-style layer costmap_).
    obstacle_grid: Grid2d<u8>,
    last_robot: Pose2,
    max_range_m: f32,
    n_beams: usize,
}

impl Layer for SimLidarLayer {
    fn reset(&mut self) {
        self.obstacle_grid.clear();
    }

    fn is_clearable(&self) -> bool {
        true
    }

    fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds) {
        self.last_robot = robot;
        bounds.expand_to_include(robot.position);
        bounds.expand_by(self.max_range_m);
    }

    fn update_costs(&mut self, master: &mut Grid2d<u8>, region: CellRegion) {
        // 1) Update internal grid origin (rolling window) and draw new rays into it.
        self.obstacle_grid.update_center(self.last_robot.position);
        let beam_step = TAU / self.n_beams as f32;
        for beam_idx in 0..self.n_beams {
            let angle = self.last_robot.yaw + beam_step * beam_idx as f32;
            let dir = Vec2::new(angle.cos(), angle.sin());
            let hit = self
                .global_grid
                .raycast_dda(self.last_robot.position, dir, self.max_range_m);
            let t = RayHit2D::distance_or(hit, self.max_range_m);
            let endpoint = hit.map(|_| COST_LETHAL);
            self.obstacle_grid
                .clear_ray(self.last_robot.position, dir, t, COST_FREE, endpoint);
        }
        // 2) Write layer to master (do not copy unknown).
        merge_overwrite(master, &self.obstacle_grid, region);
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    // Step 1: Load the global map (static environment representation)
    let grid = RosMapLoader::load_from_yaml(DEFAULT_YAML_PATH)?;
    let info = grid.info().clone();
    let global_grid = Arc::new(grid);

    // Step 2: Set up visualization (optional - Rerun is not required to use the library)
    let rec = rerun::RecordingStreamBuilder::new("costmap_local_costmap_lidar").spawn()?;
    log_occupancy_grid(&rec, "world/global_map", global_grid.as_ref(), Z_GLOBAL)?;

    let dt = DELAY_MS as f32 / 1000.0;
    let waypoints: Vec<Vec2> = WAYPOINTS.iter().map(|(x, y)| Vec2::new(*x, *y)).collect();

    // Step 3: Create layered costmap (rolling window, sensor layer + inflation layer)
    let local_info = MapInfo::square(LOCAL_SIZE_CELLS, info.resolution);
    let mut layered = LayeredGrid2d::new(local_info.clone(), COST_FREE, true);
    layered.add_layer(Box::new(SimLidarLayer {
        global_grid: Arc::clone(&global_grid),
        obstacle_grid: Grid2d::<u8>::new_with_value(local_info.clone(), COST_UNKNOWN),
        last_robot: Pose2::default(),
        max_range_m: MAX_RANGE_M,
        n_beams: N_BEAMS,
    }));
    layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
        inflation_radius_m: 0.9,
        inscribed_radius_m: 0.15,
        cost_scaling_factor: 3.0,
        ..Default::default()
    })));

    let (segment_lengths, total_length) = build_segments(&waypoints);

    // Step 4: Main loop - simulate robot moving and sensing
    let mut frame_idx: i64 = 0;
    loop {
        rec.set_time_sequence("frame", frame_idx);

        let distance = (frame_idx as f32) * WAYPOINT_SPEED_MPS * dt;
        let (robot_pos, heading_dir) =
            sample_path(&waypoints, &segment_lengths, total_length, distance);
        let heading = heading_dir.y.atan2(heading_dir.x);

        // Layered costmap update: rolling window, then each layer writes into the master
        layered.update_map(Pose2::new(robot_pos, heading));

        log_point3d(
            &rec,
            "world/robot",
            Vec3::new(robot_pos.x, robot_pos.y, Z_ROBOT),
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
