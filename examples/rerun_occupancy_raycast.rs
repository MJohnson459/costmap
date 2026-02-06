//! # Occupancy Grid Raycasting Example
//!
//! This example demonstrates 2D raycasting in an occupancy grid, a fundamental operation
//! for robotics applications such as:
//! - Laser range finder (lidar) simulation
//! - Line-of-sight checks for visibility
//! - Collision detection along a ray
//!
//! ## Robotics Context
//! In mobile robotics, raycasting is used to:
//! - Simulate sensor measurements for testing without physical hardware
//! - Check if paths are collision-free
//! - Update costmaps by marking free space along sensor beams
//!
//! This example loads a static map and continuously rotates a ray 360 degrees,
//! visualizing hits and misses using Rerun for real-time debugging.

use std::error::Error;

use std::f32::consts::TAU;
use std::time::Duration;
use voxel_grid::rerun_viz::{log_line3d, log_occupancy_grid, log_point3d};

use glam::{Vec2, Vec3};
use voxel_grid::RosMapLoader;

const DEFAULT_YAML_PATH: &str = "tests/fixtures/warehouse.yaml";

// Animation parameters
const FRAMES_PER_REV: i64 = 360;
const DELAY_MS: u64 = 30;

/// Position to cast rays from (in world frame, meters)
const RAY_ORIGIN_WORLD: (f32, f32) = (214.0, 72.0);
/// Maximum distance to check for obstacles (meters)
const MAX_RANGE_M: f32 = 30.0;

// Visualization Z-heights for layering elements in Rerun
const Z_RAY: f32 = 0.10;
const Z_ORIGIN: f32 = 0.12;
const Z_HIT: f32 = 0.13;
const Z_CENTER_MARKER: f32 = 0.15;
const Z_ORIGIN_MARKER: f32 = 0.16;

fn main() -> Result<(), Box<dyn Error>> {
    let yaml_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_YAML_PATH.to_string());

    let ray_origin_world = Vec2::new(RAY_ORIGIN_WORLD.0, RAY_ORIGIN_WORLD.1);

    // Step 1: Load an occupancy grid from a ROS-format YAML file
    let grid = RosMapLoader::load_from_yaml(&yaml_path)?;

    // Step 2: Set up Rerun for visualization (optional - remove if not needed)
    let rec = rerun::RecordingStreamBuilder::new("voxel_grid_rerun_occupancy_raycast").spawn()?;
    log_occupancy_grid(&rec, "world/map", &grid, 0.0)?;

    let center = grid.info().world_center();
    log_point3d(
        &rec,
        "world/map_center",
        Vec3::new(center.x, center.y, Z_CENTER_MARKER),
        Some(rerun::Color::from_rgb(255, 255, 0)),
        Some(4.0),
    )?;

    log_point3d(
        &rec,
        "world/ray_origin_marker",
        Vec3::new(ray_origin_world.x, ray_origin_world.y, Z_ORIGIN_MARKER),
        Some(rerun::Color::from_rgb(0, 255, 255)),
        Some(4.0),
    )?;

    // Step 3: Continuously raycast in different directions
    let mut frame_idx: i64 = 0;

    loop {
        rec.set_time_sequence("frame", frame_idx);

        // Calculate direction vector for this iteration
        let phase = (frame_idx.rem_euclid(FRAMES_PER_REV) as f32 / FRAMES_PER_REV as f32) * TAU;
        let dir = Vec2::new(phase.cos(), phase.sin());

        // Core API: raycast_dda(origin, direction, max_range)
        // Returns Some(RaycastHit) if an occupied cell is hit, None otherwise
        let hit = grid.raycast_dda(&ray_origin_world, &dir, MAX_RANGE_M);
        let t = hit.map(|h| h.hit_distance).unwrap_or(MAX_RANGE_M);
        let end_world = ray_origin_world + dir * t;

        log_raycast_frame(
            &rec,
            ray_origin_world,
            end_world,
            hit.is_some(),
            Z_RAY,
            Z_ORIGIN,
            Z_HIT,
        )?;

        if DELAY_MS > 0 {
            std::thread::sleep(Duration::from_millis(DELAY_MS));
        }

        frame_idx = frame_idx.wrapping_add(1);
    }
}

// Helper function for Rerun visualization - not essential to understanding the library
fn log_raycast_frame(
    rec: &rerun::RecordingStream,
    origin: glam::Vec2,
    end: glam::Vec2,
    hit: bool,
    z_ray: f32,
    z_origin: f32,
    z_hit: f32,
) -> Result<(), Box<dyn Error>> {
    log_line3d(
        rec,
        "world/ray",
        glam::Vec3::new(origin.x, origin.y, z_ray),
        glam::Vec3::new(end.x, end.y, z_ray),
        Some(rerun::Color::from_rgb(255, 0, 0)),
        Some(1.0),
    )?;

    log_point3d(
        rec,
        "world/ray_origin",
        glam::Vec3::new(origin.x, origin.y, z_origin),
        Some(rerun::Color::from_rgb(0, 128, 255)),
        Some(3.0),
    )?;

    if hit {
        log_point3d(
            rec,
            "world/ray_hit",
            glam::Vec3::new(end.x, end.y, z_hit),
            Some(rerun::Color::from_rgb(0, 255, 0)),
            Some(3.0),
        )?;
    } else {
        rec.log("world/ray_hit", &rerun::Points3D::clear_fields())?;
    }

    Ok(())
}
