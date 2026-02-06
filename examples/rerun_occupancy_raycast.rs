use std::error::Error;

use std::f32::consts::TAU;
use std::time::Duration;
use voxel_grid::rerun_viz::{log_line3d, log_occupancy_grid, log_point3d};

use glam::{Vec2, Vec3};
use voxel_grid::RosMapLoader;
use voxel_grid::raycast::RayHit2D;

// --- Styling ---
const Z_RAY: f32 = 0.10;
const Z_ORIGIN: f32 = 0.12;
const Z_HIT: f32 = 0.13;
const Z_CENTER_MARKER: f32 = 0.15;
const Z_ORIGIN_MARKER: f32 = 0.16;

// --- Demo parameters (constants) ---
const DEFAULT_YAML_PATH: &str = "tests/fixtures/warehouse.yaml";
const FRAMES_PER_REV: i64 = 360;
const DELAY_MS: u64 = 30;
const RAY_ORIGIN_WORLD: (f32, f32) = (214.0, 72.0);
const MAX_RANGE_M: f32 = 30.0;

fn main() -> Result<(), Box<dyn Error>> {
    // Optional: allow overriding the yaml path via a single positional arg.
    let yaml_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_YAML_PATH.to_string());

    let frames_per_rev = FRAMES_PER_REV.max(1);
    let delay_ms = DELAY_MS;
    let ray_origin_world = Vec2::new(RAY_ORIGIN_WORLD.0, RAY_ORIGIN_WORLD.1);
    let max_range_m = MAX_RANGE_M;

    let grid = RosMapLoader::load_from_yaml(&yaml_path)?;
    let info = grid.info().clone();

    let rec = rerun::RecordingStreamBuilder::new("voxel_grid_rerun_occupancy_raycast").spawn()?;

    log_occupancy_grid(&rec, "world/map", &grid, 0.0)?;

    // Debug: log the map center in world coordinates.
    let center = info.world_center();
    log_point3d(
        &rec,
        "world/map_center",
        Vec3::new(center.x, center.y, Z_CENTER_MARKER),
        Some(rerun::Color::from_rgb(255, 255, 0)),
        Some(4.0),
    )?;

    // Animate raycasts from poses around a circle, aiming toward the map center.
    // Everything is logged in absolute world coordinates to avoid transform propagation issues.
    log_point3d(
        &rec,
        "world/ray_origin_marker",
        Vec3::new(ray_origin_world.x, ray_origin_world.y, Z_ORIGIN_MARKER),
        Some(rerun::Color::from_rgb(0, 255, 255)),
        Some(4.0),
    )?;

    // Run continuously so you can watch the ray sweep.
    //
    // `frames` is interpreted as "frames per revolution" now.
    let mut frame_idx: i64 = 0;

    loop {
        rec.set_time_sequence("frame", frame_idx);

        let phase = (frame_idx.rem_euclid(frames_per_rev) as f32 / frames_per_rev as f32) * TAU;
        // Cast rays from a fixed origin in the map.
        let origin_world = ray_origin_world;
        let dir = Vec2::new(phase.cos(), phase.sin());

        // Raycasting must happen in *world* coordinates.
        let hit: Option<RayHit2D> = grid.raycast_dda(&origin_world, &dir, max_range_m);

        let t = hit.map(|h| h.hit_distance).unwrap_or(max_range_m);
        let end_world = origin_world + dir * t;

        log_raycast_frame(
            &rec,
            origin_world,
            end_world,
            hit.is_some(),
            Z_RAY,
            Z_ORIGIN,
            Z_HIT,
        )?;

        // Slow down playback so the ray is visible live.
        if delay_ms > 0 {
            std::thread::sleep(Duration::from_millis(delay_ms));
        }

        frame_idx = frame_idx.wrapping_add(1);
    }
}

fn log_raycast_frame(
    rec: &rerun::RecordingStream,
    origin: glam::Vec2,
    end: glam::Vec2,
    hit: bool,
    z_ray: f32,
    z_origin: f32,
    z_hit: f32,
) -> Result<(), Box<dyn Error>> {
    // Ray line.
    log_line3d(
        rec,
        "world/ray",
        glam::Vec3::new(origin.x, origin.y, z_ray),
        glam::Vec3::new(end.x, end.y, z_ray),
        Some(rerun::Color::from_rgb(255, 0, 0)),
        Some(1.0),
    )?;

    // Sensor origin point.
    log_point3d(
        rec,
        "world/ray_origin",
        glam::Vec3::new(origin.x, origin.y, z_origin),
        Some(rerun::Color::from_rgb(0, 128, 255)),
        Some(3.0),
    )?;

    // Hit point (if any).
    if hit {
        log_point3d(
            rec,
            "world/ray_hit",
            glam::Vec3::new(end.x, end.y, z_hit),
            Some(rerun::Color::from_rgb(0, 255, 0)),
            Some(3.0),
        )?;
    } else {
        // Clear hit when no hit is present this frame.
        rec.log("world/ray_hit", &rerun::Points3D::clear_fields())?;
    }

    Ok(())
}

// (AABB clipping intentionally removed — we use a lidar-like fixed max range instead.)
