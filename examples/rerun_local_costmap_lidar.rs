use std::error::Error;

use glam::{UVec2, Vec2, Vec3};
use std::f32::consts::TAU;
use std::time::Duration;
use voxel_grid::raycast::RayHit2D;
use voxel_grid::rerun_viz::{COST_LETHAL, costmap_to_rgb_bytes, log_textured_plane_mesh3d};
use voxel_grid::rerun_viz::{log_point3d, occupancy_to_rgb_bytes};
use voxel_grid::{Grid2d, MapInfo, RosMapLoader};

const DEFAULT_YAML_PATH: &str = "tests/fixtures/warehouse.yaml";
const DELAY_MS: u64 = 100;
const N_BEAMS: usize = 60;
const MAX_RANGE_M: f32 = 12.0;
const LOCAL_SIZE_CELLS: u32 = 240;
const INFLATION_RADIUS_M: f32 = 0.9;
const WAYPOINT_SPEED_MPS: f32 = 1.1;

const WAYPOINTS: &[(f32, f32)] = &[(258.0, 98.0), (229.0, 98.0), (229.0, 62.0), (258.0, 62.0)];

const Z_GLOBAL: f32 = 0.0;
const Z_LOCAL: f32 = 0.12;
const Z_INFLATED: f32 = 0.18;
const Z_RAYS: f32 = 0.3;
const Z_ROBOT: f32 = 0.35;

fn main() -> Result<(), Box<dyn Error>> {
    let yaml_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_YAML_PATH.to_string());

    let grid = RosMapLoader::load_from_yaml(&yaml_path)?;
    let info = grid.info().clone();

    let map_width_m = info.width as f32 * info.resolution;
    let map_height_m = info.height as f32 * info.resolution;
    let map_center = info.origin.truncate() + Vec2::new(0.5 * map_width_m, 0.5 * map_height_m);

    let rec = rerun::RecordingStreamBuilder::new("voxel_grid_rerun_local_costmap_lidar").spawn()?;

    let (global_width, global_height, global_rgb) = occupancy_to_rgb_bytes(&grid);
    log_textured_plane_mesh3d(
        &rec,
        "world/global_map",
        info.origin.truncate(),
        map_width_m,
        map_height_m,
        Z_GLOBAL,
        global_width,
        global_height,
        global_rgb,
    )?;

    let dt = DELAY_MS as f32 / 1000.0;

    let mut waypoints: Vec<Vec2> = WAYPOINTS.iter().map(|(x, y)| Vec2::new(*x, *y)).collect();
    if waypoints.len() < 2 {
        let half_span = 0.2 * map_width_m.min(map_height_m);
        waypoints = vec![
            map_center + Vec2::new(-half_span, -half_span),
            map_center + Vec2::new(half_span, -half_span),
            map_center + Vec2::new(half_span, half_span),
            map_center + Vec2::new(-half_span, half_span),
        ];
    }

    let local_size_m = LOCAL_SIZE_CELLS as f32 * info.resolution;
    let initial_origin = map_center - Vec2::splat(0.5 * local_size_m);
    let local_info = MapInfo {
        width: LOCAL_SIZE_CELLS,
        height: LOCAL_SIZE_CELLS,
        depth: 1,
        resolution: info.resolution,
        origin: initial_origin.extend(0.0),
    };

    let mut local_costmap = Grid2d::<u8>::empty(local_info.clone());
    let mut inflated_costmap = Grid2d::<u8>::empty(local_info);

    let (segment_lengths, total_length) = build_segments(&waypoints);

    let mut frame_idx: i64 = 0;
    loop {
        rec.set_time_sequence("frame", frame_idx);

        let distance = (frame_idx as f32) * WAYPOINT_SPEED_MPS * dt;
        let (robot_pos, heading_dir) =
            sample_path(&waypoints, &segment_lengths, total_length, distance);
        let heading = heading_dir.y.atan2(heading_dir.x);

        let local_origin = robot_pos - Vec2::splat(0.5 * local_size_m);
        local_costmap.update_origin(&local_origin.extend(0.0));
        inflated_costmap.update_origin(&local_origin.extend(0.0));

        let mut ray_segments = Vec::with_capacity(N_BEAMS);
        let beam_step = TAU / N_BEAMS as f32;

        for beam_idx in 0..N_BEAMS {
            let angle = heading + beam_step * beam_idx as f32;
            let dir = Vec2::new(angle.cos(), angle.sin()).normalize_or_zero();

            let hit: Option<RayHit2D> = grid.raycast_dda(&robot_pos, &dir, MAX_RANGE_M);
            let t = hit.map(|h| h.hit_distance).unwrap_or(MAX_RANGE_M);
            let end_world = robot_pos + dir * t;

            if let Some(mut iter) = local_costmap.line_value_mut(&robot_pos, &dir, t) {
                for cell in &mut iter {
                    *cell = 0;
                }
            }

            if let Some(hit) = hit {
                let hit_world =
                    grid.map_to_world(&hit.cell.as_vec2()) + Vec2::splat(0.5 * info.resolution);
                if let Some(map_pos) = local_costmap.world_to_map(&hit_world) {
                    let cell = map_pos.floor().as_uvec2();
                    let _ = local_costmap.set(&cell, COST_LETHAL);
                }
            }

            ray_segments.push([
                [robot_pos.x, robot_pos.y, Z_RAYS],
                [end_world.x, end_world.y, Z_RAYS],
            ]);
        }

        let colors = vec![rerun::Color::from_rgb(255, 128, 0); ray_segments.len()];
        let radii = vec![rerun::Radius::new_ui_points(1.0); ray_segments.len()];
        rec.log(
            "world/rays",
            &rerun::LineStrips3D::new(ray_segments)
                .with_colors(colors)
                .with_radii(radii),
        )?;

        log_point3d(
            &rec,
            "world/robot",
            Vec3::new(robot_pos.x, robot_pos.y, Z_ROBOT),
            Some(rerun::Color::from_rgb(0, 200, 255)),
            Some(5.0),
        )?;

        inflate_costmap(&local_costmap, &mut inflated_costmap, INFLATION_RADIUS_M);

        log_costmap(&rec, "world/local_costmap", &local_costmap, Z_LOCAL)?;
        log_costmap(
            &rec,
            "world/inflated_costmap",
            &inflated_costmap,
            Z_INFLATED,
        )?;

        if DELAY_MS > 0 {
            std::thread::sleep(Duration::from_millis(DELAY_MS));
        }

        frame_idx = frame_idx.wrapping_add(1);
    }
}

fn log_costmap(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    costmap: &Grid2d<u8>,
    z_world: f32,
) -> Result<(), Box<dyn Error>> {
    let info = costmap.info();
    let width_world = info.width as f32 * info.resolution;
    let height_world = info.height as f32 * info.resolution;
    let origin_xy_world = info.origin.truncate();

    let (width, height, rgb_bytes) = costmap_to_rgb_bytes(costmap);
    log_textured_plane_mesh3d(
        rec,
        entity_path,
        origin_xy_world,
        width_world,
        height_world,
        z_world,
        width,
        height,
        rgb_bytes,
    )?;

    Ok(())
}

fn inflate_costmap(local: &Grid2d<u8>, inflated: &mut Grid2d<u8>, inflation_radius_m: f32) {
    for (_, value) in inflated.iter_cells_mut() {
        *value = 0;
    }

    let resolution = local.info().resolution;
    if resolution <= 0.0 {
        return;
    }

    let radius_cells = (inflation_radius_m / resolution).ceil() as i32;
    if radius_cells <= 0 {
        return;
    }

    let radius_cells_f = radius_cells as f32;

    for (cell, cost) in local.iter_cells() {
        if *cost < COST_LETHAL {
            continue;
        }

        let cx = cell.x as i32;
        let cy = cell.y as i32;

        for dy in -radius_cells..=radius_cells {
            for dx in -radius_cells..=radius_cells {
                let dist = ((dx * dx + dy * dy) as f32).sqrt();
                if dist > radius_cells_f {
                    continue;
                }

                let nx = cx + dx;
                let ny = cy + dy;
                if nx < 0 || ny < 0 {
                    continue;
                }

                let pos = UVec2::new(nx as u32, ny as u32);
                if pos.x >= inflated.width() || pos.y >= inflated.height() {
                    continue;
                }

                let inflated_cost = inflation_cost(dist, radius_cells_f);
                let current = inflated.get(&pos).copied().unwrap_or(0);
                if inflated_cost > current {
                    let _ = inflated.set(&pos, inflated_cost);
                }
            }
        }
    }
}

fn inflation_cost(dist_cells: f32, radius_cells: f32) -> u8 {
    if dist_cells <= 0.0 {
        return COST_LETHAL;
    }

    let t = (1.0 - dist_cells / radius_cells).clamp(0.0, 1.0);
    (COST_LETHAL as f32 * t).round() as u8
}

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
