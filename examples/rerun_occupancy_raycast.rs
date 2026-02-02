use std::error::Error;

#[cfg(not(feature = "rerun"))]
fn main() {
    eprintln!(
        "This example requires the `rerun` feature.\n\n\
         Try:\n  cargo run --example rerun_occupancy_raycast --features rerun -- [map.yaml]\n"
    );
}

#[cfg(feature = "rerun")]
fn main() -> Result<(), Box<dyn Error>> {
    use std::f32::consts::TAU;
    use std::time::Duration;

    use glam::Vec2;
    use voxel_grid::RosMapLoader;
    use voxel_grid::raycast::RayHit2D;
    use voxel_grid::visualization::occupancy_grid_to_image;

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

    // Build an RGB texture from the crate’s grayscale visualization.
    // (Mesh3D albedo textures currently support only sRGB(A).)
    let gray = occupancy_grid_to_image(&grid);
    let (width, height) = (gray.width(), gray.height());
    let gray_bytes = gray.into_raw();
    let rgb_bytes = gray_to_rgb(&gray_bytes);

    // Place the map as a textured plane in world coordinates.
    let width_world = width as f32 * info.resolution;
    let height_world = height as f32 * info.resolution;
    let origin_xy_world = info.origin.truncate();

    log_map_mesh(
        &rec,
        origin_xy_world,
        width_world,
        height_world,
        width,
        height,
        rgb_bytes,
    )?;

    // Debug: log the map center in world coordinates.
    let center = origin_xy_world + Vec2::new(0.5 * width_world, 0.5 * height_world);
    log_center_marker(&rec, center, Z_CENTER_MARKER)?;

    // Animate raycasts from poses around a circle, aiming toward the map center.
    // Everything is logged in absolute world coordinates to avoid transform propagation issues.
    log_origin_marker(&rec, ray_origin_world, Z_ORIGIN_MARKER)?;

    // Run continuously so you can watch the ray sweep.
    //
    // `frames` is interpreted as "frames per revolution" now.
    let mut frame_idx: i64 = 0;

    loop {
        rec.set_time_sequence("frame", frame_idx);

        let phase = (frame_idx.rem_euclid(frames_per_rev) as f32 / frames_per_rev as f32) * TAU;
        // Cast rays from a fixed origin in the map.
        let origin_world = ray_origin_world;
        let dir = Vec2::new(phase.cos(), phase.sin()).normalize_or_zero();

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

#[cfg(feature = "rerun")]
fn log_map_mesh(
    rec: &rerun::RecordingStream,
    origin_xy_world: glam::Vec2,
    width_world: f32,
    height_world: f32,
    width_px: u32,
    height_px: u32,
    rgb_bytes: Vec<u8>,
) -> Result<(), Box<dyn Error>> {
    rec.log(
        "world/map",
        &rerun::Mesh3D::new([
            [origin_xy_world.x, origin_xy_world.y + height_world, 0.0],
            [origin_xy_world.x + width_world, origin_xy_world.y, 0.0],
            [origin_xy_world.x, origin_xy_world.y, 0.0],
            [
                origin_xy_world.x + width_world,
                origin_xy_world.y + height_world,
                0.0,
            ],
        ])
        .with_vertex_normals([[0.0, 0.0, 1.0]])
        .with_triangle_indices([[2, 1, 0], [3, 1, 0]])
        // Flip V to match the viewer's texture coordinate convention.
        .with_vertex_texcoords([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]])
        .with_albedo_texture(
            rerun::datatypes::ImageFormat {
                width: width_px,
                height: height_px,
                color_model: Some(rerun::datatypes::ColorModel::RGB),
                channel_datatype: Some(rerun::datatypes::ChannelDatatype::U8),
                ..Default::default()
            },
            rgb_bytes,
        ),
    )?;
    Ok(())
}

#[cfg(feature = "rerun")]
fn log_center_marker(
    rec: &rerun::RecordingStream,
    center: glam::Vec2,
    z: f32,
) -> Result<(), Box<dyn Error>> {
    rec.log(
        "world/map_center",
        &rerun::Points3D::new([[center.x, center.y, z]])
            .with_colors([rerun::Color::from_rgb(255, 255, 0)])
            .with_radii([rerun::Radius::new_ui_points(4.0)]),
    )?;
    Ok(())
}

#[cfg(feature = "rerun")]
fn log_origin_marker(
    rec: &rerun::RecordingStream,
    origin: glam::Vec2,
    z: f32,
) -> Result<(), Box<dyn Error>> {
    rec.log(
        "world/ray_origin_marker",
        &rerun::Points3D::new([[origin.x, origin.y, z]])
            .with_colors([rerun::Color::from_rgb(0, 255, 255)])
            .with_radii([rerun::Radius::new_ui_points(4.0)]),
    )?;
    Ok(())
}

#[cfg(feature = "rerun")]
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
    rec.log(
        "world/ray",
        &rerun::LineStrips3D::new([[[origin.x, origin.y, z_ray], [end.x, end.y, z_ray]]])
            .with_colors([rerun::Color::from_rgb(255, 0, 0)])
            // UI radii makes this visible regardless of zoom level.
            .with_radii([rerun::Radius::new_ui_points(1.0)]),
    )?;

    // Sensor origin point.
    rec.log(
        "world/ray_origin",
        &rerun::Points3D::new([[origin.x, origin.y, z_origin]])
            .with_colors([rerun::Color::from_rgb(0, 128, 255)])
            .with_radii([rerun::Radius::new_ui_points(3.0)]),
    )?;

    // Hit point (if any).
    if hit {
        rec.log(
            "world/ray_hit",
            &rerun::Points3D::new([[end.x, end.y, z_hit]])
                .with_colors([rerun::Color::from_rgb(0, 255, 0)])
                .with_radii([rerun::Radius::new_ui_points(3.0)]),
        )?;
    } else {
        // Clear hit when no hit is present this frame.
        rec.log("world/ray_hit", &rerun::Points3D::clear_fields())?;
    }

    Ok(())
}

#[cfg(feature = "rerun")]
fn gray_to_rgb(gray: &[u8]) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(gray.len() * 3);
    for &v in gray {
        rgb.push(v);
        rgb.push(v);
        rgb.push(v);
    }
    rgb
}

// (AABB clipping intentionally removed — we use a lidar-like fixed max range instead.)
