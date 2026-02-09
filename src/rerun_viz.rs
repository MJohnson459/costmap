use std::error::Error;

use glam::{Vec2, Vec3};

use crate::{Grid2d, OccupancyGrid, visualization::occupancy_grid_to_image};

// Re-export cost constants so existing `use costmap::rerun_viz::COST_LETHAL` still works.
pub use crate::types::{COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN};

pub fn occupancy_to_rgb_bytes(grid: &OccupancyGrid) -> (u32, u32, Vec<u8>) {
    let gray = occupancy_grid_to_image(grid);
    let (width, height) = (gray.width(), gray.height());
    let gray_bytes = gray.into_raw();
    let rgb_bytes = gray_to_rgb(&gray_bytes);
    (width, height, rgb_bytes)
}

pub fn costmap_to_rgb_bytes(grid: &Grid2d<u8>) -> (u32, u32, Vec<u8>) {
    let width = grid.width();
    let height = grid.height();
    let mut rgb = Vec::with_capacity((width * height * 3) as usize);

    for y_img in 0..height {
        let y_grid = height - 1 - y_img;
        for x in 0..width {
            let cost = grid
                .get(&glam::UVec2::new(x, y_grid))
                .copied()
                .unwrap_or(COST_UNKNOWN);
            let [r, g, b] = cost_to_rgb(cost);
            rgb.push(r);
            rgb.push(g);
            rgb.push(b);
        }
    }

    (width, height, rgb)
}

pub fn log_textured_plane_mesh3d(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    origin_xy_world: Vec2,
    width_world: f32,
    height_world: f32,
    z_world: f32,
    texture_width: u32,
    texture_height: u32,
    rgb_bytes: Vec<u8>,
) -> Result<(), Box<dyn Error>> {
    rec.log(
        entity_path,
        &rerun::Mesh3D::new([
            [origin_xy_world.x, origin_xy_world.y + height_world, z_world],
            [origin_xy_world.x + width_world, origin_xy_world.y, z_world],
            [origin_xy_world.x, origin_xy_world.y, z_world],
            [
                origin_xy_world.x + width_world,
                origin_xy_world.y + height_world,
                z_world,
            ],
        ])
        .with_vertex_normals([[0.0, 0.0, 1.0]])
        .with_triangle_indices([[2, 1, 0], [3, 1, 0]])
        // Flip V to match the viewer's texture coordinate convention.
        .with_vertex_texcoords([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]])
        .with_albedo_texture(
            rerun::datatypes::ImageFormat {
                width: texture_width,
                height: texture_height,
                color_model: Some(rerun::datatypes::ColorModel::RGB),
                channel_datatype: Some(rerun::datatypes::ChannelDatatype::U8),
                ..Default::default()
            },
            rgb_bytes,
        ),
    )?;
    Ok(())
}

pub fn log_point3d(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    point: Vec3,
    color: Option<rerun::Color>,
    radius_ui_points: Option<f32>,
) -> Result<(), Box<dyn Error>> {
    let mut points = rerun::Points3D::new([[point.x, point.y, point.z]]);
    if let Some(color) = color {
        points = points.with_colors([color]);
    }
    if let Some(radius) = radius_ui_points {
        points = points.with_radii([rerun::Radius::new_ui_points(radius)]);
    }
    rec.log(entity_path, &points)?;
    Ok(())
}

pub fn log_line3d(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    start: Vec3,
    end: Vec3,
    color: Option<rerun::Color>,
    radius_ui_points: Option<f32>,
) -> Result<(), Box<dyn Error>> {
    let mut line = rerun::LineStrips3D::new([[[start.x, start.y, start.z], [end.x, end.y, end.z]]]);
    if let Some(color) = color {
        line = line.with_colors([color]);
    }
    if let Some(radius) = radius_ui_points {
        line = line.with_radii([rerun::Radius::new_ui_points(radius)]);
    }
    rec.log(entity_path, &line)?;
    Ok(())
}

/// Log a closed polygon footprint as a 3D line strip in Rerun.
///
/// `points` are in world XY (meters); the polygon is drawn at `z` height.
/// The polygon is closed by connecting the last point back to the first.
pub fn log_footprint_polygon(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    points: &[Vec2],
    z: f32,
    color: rerun::Color,
) -> Result<(), Box<dyn Error>> {
    if points.len() < 2 {
        return Ok(());
    }
    let strip: Vec<[f32; 3]> = points
        .iter()
        .map(|p| [p.x, p.y, z])
        .chain(std::iter::once([points[0].x, points[0].y, z]))
        .collect();
    let line = rerun::LineStrips3D::new([strip])
        .with_colors([color])
        .with_radii([rerun::Radius::new_ui_points(2.0)]);
    rec.log(entity_path, &line)?;
    Ok(())
}

fn gray_to_rgb(gray: &[u8]) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(gray.len() * 3);
    for &v in gray {
        rgb.push(v);
        rgb.push(v);
        rgb.push(v);
    }
    rgb
}

/// Convert a costmap cost value to a Rerun color matching the RViz costmap palette.
/// Use this to make footprint colors consistent with the costmap visualization.
pub fn cost_to_rerun_color(cost: u8) -> rerun::Color {
    let [r, g, b] = cost_to_rgb(cost);
    rerun::Color::from_rgb(r, g, b)
}

/// Map a costmap cost value to an RGB colour matching the RViz costmap palette.
fn cost_to_rgb(cost: u8) -> [u8; 3] {
    match cost {
        COST_UNKNOWN => [0, 97, 127],    // teal-grey (no information)
        COST_LETHAL => [0, 255, 255],    // cyan (lethal obstacle)
        COST_INSCRIBED => [128, 0, 255], // purple (inscribed)
        0 => [0, 172, 230],              // light blue (free space)
        c => {
            // Gradient: low cost (blue-ish) → high cost (red-ish), matching RViz.
            let t = c as f32 / (COST_INSCRIBED - 1) as f32;
            let r = (255.0 * t) as u8;
            let b = (255.0 * (1.0 - t)) as u8;
            [r, 0, b]
        }
    }
}

/// Log a `Grid2d<u8>` costmap as a textured 3D plane in Rerun.
///
/// Converts the costmap to an RGB texture using the standard costmap colour
/// palette and places it at the grid's world-space origin at the given z height.
pub fn log_costmap(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    costmap: &Grid2d<u8>,
    z_world: f32,
) -> Result<(), Box<dyn Error>> {
    let info = costmap.info();
    let (width, height, rgb_bytes) = costmap_to_rgb_bytes(costmap);
    log_textured_plane_mesh3d(
        rec,
        entity_path,
        info.origin,
        info.world_width(),
        info.world_height(),
        z_world,
        width,
        height,
        rgb_bytes,
    )
}

/// Log an `OccupancyGrid` (`Grid2d<i8>`) as a textured 3D plane in Rerun.
///
/// Converts the occupancy grid to a greyscale RGB texture and places it at the
/// grid's world-space origin at the given z height.
pub fn log_occupancy_grid(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    grid: &OccupancyGrid,
    z_world: f32,
) -> Result<(), Box<dyn Error>> {
    let info = grid.info();
    let (width, height, rgb_bytes) = occupancy_to_rgb_bytes(grid);
    log_textured_plane_mesh3d(
        rec,
        entity_path,
        info.origin,
        info.world_width(),
        info.world_height(),
        z_world,
        width,
        height,
        rgb_bytes,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{FREE, UNKNOWN};
    use crate::{MapInfo, types::OCCUPIED};

    #[test]
    fn occupancy_to_rgb_matches_gray_image() {
        let info = MapInfo {
            width: 2,
            height: 2,
            resolution: 1.0,
            ..Default::default()
        };
        let grid = OccupancyGrid::new(info, vec![UNKNOWN, FREE, OCCUPIED, FREE]).unwrap();

        let (width, height, rgb) = occupancy_to_rgb_bytes(&grid);
        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(rgb.len(), 2 * 2 * 3);

        let gray = occupancy_grid_to_image(&grid).into_raw();
        for (idx, g) in gray.iter().enumerate() {
            let base = idx * 3;
            assert_eq!(rgb[base], *g);
            assert_eq!(rgb[base + 1], *g);
            assert_eq!(rgb[base + 2], *g);
        }
    }

    #[test]
    fn costmap_to_rgb_respects_palette_and_flip() {
        let info = MapInfo {
            width: 2,
            height: 2,
            resolution: 1.0,
            ..Default::default()
        };
        let costmap = Grid2d::new(info, vec![0, 10, COST_LETHAL, COST_UNKNOWN]).unwrap();

        let (width, height, rgb) = costmap_to_rgb_bytes(&costmap);
        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(rgb.len(), 2 * 2 * 3);

        // cost 10: t = 10/252 ≈ 0.0397
        let t = 10.0 / (COST_INSCRIBED - 1) as f32;
        let r = (255.0 * t) as u8;
        let b = (255.0 * (1.0 - t)) as u8;
        let expected = [
            [0, 255, 255], // y=1, x=0 (lethal → cyan)
            [0, 97, 127],  // y=1, x=1 (unknown → teal-grey)
            [0, 172, 230], // y=0, x=0 (free → light blue)
            [r, 0, b],     // y=0, x=1 (low cost → blue-ish)
        ];

        for (idx, rgb_triplet) in expected.iter().enumerate() {
            let base = idx * 3;
            assert_eq!(&rgb[base..base + 3], rgb_triplet);
        }
    }
}
