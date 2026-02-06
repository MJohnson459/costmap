use std::error::Error;

use glam::{Vec2, Vec3};

use crate::{Grid2d, OccupancyGrid, visualization::occupancy_grid_to_image};

// Re-export cost constants so existing `use voxel_grid::rerun_viz::COST_LETHAL` still works.
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

fn gray_to_rgb(gray: &[u8]) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(gray.len() * 3);
    for &v in gray {
        rgb.push(v);
        rgb.push(v);
        rgb.push(v);
    }
    rgb
}

fn cost_to_rgb(cost: u8) -> [u8; 3] {
    if cost == COST_UNKNOWN {
        return [64, 64, 64];
    }
    if cost >= COST_LETHAL {
        return [255, 0, 0];
    }

    let t = (cost as f32 / COST_LETHAL as f32).clamp(0.0, 1.0);
    let g = (255.0 * (1.0 - t)) as u8;
    [255, g, g]
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
        info.origin_xy(),
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
        info.origin_xy(),
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
    use glam::Vec3;

    use super::*;
    use crate::types::{FREE, UNKNOWN};
    use crate::{MapInfo, types::OCCUPIED};

    #[test]
    fn occupancy_to_rgb_matches_gray_image() {
        let info = MapInfo {
            width: 2,
            height: 2,
            depth: 1,
            resolution: 1.0,
            origin: Vec3::ZERO,
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
            depth: 1,
            resolution: 1.0,
            origin: Vec3::ZERO,
        };
        let costmap = Grid2d::new(info, vec![0, 10, COST_LETHAL, COST_UNKNOWN]).unwrap();

        let (width, height, rgb) = costmap_to_rgb_bytes(&costmap);
        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(rgb.len(), 2 * 2 * 3);

        let g = (255.0 * (1.0 - 10.0 / COST_LETHAL as f32)) as u8;
        let expected = [
            [255, 0, 0],     // y=1, x=0 (lethal)
            [64, 64, 64],    // y=1, x=1 (unknown)
            [255, 255, 255], // y=0, x=0 (free)
            [255, g, g],     // y=0, x=1 (low cost)
        ];

        for (idx, rgb_triplet) in expected.iter().enumerate() {
            let base = idx * 3;
            assert_eq!(&rgb[base..base + 3], rgb_triplet);
        }
    }
}
