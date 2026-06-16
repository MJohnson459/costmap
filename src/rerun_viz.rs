use std::error::Error;

use glam::{Vec2, Vec3};

use crate::{Costmap, OccupancyGrid, types::UNKNOWN};

// Re-export cost constants so existing `use costmap::rerun_viz::COST_LETHAL` still works.
pub use crate::types::{COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN};

/// Log a `Costmap` as a native Rerun `GridMap` archetype.
///
/// Cost values are translated to nav2's published cost convention (see
/// [`cost_to_rviz_value`]) and logged as a single-channel (L/U8) grid coloured by
/// Rerun's built-in `RvizCostmap` colormap, which matches the classic RViz costmap
/// palette (blue→red gradient, highlight colours for special costs, transparent free
/// space). The grid is anchored at its world-space origin (lower-left corner) and
/// lifted to the given z height.
pub fn log_costmap(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    costmap: &Costmap,
    z_world: f32,
) -> Result<(), Box<dyn Error>> {
    let info = costmap.info();
    let (width, height, cells) = costmap_to_l_bytes(costmap);
    log_grid_map(
        rec,
        entity_path,
        info.origin,
        info.resolution,
        z_world,
        width,
        height,
        cells,
        rerun::components::Colormap::RvizCostmap,
    )
}

/// Log an `OccupancyGrid` (`Grid2d<i8>`) as a native Rerun `GridMap` archetype.
///
/// The raw occupancy values (ROS `nav_msgs/OccupancyGrid` convention: -1 unknown,
/// 0 free, 100 occupied) are logged as a single-channel (L/U8) grid and coloured by
/// Rerun's built-in `RvizMap` colormap (white free → black occupied, green-blue
/// unknown). The grid is anchored at its world-space origin (lower-left corner) and
/// lifted to the given z height.
pub fn log_occupancy_grid(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    grid: &OccupancyGrid,
    z_world: f32,
) -> Result<(), Box<dyn Error>> {
    let info = grid.info();
    let (width, height, cells) = occupancy_to_l_bytes(grid);
    log_grid_map(
        rec,
        entity_path,
        info.origin,
        info.resolution,
        z_world,
        width,
        height,
        cells,
        rerun::components::Colormap::RvizMap,
    )
}

/// Log a single-channel (L/U8) grid as a Rerun `GridMap` archetype.
///
/// `origin_xy_world` is the lower-left corner of the grid in world XY coordinates
/// (meters); `cell_size` is the world size of one cell (meters/pixel); `z_world` is
/// the height the grid is lifted to. `cells` are the raw u8 cell values laid out
/// with row 0 at the top of the image (Rerun's image convention), and `colormap`
/// selects how those values are coloured.
#[allow(clippy::too_many_arguments)]
fn log_grid_map(
    rec: &rerun::RecordingStream,
    entity_path: &str,
    origin_xy_world: Vec2,
    cell_size: f32,
    z_world: f32,
    width: u32,
    height: u32,
    cells: Vec<u8>,
    colormap: rerun::components::Colormap,
) -> Result<(), Box<dyn Error>> {
    rec.log(
        entity_path,
        &rerun::GridMap::new(
            cells,
            rerun::components::ImageFormat::from_color_model(
                [width, height],
                rerun::ColorModel::L,
                rerun::ChannelDatatype::U8,
            ),
            cell_size,
        )
        .with_translation([origin_xy_world.x, origin_xy_world.y, z_world])
        .with_colormap(colormap),
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

/// Build a single-channel (L/U8) cell buffer from an `OccupancyGrid`.
///
/// Rows are flipped vertically so image row 0 is the grid's top row, matching
/// Rerun's image convention (row 0 at the top, the `GridMap` origin at the
/// lower-left corner). Raw ROS occupancy values are preserved: -1 unknown maps to
/// 255, 0..=100 pass through unchanged, ready for the `RvizMap` colormap.
fn occupancy_to_l_bytes(grid: &OccupancyGrid) -> (u32, u32, Vec<u8>) {
    let width = grid.width();
    let height = grid.height();
    let mut cells = Vec::with_capacity((width * height) as usize);

    for y_img in 0..height {
        let y_grid = height - 1 - y_img;
        for x in 0..width {
            let value = grid
                .get(glam::UVec2::new(x, y_grid))
                .copied()
                .unwrap_or(UNKNOWN);
            cells.push(value as u8);
        }
    }
    (width, height, cells)
}

/// Build a single-channel (L/U8) cell buffer from a `Costmap`.
///
/// Rows are flipped vertically so image row 0 is the grid's top row, matching
/// Rerun's image convention. Raw `costmap_2d` costs are translated to the value
/// convention Rerun's `RvizCostmap` colormap expects (see [`cost_to_rviz_value`]).
fn costmap_to_l_bytes(grid: &Costmap) -> (u32, u32, Vec<u8>) {
    let width = grid.width();
    let height = grid.height();
    let mut cells = Vec::with_capacity((width * height) as usize);

    for y_img in 0..height {
        let y_grid = height - 1 - y_img;
        for x in 0..width {
            let cost = grid
                .get(glam::UVec2::new(x, y_grid))
                .copied()
                .unwrap_or(COST_UNKNOWN);
            cells.push(cost_to_rviz_value(cost));
        }
    }

    (width, height, cells)
}

/// Translate a raw `costmap_2d` cost into the value convention Rerun's `RvizCostmap`
/// colormap expects.
///
/// Rerun's `RvizCostmap` is a port of the RViz costmap palette, which is indexed by
/// the *published* cost values nav2's `Costmap2DPublisher` emits (a costmap exposed
/// as a `nav_msgs/OccupancyGrid`), not the raw 0..=255 `costmap_2d` cost. This applies
/// nav2's cost translation table: free → 0, inscribed → 99, lethal → 100, unknown →
/// 255, and the 1..=252 gradient linearly scaled into 1..=98. Without it, our raw
/// special values (253/254) and gradient land on the colormap's 99/100/101-127 bands,
/// producing spurious cyan/magenta/green artifacts.
fn cost_to_rviz_value(cost: u8) -> u8 {
    match cost {
        COST_FREE => 0,
        COST_INSCRIBED => 99,
        COST_LETHAL => 100,
        COST_UNKNOWN => 255,
        c => (1 + 97 * (c as u16 - 1) / 251) as u8,
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{FREE, UNKNOWN};
    use crate::{MapInfo, types::OCCUPIED};

    #[test]
    fn occupancy_to_l_preserves_values_and_flips() {
        let info = MapInfo {
            width: 2,
            height: 2,
            resolution: 1.0,
            ..Default::default()
        };
        // Grid data is row-major from the bottom row (ROS convention):
        //   grid row 0 (bottom): [UNKNOWN, FREE]
        //   grid row 1 (top):    [OCCUPIED, FREE]
        let grid = OccupancyGrid::init(info, vec![UNKNOWN, FREE, OCCUPIED, FREE]).unwrap();

        let (width, height, cells) = occupancy_to_l_bytes(&grid);
        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(cells.len(), 2 * 2);

        // Image row 0 is the grid's top row; raw values pass through (-1 → 255).
        assert_eq!(
            cells,
            vec![
                OCCUPIED as u8, // img (0,0) = grid top-left
                FREE as u8,     // img (1,0) = grid top-right
                UNKNOWN as u8,  // img (0,1) = grid bottom-left → 255
                FREE as u8,     // img (1,1) = grid bottom-right
            ]
        );
    }

    #[test]
    fn costmap_to_l_translates_values_and_flips() {
        let info = MapInfo {
            width: 2,
            height: 2,
            resolution: 1.0,
            ..Default::default()
        };
        // grid row 0 (bottom): [0, 10], grid row 1 (top): [LETHAL, UNKNOWN]
        let costmap = Costmap::init(info, vec![0, 10, COST_LETHAL, COST_UNKNOWN]).unwrap();

        let (width, height, cells) = costmap_to_l_bytes(&costmap);
        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(cells.len(), 2 * 2);

        // Image row 0 is the grid's top row; costs are translated to nav2's published
        // convention: 0 → 0, 10 → 1 + 97*9/251 = 4, lethal → 100, unknown → 255.
        assert_eq!(cells, vec![100, 255, 0, 4]);
    }

    #[test]
    fn cost_to_rviz_value_maps_special_and_gradient() {
        assert_eq!(cost_to_rviz_value(COST_FREE), 0);
        assert_eq!(cost_to_rviz_value(COST_INSCRIBED), 99);
        assert_eq!(cost_to_rviz_value(COST_LETHAL), 100);
        assert_eq!(cost_to_rviz_value(COST_UNKNOWN), 255);
        // Gradient endpoints scale into 1..=98, staying clear of the 99/100 specials.
        assert_eq!(cost_to_rviz_value(1), 1);
        assert_eq!(cost_to_rviz_value(252), 98);
    }
}
