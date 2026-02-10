use image::{GrayImage, Luma};

use crate::OccupancyGrid;
use crate::types::UNKNOWN;

/// Convert an occupancy grid to a grayscale image preview.
///
/// - **FREE** (0) becomes white-ish.
/// - **OCCUPIED** (100) becomes black.
/// - **UNKNOWN** (-1) becomes mid-gray.
///
/// The output image is oriented like typical map images: the grid's \(y=0\) row
/// (lowest in map coordinates) is written to the **bottom** of the image.
pub fn occupancy_grid_to_image(grid: &OccupancyGrid) -> GrayImage {
    let width = grid.width();
    let height = grid.height();
    let mut img = GrayImage::new(width, height);

    for y_img in 0..height {
        // Flip vertically to match the ROS loader’s map coordinate convention.
        let y_grid = height - 1 - y_img;
        for x in 0..width {
            let value = grid
                .get(glam::UVec2::new(x, y_grid))
                .copied()
                .unwrap_or(UNKNOWN);
            let px = occupancy_to_gray(value);
            img.put_pixel(x, y_img, Luma([px]));
        }
    }

    img
}

fn occupancy_to_gray(value: i8) -> u8 {
    // Common ROS-ish palette for quick previews:
    // - unknown: ~205
    // - free: white
    // - occupied: black
    if value == UNKNOWN {
        return 205;
    }

    let v = (value as i16).clamp(0, 100) as i16;
    // 0 (free) -> 254, 100 (occupied) -> 0
    let gray = 254 - ((v * 254) / 100);
    gray as u8
}

#[cfg(test)]
mod tests {
    use glam::UVec2;

    use super::*;
    use crate::MapInfo;
    use crate::types::{FREE, OCCUPIED};

    #[test]
    fn occupancy_grid_to_image_maps_values_and_flips_y() {
        // Grid is 2x2:
        // y=1: [OCCUPIED, FREE]
        // y=0: [UNKNOWN,  FREE]
        let info = MapInfo {
            width: 2,
            height: 2,
            resolution: 1.0,
            ..Default::default()
        };
        let grid = OccupancyGrid::new(info, vec![UNKNOWN, FREE, OCCUPIED, FREE]).unwrap();

        // Sanity check the layout we intend (row-major, y=0 first).
        assert_eq!(grid.get(UVec2::new(0, 0)).copied(), Some(UNKNOWN));
        assert_eq!(grid.get(UVec2::new(0, 1)).copied(), Some(OCCUPIED));

        let img = occupancy_grid_to_image(&grid);
        assert_eq!(img.width(), 2);
        assert_eq!(img.height(), 2);

        // Because we flip Y: image y=0 corresponds to grid y=1.
        let top_left = img.get_pixel(0, 0).0[0];
        let bottom_left = img.get_pixel(0, 1).0[0];

        assert_eq!(top_left, occupancy_to_gray(OCCUPIED));
        assert_eq!(bottom_left, occupancy_to_gray(UNKNOWN));

        // Ensure free is brighter than occupied.
        let free_px = occupancy_to_gray(FREE);
        let occ_px = occupancy_to_gray(OCCUPIED);
        assert!(free_px > occ_px);
    }
}
