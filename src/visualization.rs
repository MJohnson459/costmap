use image::{GrayImage, Luma};

use crate::grid::OccupancyGrid;
use crate::types::{FREE, OCCUPIED};

pub fn occupancy_grid_to_image(grid: &OccupancyGrid) -> GrayImage {
    let width = grid.width();
    let height = grid.height();
    let mut image = GrayImage::new(width, height);

    for img_y in 0..height {
        for x in 0..width {
            let grid_y = height - img_y - 1;
            let value = grid.get(x, grid_y).unwrap_or(-1);
            let color = match value {
                OCCUPIED => 0,
                FREE => 254,
                _ => 205,
            };
            image.put_pixel(x, img_y, Luma([color]));
        }
    }

    image
}
