use glam::{IVec2, Vec2};

use crate::grid::OccupancyGrid;
use crate::raycast::RayHit2D;
use crate::types::OCCUPIED;

/// Grid-step traversal (Bresenham-style) over all crossed cells until hit.
pub fn raycast_grid_step(
    grid: &OccupancyGrid,
    origin: &Vec2,
    dir: &Vec2,
    max_t: f32,
) -> Option<RayHit2D> {
    if dir.length_squared() == 0.0 {
        return None;
    }

    let info = grid.info();
    let resolution = info.resolution;
    let dir = dir.normalize();

    let start = grid.world_to_map(&origin)?;
    let end = start + dir * (max_t / resolution);

    let mut x0 = start.floor().x as i32;
    let mut y0 = start.floor().y as i32;
    let x1 = end.floor().x as i32;
    let y1 = end.floor().y as i32;

    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        let cell = IVec2::new(x0, y0);
        if in_bounds(cell, info.width as i32, info.height as i32) && is_occupied(grid, cell) {
            let center = grid.map_to_world(&cell.as_vec2());
            let hit_distance = (center - origin).dot(dir).max(0.0);
            return Some(RayHit2D {
                cell: cell.as_uvec2(),
                hit_distance,
            });
        }

        if x0 == x1 && y0 == y1 {
            break;
        }

        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }

    None
}

fn in_bounds(cell: IVec2, width: i32, height: i32) -> bool {
    cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height
}

fn is_occupied(grid: &OccupancyGrid, cell: IVec2) -> bool {
    if cell.x < 0 || cell.y < 0 {
        return false;
    }
    let value = grid.get(&cell.as_uvec2()).unwrap_or(0);
    value >= OCCUPIED
}

#[cfg(test)]
mod tests {
    use glam::UVec2;

    use super::*;
    use crate::types::{FREE, OCCUPIED};

    fn test_grid(occupied: Option<UVec2>) -> OccupancyGrid {
        let width = 5;
        let height = 5;
        let mut data = vec![FREE; width * height];
        if let Some(cell) = occupied {
            let idx = (cell.y as usize) * width + (cell.x as usize);
            data[idx] = OCCUPIED;
        }

        let info = crate::types::MapInfo {
            width: width as u32,
            height: height as u32,
            depth: 1,
            resolution: 1.0,
            origin: glam::Vec3::new(0.0, 0.0, 0.0),
        };

        OccupancyGrid::new(info, data).expect("grid should build")
    }

    #[test]
    fn hit_positive() {
        let goal = UVec2::new(4, 1);
        let grid = test_grid(Some(goal));
        let dir = Vec2::new(4.0, 1.0).normalize();

        let hit = raycast_grid_step(&grid, &Vec2::ZERO, &dir, 30.0).expect("hit expected");
        assert_eq!(hit.cell, goal);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = raycast_grid_step(&grid, &Vec2::ZERO, &dir, 30.0);
        assert!(hit.is_none());
    }
}
