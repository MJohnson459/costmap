use glam::{IVec2, Vec2};

use crate::grid::OccupancyGrid;
use crate::raycast::RayHit2D;
use crate::types::OCCUPIED;

/// Fast voxel traversal (Amanatides & Woo) that returns the first occupied cell hit.
pub fn raycast_dda(grid: &OccupancyGrid, origin: Vec2, dir: Vec2, max_t: f32) -> Option<RayHit2D> {
    if dir.length_squared() == 0.0 {
        return None;
    }

    let info = grid.info();
    let resolution = info.resolution;
    let max_t_grid = max_t / resolution;
    let dir = dir.normalize();

    let start = grid.world_to_map(origin)?;

    // We use ivecs internally as the steps can be negative.
    let mut cell = start.floor().as_ivec2();
    if is_occupied(grid, cell) {
        return Some(RayHit2D {
            cell: cell.as_uvec2(),
            hit_distance: 0.0,
        });
    }

    let step = IVec2::new(dir.x.signum() as i32, dir.y.signum() as i32);
    let (t_delta_x, t_max_x) = axis_params(start.x, dir.x);
    let (t_delta_y, t_max_y) = axis_params(start.y, dir.y);

    let mut t_max = Vec2::new(t_max_x, t_max_y);
    let t_delta = Vec2::new(t_delta_x, t_delta_y);

    loop {
        let t;
        if t_max.x < t_max.y {
            t = t_max.x;
            t_max.x += t_delta.x;
            cell.x += step.x;
        } else {
            t = t_max.y;
            t_max.y += t_delta.y;
            cell.y += step.y;
        }

        if t > max_t_grid {
            eprintln!("t > max_t_grid: {}", t);
            return None;
        }

        if cell.x < 0 || cell.y < 0 || cell.x >= info.width as i32 || cell.y >= info.height as i32 {
            eprintln!("cell out of bounds: {:?}", cell);
            return None;
        }

        if is_occupied(grid, cell) {
            return Some(RayHit2D {
                cell: cell.as_uvec2(),
                hit_distance: t * resolution,
            });
        }
    }
}

fn axis_params(start: f32, dir: f32) -> (f32, f32) {
    if dir == 0.0 {
        return (f32::INFINITY, f32::INFINITY);
    }

    let step = dir.signum();
    let dist_to_boundary = if step > 0.0 {
        1.0 - start.fract()
    } else {
        start.fract()
    };

    let t_delta = (1.0 / dir).abs();
    let t_max = dist_to_boundary * t_delta;
    (t_delta, t_max)
}

fn is_occupied(grid: &OccupancyGrid, cell: IVec2) -> bool {
    if cell.x < 0 || cell.y < 0 {
        return false;
    }
    let value = grid.get(cell.x as u32, cell.y as u32).unwrap_or(0);
    value >= OCCUPIED
}

#[cfg(test)]
mod tests {
    use glam::UVec2;

    use super::*;
    use crate::types::{FREE, OCCUPIED};

    fn test_grid(occupied: Option<UVec2>, resolution: f32, origin: Vec2) -> OccupancyGrid {
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
            resolution,
            origin: origin.extend(0.0),
        };

        OccupancyGrid::new(info, data).expect("grid should build")
    }

    #[test]
    fn hit_positive() {
        let goal = UVec2::new(4, 1);
        let grid = test_grid(Some(goal), 1.0, Vec2::ZERO);
        let dir = goal.as_vec2().normalize();

        let hit = raycast_dda(&grid, Vec2::ZERO, dir, 30.0).expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert!((hit.hit_distance - 4.1231055).abs() < 1e-4);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None, 1.0, Vec2::ZERO);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = raycast_dda(&grid, Vec2::ZERO, dir, 30.0);
        assert!(hit.is_none());
    }

    #[test]
    fn hit_negative_x() {
        let goal = UVec2::new(0, 1);
        let grid = test_grid(Some(goal), 1.0, Vec2::ZERO);
        let dir = Vec2::new(-3.9, 1.0).normalize();
        let hit = raycast_dda(&grid, Vec2::new(4.0, 0.0), dir, 30.0).expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert!((hit.hit_distance - 4.026176).abs() < 1e-4);
    }

    #[test]
    fn resolution_with_offset() {
        let goal = UVec2::new(2, 4);
        let grid = test_grid(Some(goal), 0.05, Vec2::new(-0.18, 0.0));
        let dir = Vec2::new(-1.0, 2.0).normalize();
        let hit = raycast_dda(&grid, Vec2::new(0.04, 0.0), dir, 30.0).expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert!((hit.hit_distance - 0.2236068).abs() < 1e-4);
    }
}
