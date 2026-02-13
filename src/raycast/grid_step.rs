use glam::{IVec2, UVec2, Vec2};

use crate::OccupancyGrid;
use crate::raycast::RayHit2D;

use super::utils;

impl OccupancyGrid {
    /// Grid-step traversal (Bresenham-style) over all crossed cells until hit.
    pub fn raycast_grid_step(&self, origin: Vec2, dir: Vec2, max_t: f32) -> Option<RayHit2D> {
        if dir.length_squared() == 0.0 {
            return None;
        }

        let info = self.info();
        let resolution = info.resolution;
        let dir = dir.normalize();

        let start = self.world_to_map_continuous(origin)?;
        let end = start + dir * (max_t / resolution);

        let mut current = start.floor().as_ivec2();
        let end_cell = end.floor().as_ivec2();
        let step = IVec2::new(
            if current.x < end_cell.x { 1 } else { -1 },
            if current.y < end_cell.y { 1 } else { -1 },
        );

        let dx = (end_cell.x - current.x).abs();
        let dy = -(end_cell.y - current.y).abs();
        let mut err = dx + dy;
        let bounds = UVec2::new(info.width, info.height);

        loop {
            if utils::in_bounds(current, bounds) && utils::is_occupied(self, current) {
                let center = self.map_to_world(current.as_uvec2());
                let hit_distance = (center - origin).dot(dir).max(0.0);
                return Some(RayHit2D {
                    cell: current.as_uvec2(),
                    hit_distance,
                });
            }

            if current == end_cell {
                break;
            }

            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                current.x += step.x;
            }
            if e2 <= dx {
                err += dx;
                current.y += step.y;
            }
        }

        None
    }
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
            resolution: 1.0,
            origin: glam::Vec2::ZERO,
        };

        OccupancyGrid::init(info, data).expect("grid should build")
    }

    #[test]
    fn hit_positive() {
        let goal = UVec2::new(4, 1);
        let grid = test_grid(Some(goal));
        let dir = Vec2::new(4.0, 1.0).normalize();

        let hit = grid
            .raycast_grid_step(Vec2::ZERO, dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = grid.raycast_grid_step(Vec2::ZERO, dir, 30.0);
        assert!(hit.is_none());
    }
}
