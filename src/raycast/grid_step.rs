use glam::{IVec2, Vec2};

use crate::OccupancyGrid;
use crate::raycast::RayHit2D;
use crate::types::OCCUPIED;

impl OccupancyGrid {
    /// Grid-step traversal (Bresenham-style) over all crossed cells until hit.
    pub fn raycast_grid_step(&self, origin: &Vec2, dir: &Vec2, max_t: f32) -> Option<RayHit2D> {
        if dir.length_squared() == 0.0 {
            return None;
        }

        let info = self.info();
        let resolution = info.resolution;
        let dir = dir.normalize();

        let start = self.world_to_map(&origin)?;
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
        let bounds = IVec2::new(info.width as i32, info.height as i32);

        loop {
            if in_bounds(current, bounds) && is_occupied(self, current) {
                let center = self.map_to_world(&current.as_vec2());
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

fn in_bounds(cell: IVec2, bounds: IVec2) -> bool {
    cell.x >= 0 && cell.y >= 0 && cell.x < bounds.x && cell.y < bounds.y
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

        let hit = grid
            .raycast_grid_step(&Vec2::ZERO, &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = grid.raycast_grid_step(&Vec2::ZERO, &dir, 30.0);
        assert!(hit.is_none());
    }
}
