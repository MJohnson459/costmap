use glam::Vec2;

use crate::types::OCCUPIED;
use crate::{OccupancyGrid, iterators::line::LineIterator};
use crate::{raycast::RayHit2D, types::FREE};

impl OccupancyGrid {
    /// Fast voxel traversal (Amanatides & Woo) that returns the first occupied cell hit.
    ///
    /// # Arguments
    ///
    /// * `origin` - The origin of the ray in world coordinates.
    /// * `dir` - The direction of the ray in world coordinates.
    /// * `max_t` - The maximum distance of the ray in world coordinates.
    ///
    /// # Returns
    ///
    /// A `RayHit2D` struct containing the cell and hit distance.
    pub fn raycast_dda(&self, origin: &Vec2, dir: &Vec2, max_t: f32) -> Option<RayHit2D> {
        let iter = LineIterator::new(self, origin, dir, max_t)?;

        let resolution = self.info().resolution;
        for step in iter {
            let value = self.get(&step.cell).unwrap_or(&FREE);
            if value >= &OCCUPIED {
                return Some(RayHit2D {
                    cell: step.cell,
                    hit_distance: step.t * resolution,
                });
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
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
            depth: 1,
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

        let hit = grid
            .raycast_dda(&Vec2::ZERO, &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 4.1231055, epsilon = 1e-4);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None, 1.0, Vec2::ZERO);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = grid.raycast_dda(&Vec2::ZERO, &dir, 30.0);
        assert!(hit.is_none());
    }

    #[test]
    fn hit_negative_x() {
        let goal = UVec2::new(0, 1);
        let grid = test_grid(Some(goal), 1.0, Vec2::ZERO);
        let dir = Vec2::new(-3.9, 1.0).normalize();
        let hit = grid
            .raycast_dda(&Vec2::new(4.0, 0.0), &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 4.026176, epsilon = 1e-4);
    }

    #[test]
    fn resolution_with_offset() {
        let goal = UVec2::new(2, 4);
        let grid = test_grid(Some(goal), 0.05, Vec2::new(-0.18, 0.0));
        let dir = Vec2::new(-1.0, 2.0).normalize();
        let hit = grid
            .raycast_dda(&Vec2::new(0.04, 0.0), &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 0.2236068, epsilon = 1e-4);
    }
}
