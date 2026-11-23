use glam::{DVec2, Vec2};

use crate::raycast::RayHit2D;
use crate::types::OCCUPIED;
use crate::{OccupancyGrid, iterators::line::LineIterator};

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
    pub fn raycast_dda(&self, origin: DVec2, dir: Vec2, max_t: f32) -> Option<RayHit2D> {
        let mut line = LineIterator::new(self, origin, dir, max_t)?;

        let resolution = self.lattice().resolution as f32;
        while let Some((offset, t)) = line.next_offset() {
            if *self.at_held(offset) >= OCCUPIED {
                return Some(RayHit2D {
                    cell: self.cell_of(offset),
                    hit_distance: t * resolution,
                });
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use glam::{IVec2, UVec2};

    use super::*;
    use crate::types::{CellRegion, FREE, Lattice, OCCUPIED};

    fn test_grid(occupied: Option<IVec2>, resolution: f64, origin: DVec2) -> OccupancyGrid {
        let width = 5;
        let height = 5;
        let mut data = vec![FREE; width * height];
        if let Some(cell) = occupied {
            let idx = (cell.y as usize) * width + (cell.x as usize);
            data[idx] = OCCUPIED;
        }

        let lattice = Lattice::new(origin, resolution);
        let cells = CellRegion::from_size(UVec2::new(width as u32, height as u32));

        OccupancyGrid::from_vec(lattice, cells, data).expect("grid should build")
    }

    #[test]
    fn hit_positive() {
        let goal = IVec2::new(4, 1);
        let grid = test_grid(Some(goal), 1.0, DVec2::ZERO);
        let dir = goal.as_vec2().normalize();

        let hit = grid
            .raycast_dda(DVec2::ZERO, dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 4.1231055, epsilon = 1e-4);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None, 1.0, DVec2::ZERO);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = grid.raycast_dda(DVec2::ZERO, dir, 30.0);
        assert!(hit.is_none());
    }

    #[test]
    fn hit_negative_x() {
        let goal = IVec2::new(0, 1);
        let grid = test_grid(Some(goal), 1.0, DVec2::ZERO);
        let dir = Vec2::new(-3.9, 1.0).normalize();
        let hit = grid
            .raycast_dda(DVec2::new(4.0, 0.0), dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 4.026176, epsilon = 1e-4);
    }

    #[test]
    fn resolution_with_offset() {
        let goal = IVec2::new(2, 4);
        let grid = test_grid(Some(goal), 0.05, DVec2::new(-0.18, 0.0));
        let dir = Vec2::new(-1.0, 2.0).normalize();
        let hit = grid
            .raycast_dda(DVec2::new(0.04, 0.0), dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 0.2236068, epsilon = 1e-4);
    }
}
