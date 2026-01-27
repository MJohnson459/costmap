use glam::{UVec2, UVec3, Vec2};

use crate::grid::Grid;
use crate::types::{MapInfo, VoxelError};

#[derive(Debug, Clone)]
pub struct Grid2d<T> {
    info: MapInfo,
    data: Vec<T>,
}

impl<T> Grid2d<T> {
    pub fn new(info: MapInfo, data: Vec<T>) -> Result<Self, VoxelError> {
        let expected_len = (info.width as usize) * (info.height as usize);
        if data.len() != expected_len {
            return Err(VoxelError::InvalidMetadata(format!(
                "data length {} does not match map size {}",
                data.len(),
                expected_len
            )));
        }

        Ok(Self { info, data })
    }

    pub fn info(&self) -> &MapInfo {
        &self.info
    }

    pub fn width(&self) -> u32 {
        self.info.width
    }

    pub fn height(&self) -> u32 {
        self.info.height
    }

    pub fn get(&self, pos: &UVec2) -> Option<&T> {
        if pos.x >= self.info.width || pos.y >= self.info.height {
            return None;
        }
        let idx = self.index(pos);
        Some(&self.data[idx])
    }

    pub fn set(&mut self, pos: &UVec2, value: T) -> Result<(), VoxelError> {
        if pos.x >= self.info.width || pos.y >= self.info.height {
            return Err(VoxelError::OutOfBounds(format!(
                "cell ({}, {}) out of bounds for map {}x{}",
                pos.x, pos.y, self.info.width, self.info.height
            )));
        }
        let idx = self.index(pos);
        self.data[idx] = value;
        Ok(())
    }

    fn index(&self, pos: &UVec2) -> usize {
        (pos.y as usize) * (self.info.width as usize) + (pos.x as usize)
    }

    pub fn map_to_world(&self, pos: &Vec2) -> Vec2 {
        Vec2::new(
            self.info.origin.x + pos.x * self.info.resolution,
            self.info.origin.y + pos.y * self.info.resolution,
        )
    }

    pub fn world_to_map(&self, pos: &Vec2) -> Option<Vec2> {
        let mx = (pos.x - self.info.origin.x) / self.info.resolution;
        let my = (pos.y - self.info.origin.y) / self.info.resolution;
        if mx < 0.0 || my < 0.0 || mx >= self.info.width as f32 || my >= self.info.height as f32 {
            return None;
        }
        Some(Vec2::new(mx, my))
    }

    pub fn data(&self) -> &[T] {
        &self.data
    }
}

impl<T> Grid for Grid2d<T> {
    type Cell = T;

    fn info(&self) -> &MapInfo {
        self.info()
    }

    fn get(&self, pos: &UVec3) -> Option<&Self::Cell> {
        self.get(&pos.truncate())
    }

    fn set(&mut self, pos: &UVec3, value: Self::Cell) -> Result<(), VoxelError> {
        self.set(&pos.truncate(), value)
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;

    fn world_to_map_to_world(grid: &Grid2d<i8>, pos: Vec2) -> Vec2 {
        let map_pos = grid.world_to_map(&pos).unwrap();
        grid.map_to_world(&map_pos)
    }

    #[test]
    fn test_world_to_map_to_world() {
        let grid = Grid2d::<i8>::new(
            MapInfo {
                width: 10,
                height: 10,
                depth: 1,
                resolution: 1.0,
                origin: Vec3::new(0.0, 0.0, 0.0),
            },
            vec![0; 100],
        )
        .unwrap();

        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(0.0, 0.0)),
            Vec2::new(0.0, 0.0)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(1.0, 1.0)),
            Vec2::new(1.0, 1.0)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(0.5, 0.5)),
            Vec2::new(0.5, 0.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(1.5, 1.5)),
            Vec2::new(1.5, 1.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(0.5, 1.5)),
            Vec2::new(0.5, 1.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(1.5, 0.5)),
            Vec2::new(1.5, 0.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(0.5, 0.5)),
            Vec2::new(0.5, 0.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, Vec2::new(1.5, 1.5)),
            Vec2::new(1.5, 1.5)
        );
    }
}
