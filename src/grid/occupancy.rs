use glam::Vec2;

use crate::types::{MapInfo, VoxelError};

#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    info: MapInfo,
    data: Vec<i8>,
}

impl OccupancyGrid {
    pub fn new(info: MapInfo, data: Vec<i8>) -> Result<Self, VoxelError> {
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

    pub fn get(&self, x: u32, y: u32) -> Option<i8> {
        if x >= self.info.width || y >= self.info.height {
            return None;
        }
        let idx = self.index(x, y);
        Some(self.data[idx])
    }

    fn index(&self, x: u32, y: u32) -> usize {
        (y as usize) * (self.info.width as usize) + (x as usize)
    }

    pub fn map_to_world(&self, pos: Vec2) -> Vec2 {
        Vec2::new(
            self.info.origin.x + pos.x * self.info.resolution,
            self.info.origin.y + pos.y * self.info.resolution,
        )
    }

    pub fn world_to_map(&self, pos: Vec2) -> Option<Vec2> {
        let mx = (pos.x - self.info.origin.x) / self.info.resolution;
        let my = (pos.y - self.info.origin.y) / self.info.resolution;
        if mx < 0.0 || my < 0.0 || mx >= self.info.width as f32 || my >= self.info.height as f32 {
            return None;
        }
        Some(Vec2::new(mx, my))
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;

    fn world_to_map_to_world(grid: &OccupancyGrid, pos: Vec2) -> Vec2 {
        let map_pos = grid.world_to_map(pos).unwrap();
        grid.map_to_world(map_pos)
    }

    #[test]
    fn test_world_to_map_to_world() {
        let grid = OccupancyGrid::new(
            MapInfo {
                width: 10,
                height: 10,
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
