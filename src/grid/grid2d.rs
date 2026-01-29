use glam::{IVec2, UVec2, UVec3, Vec2, Vec3};

use crate::{
    grid::Grid,
    iterators::line::{LineIterator, LineValueIterator},
};
use crate::{
    iterators::line::LineValueMutIterator,
    types::{MapInfo, VoxelError},
};

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

    pub fn get_mut(&mut self, pos: &UVec2) -> &mut T {
        let idx = self.index(pos);
        &mut self.data[idx]
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

    pub fn empty(info: MapInfo) -> Self
    where
        T: Default + Clone,
    {
        let expected_len = (info.width as usize) * (info.height as usize);
        Self {
            info,
            data: vec![T::default(); expected_len],
        }
    }

    /// Resize the map and update resolution/origin.
    ///
    /// This method clears the data to the default value.
    pub fn resize_map(&mut self, size: UVec2, resolution: f32, origin: &Vec3)
    where
        T: Default + Clone,
    {
        self.info.width = size.x;
        self.info.height = size.y;
        self.info.resolution = resolution;
        self.info.origin = *origin;
        self.data = vec![T::default(); size.x as usize * size.y as usize];
    }

    pub fn reset_map(&mut self, lower: UVec2, upper: UVec2)
    where
        T: Default + Clone,
    {
        let width = self.info.width;
        let height = self.info.height;
        let start_x = lower.x.min(width);
        let end_x = upper.x.min(width);
        let start_y = lower.y.min(height);
        let end_y = upper.y.min(height);

        if start_x >= end_x || start_y >= end_y {
            return;
        }

        let row_width = (end_x - start_x) as usize;
        let fill = T::default();
        let stride = width as usize;

        for y in start_y..end_y {
            let row_start = (y as usize) * stride + start_x as usize;
            let row_end = row_start + row_width;
            self.data[row_start..row_end].fill(fill.clone());
        }
    }

    pub fn update_origin(&mut self, origin: &Vec3)
    where
        T: Default + Clone,
    {
        let resolution = self.info.resolution;
        if resolution <= 0.0 {
            self.info.origin = *origin;
            self.data = vec![T::default(); self.data.len()];
            return;
        }

        let old_origin = self.info.origin;
        let cell_offset = ((origin - &old_origin) / resolution)
            .truncate()
            .floor()
            .as_ivec2();
        let grid_size = IVec2::new(self.info.width as i32, self.info.height as i32);

        self.data = self.copy_overlapping_region(grid_size, grid_size, cell_offset);

        self.info.origin = Vec3::new(
            old_origin.x + cell_offset.x as f32 * resolution,
            old_origin.y + cell_offset.y as f32 * resolution,
            old_origin.z,
        );
    }

    pub fn line(&self, origin: &Vec2, dir: &Vec2, max_t: f32) -> Option<LineIterator> {
        LineIterator::new(self, origin, dir, max_t)
    }

    pub fn line_value<'a>(
        &'a self,
        origin: &Vec2,
        dir: &Vec2,
        max_t: f32,
    ) -> Option<LineValueIterator<'a, T>> {
        LineValueIterator::new(self, origin, dir, max_t)
    }

    pub fn line_value_mut<'a>(
        &'a mut self,
        origin: &Vec2,
        dir: &Vec2,
        max_t: f32,
    ) -> Option<LineValueMutIterator<'a, T>> {
        LineValueMutIterator::new(self, origin, dir, max_t)
    }

    fn copy_overlapping_region(
        &self,
        old_size: IVec2,
        new_size: IVec2,
        cell_offset: IVec2,
    ) -> Vec<T>
    where
        T: Default + Clone,
    {
        let overlap_min = cell_offset.clamp(IVec2::ZERO, old_size);
        let overlap_max = (cell_offset + new_size).clamp(IVec2::ZERO, old_size);
        let overlap_size = (overlap_max - overlap_min).max(IVec2::ZERO);

        let cell_size_x = overlap_size.x as usize;
        let cell_size_y = overlap_size.y as usize;

        let mut local = vec![T::default(); cell_size_x * cell_size_y];
        if cell_size_x > 0 && cell_size_y > 0 {
            for y in 0..cell_size_y {
                let src_start =
                    (overlap_min.y as usize + y) * (old_size.x as usize) + overlap_min.x as usize;
                let src_end = src_start + cell_size_x;
                let dst_start = y * cell_size_x;
                local[dst_start..dst_start + cell_size_x]
                    .clone_from_slice(&self.data[src_start..src_end]);
            }
        }

        let mut new_data = vec![T::default(); new_size.x as usize * new_size.y as usize];
        let start = overlap_min - cell_offset;
        if cell_size_x > 0 && cell_size_y > 0 {
            for y in 0..cell_size_y {
                let dst_start = (start.y as usize + y) * (new_size.x as usize) + start.x as usize;
                let dst_end = dst_start + cell_size_x;
                let src_start = y * cell_size_x;
                let src_end = src_start + cell_size_x;
                new_data[dst_start..dst_end].clone_from_slice(&local[src_start..src_end]);
            }
        }

        new_data
    }

    fn index(&self, pos: &UVec2) -> usize {
        (pos.y as usize) * (self.info.width as usize) + (pos.x as usize)
    }
}

impl<T> Grid for Grid2d<T> {
    type Cell = T;

    fn info(&self) -> &MapInfo {
        self.info()
    }

    fn get(&self, pos: &UVec3) -> Option<&Self::Cell> {
        Grid2d::get(self, &pos.truncate())
    }

    fn set(&mut self, pos: &UVec3, value: Self::Cell) -> Result<(), VoxelError> {
        Grid2d::set(self, &pos.truncate(), value)
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

    #[test]
    fn test_update_origin_shifts_data() {
        let mut grid = Grid2d::<i8>::new(
            MapInfo {
                width: 4,
                height: 4,
                depth: 1,
                resolution: 1.0,
                origin: Vec3::new(0.0, 0.0, 0.0),
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(&UVec2::new(1, 1), 7).unwrap();
        grid.update_origin(&Vec3::new(1.0, 0.0, 0.0));

        assert_eq!(grid.get(&UVec2::new(0, 1)), Some(&7));
        assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn test_resize_map_clears_data() {
        let mut grid = Grid2d::<i8>::new(
            MapInfo {
                width: 4,
                height: 4,
                depth: 1,
                resolution: 1.0,
                origin: Vec3::new(0.0, 0.0, 0.0),
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(&UVec2::new(1, 1), 9).unwrap();
        grid.resize_map(UVec2::new(6, 6), 1.0, &Vec3::new(1.0, 0.0, 0.0));

        assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&0));
    }
}
