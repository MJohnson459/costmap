//! 2D occupancy and costmap grid.
//!
//! ## Coordinate frames
//!
//! - **Map coordinates**: Cell indices (UVec2) or continuous (Vec2) in the grid.
//! - **World coordinates**: Physical coordinates in meters. The origin in [`MapInfo`]
//!   gives the world coordinate of the lower-left corner of cell (0, 0).

use glam::{IVec2, USizeVec2, UVec2, Vec2};
use std::{
    ops::{Index, IndexMut},
    slice::{Iter, IterMut},
};

use crate::types::{MapInfo, VoxelError};

#[derive(Debug, Clone)]
pub struct Grid2d<T> {
    info: MapInfo,
    data: Vec<T>,
    fill_value: T,
}

impl<T> Grid2d<T> {
    /// Create a grid from existing data, using `T::default()` as the fill value.
    pub fn init(info: MapInfo, data: Vec<T>) -> Result<Self, VoxelError>
    where
        T: Default,
    {
        Self::init_with_value(info, data, T::default())
    }

    /// Create a grid from existing data with an explicit fill value.
    ///
    /// The fill value is used by [`clear`](Self::clear), [`update_origin`](Self::update_origin),
    /// [`resize_map`](Self::resize_map), and [`reset_map`](Self::reset_map) when new or
    /// reset cells need a value.
    pub fn init_with_value(info: MapInfo, data: Vec<T>, fill_value: T) -> Result<Self, VoxelError> {
        let expected_len = (info.width as usize) * (info.height as usize);
        if data.len() != expected_len {
            return Err(VoxelError::InvalidMetadata(format!(
                "data length {} does not match map size {}",
                data.len(),
                expected_len
            )));
        }

        Ok(Self {
            info,
            data,
            fill_value,
        })
    }

    /// Create an empty grid filled with `T::default()`.
    pub fn new(info: MapInfo) -> Self
    where
        T: Default + Clone,
    {
        Self::new_with_value(info, T::default())
    }

    /// Create a grid where every cell is initialised to `fill_value`.
    ///
    /// The same value is used by [`clear`](Self::clear),
    /// [`update_origin`](Self::update_origin), [`resize_map`](Self::resize_map),
    /// and [`reset_map`](Self::reset_map) when new or reset cells need a value.
    pub fn new_with_value(info: MapInfo, fill_value: T) -> Self
    where
        T: Clone,
    {
        let expected_len = (info.width as usize) * (info.height as usize);
        Self {
            data: vec![fill_value.clone(); expected_len],
            info,
            fill_value,
        }
    }

    /// Create a grid by calling `f(x, y)` for each cell in row-major order.
    pub fn new_with(info: MapInfo, mut f: impl FnMut(u32, u32) -> T) -> Self
    where
        T: Default,
    {
        let mut data = Vec::with_capacity((info.width * info.height) as usize);
        for y in 0..info.height {
            for x in 0..info.width {
                data.push(f(x, y));
            }
        }
        Self {
            info,
            data,
            fill_value: T::default(),
        }
    }

    pub fn info(&self) -> &MapInfo {
        &self.info
    }

    #[inline]
    pub fn width(&self) -> u32 {
        self.info.width
    }

    #[inline]
    pub fn height(&self) -> u32 {
        self.info.height
    }

    /// Returns a reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos.x < self.width()` and `pos.y < self.height()`.
    #[inline]
    pub unsafe fn get_unchecked(&self, pos: UVec2) -> &T {
        &self.data[self.index(pos)]
    }

    /// Returns a mutable reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos.x < self.width()` and `pos.y < self.height()`.
    #[inline]
    pub unsafe fn get_unchecked_mut(&mut self, pos: UVec2) -> &mut T {
        let idx = self.index(pos);
        &mut self.data[idx]
    }

    #[inline]
    pub fn get(&self, pos: UVec2) -> Option<&T> {
        if pos.x >= self.info.width || pos.y >= self.info.height {
            return None;
        }
        Some(unsafe { self.get_unchecked(pos) })
    }

    #[inline]
    pub fn get_mut(&mut self, pos: UVec2) -> Option<&mut T> {
        if pos.x >= self.info.width || pos.y >= self.info.height {
            return None;
        }
        Some(unsafe { self.get_unchecked_mut(pos) })
    }

    #[inline]
    pub fn set(&mut self, pos: UVec2, value: T) -> Result<(), VoxelError> {
        if pos.x >= self.info.width || pos.y >= self.info.height {
            return Err(VoxelError::OutOfBounds(format!(
                "cell ({}, {}) out of bounds for map {}x{}",
                pos.x, pos.y, self.info.width, self.info.height
            )));
        }
        *unsafe { self.get_unchecked_mut(pos) } = value;
        Ok(())
    }

    /// Map (cell indices) → world. Returns the cell center.
    #[inline]
    pub fn map_to_world(&self, cell: UVec2) -> Vec2 {
        self.map_to_world_continuous(cell.as_vec2()) + Vec2::splat(0.5 * self.info.resolution)
    }

    /// Map (continuous) → world. For integer coords, returns the cell's lower-left corner.
    #[inline]
    pub fn map_to_world_continuous(&self, pos: Vec2) -> Vec2 {
        Vec2::new(
            self.info.origin.x + pos.x * self.info.resolution,
            self.info.origin.y + pos.y * self.info.resolution,
        )
    }

    /// World → map (cell indices). Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_map(&self, pos: Vec2) -> Option<UVec2> {
        let mp = self.world_to_map_continuous(pos)?;
        Some(UVec2::new(mp.x as u32, mp.y as u32))
    }

    /// World → map (continuous). Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_map_continuous(&self, pos: Vec2) -> Option<Vec2> {
        let mx = (pos.x - self.info.origin.x) / self.info.resolution;
        let my = (pos.y - self.info.origin.y) / self.info.resolution;
        if mx < 0.0 || my < 0.0 || mx >= self.info.width as f32 || my >= self.info.height as f32 {
            return None;
        }
        Some(Vec2::new(mx, my))
    }

    /// Iterate over all cells in row-major order.
    pub fn iter(&self) -> Iter<'_, T> {
        self.data.iter()
    }

    /// Iterate over all cells in row-major order, yielding mutably.
    pub fn iter_mut(&mut self) -> IterMut<'_, T> {
        self.data.iter_mut()
    }

    /// Iterate over all cells in row-major order, yielding `(cell, value)`.
    ///
    /// Order: `(0,0), (1,0), ... (width-1,0), (0,1), ...`
    pub fn indexed_iter(&self) -> impl Iterator<Item = (UVec2, &T)> {
        let width = self.info.width as usize;
        self.iter().enumerate().map(move |(idx, v)| {
            let x = (idx % width) as u32;
            let y = (idx / width) as u32;
            (UVec2::new(x, y), v)
        })
    }

    /// Iterate over all cells in row-major order, yielding `(cell, value)` mutably.
    ///
    /// Order: `(0,0), (1,0), ... (width-1,0), (0,1), ...`
    pub fn indexed_iter_mut(&mut self) -> impl Iterator<Item = (UVec2, &mut T)> {
        let width = self.info.width as usize;
        self.iter_mut().enumerate().map(move |(idx, v)| {
            let x = (idx % width) as u32;
            let y = (idx / width) as u32;
            (UVec2::new(x, y), v)
        })
    }

    /// Reset all cells to the grid's fill value.
    pub fn clear(&mut self)
    where
        T: Clone,
    {
        self.data.fill(self.fill_value.clone());
    }

    /// Resize the map and update resolution/origin.
    ///
    /// This method clears the data to the fill value.
    pub fn resize_map(&mut self, size: UVec2, resolution: f32, origin: Vec2)
    where
        T: Clone,
    {
        self.info.width = size.x;
        self.info.height = size.y;
        self.info.resolution = resolution;
        self.info.origin = origin;
        self.data = vec![self.fill_value.clone(); size.x as usize * size.y as usize];
    }

    pub fn reset_map(&mut self, lower: UVec2, upper: UVec2)
    where
        T: Clone,
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
        let fill = self.fill_value.clone();
        let stride = width as usize;

        for y in start_y..end_y {
            let row_start = (y as usize) * stride + start_x as usize;
            let row_end = row_start + row_width;
            self.data[row_start..row_end].fill(fill.clone());
        }
    }

    /// Updates the origin of the grid.
    pub fn update_origin(&mut self, origin: Vec2)
    where
        T: Clone,
    {
        let resolution = self.info.resolution;

        let old_origin = self.info.origin;
        let cell_offset = ((origin - old_origin) / resolution).floor().as_ivec2();
        let grid_size = USizeVec2::new(self.info.width as usize, self.info.height as usize);

        let overlap_min = cell_offset.clamp(IVec2::ZERO, grid_size.as_ivec2());
        let overlap_max =
            (cell_offset + grid_size.as_ivec2()).clamp(IVec2::ZERO, grid_size.as_ivec2());
        let overlap_size = (overlap_max - overlap_min).max(IVec2::ZERO).as_usizevec2();

        let mut new_data = vec![self.fill_value.clone(); grid_size.x * grid_size.y];
        let start = (overlap_min - cell_offset).as_usizevec2();
        if overlap_size.x > 0 && overlap_size.y > 0 {
            for y in 0..overlap_size.y {
                let dst_start = (start.y + y) * grid_size.x + start.x;
                let dst_end = dst_start + overlap_size.x;
                let src_start = (overlap_min.y as usize + y) * grid_size.x + overlap_min.x as usize;
                let src_end = src_start + overlap_size.x;
                new_data[dst_start..dst_end].clone_from_slice(&self.data[src_start..src_end]);
            }
        }

        self.data = new_data;

        self.info.origin = Vec2::new(
            old_origin.x + cell_offset.x as f32 * resolution,
            old_origin.y + cell_offset.y as f32 * resolution,
        );
    }

    /// Updates the origin based on the desired center of the map.
    ///
    /// # Arguments
    ///
    /// * `center` - The desired center of the map in world coordinates.
    pub fn update_center(&mut self, center: Vec2)
    where
        T: Clone,
    {
        let origin = center
            - Vec2::new(
                self.info.width as f32 * self.info.resolution,
                self.info.height as f32 * self.info.resolution,
            ) * 0.5;
        self.update_origin(origin);
    }

    #[inline]
    fn index(&self, pos: UVec2) -> usize {
        (pos.y as usize) * (self.info.width as usize) + (pos.x as usize)
    }
}

impl<T> Index<UVec2> for Grid2d<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: UVec2) -> &Self::Output {
        self.get(index).unwrap_or_else(|| {
            panic!(
                "grid index ({}, {}) out of bounds for grid {}x{}",
                index.x, index.y, self.info.width, self.info.height
            )
        })
    }
}

impl<T> IndexMut<UVec2> for Grid2d<T> {
    #[inline]
    fn index_mut(&mut self, index: UVec2) -> &mut Self::Output {
        self.get_mut(index).unwrap_or_else(|| {
            panic!(
                "grid index ({}, {}) out of bounds for grid",
                index.x, index.y
            );
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world_to_map_to_world(grid: &Grid2d<i8>, pos: Vec2) -> Vec2 {
        let map_pos = grid.world_to_map_continuous(pos).unwrap();
        grid.map_to_world_continuous(map_pos)
    }

    #[test]
    fn test_world_to_map_to_world() {
        let grid = Grid2d::<i8>::init(
            MapInfo {
                width: 10,
                height: 10,
                resolution: 1.0,
                ..Default::default()
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
        let mut grid = Grid2d::<i8>::init(
            MapInfo {
                width: 4,
                height: 4,
                resolution: 1.0,
                ..Default::default()
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(UVec2::new(1, 1), 7).unwrap();
        grid.update_origin(Vec2::new(1.0, 0.0));

        assert_eq!(grid.get(UVec2::new(0, 1)), Some(&7));
        assert_eq!(grid.get(UVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn test_resize_map_clears_data() {
        let mut grid = Grid2d::<i8>::init(
            MapInfo {
                width: 4,
                height: 4,
                resolution: 1.0,
                ..Default::default()
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(UVec2::new(1, 1), 9).unwrap();
        grid.resize_map(UVec2::new(6, 6), 1.0, Vec2::new(1.0, 0.0));

        assert_eq!(grid.get(UVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn iter_cells_row_major_order() {
        let grid = Grid2d::<u8>::init(
            MapInfo {
                width: 3,
                height: 2,
                resolution: 1.0,
                ..Default::default()
            },
            vec![10, 11, 12, 20, 21, 22],
        )
        .unwrap();

        let cells: Vec<(UVec2, u8)> = grid.indexed_iter().map(|(p, v)| (p, *v)).collect();
        assert_eq!(
            cells,
            vec![
                (UVec2::new(0, 0), 10),
                (UVec2::new(1, 0), 11),
                (UVec2::new(2, 0), 12),
                (UVec2::new(0, 1), 20),
                (UVec2::new(1, 1), 21),
                (UVec2::new(2, 1), 22),
            ]
        );
    }

    #[test]
    fn iter_cells_mut_writes_values() {
        let mut grid = Grid2d::<u8>::init(
            MapInfo {
                width: 2,
                height: 2,
                ..Default::default()
            },
            vec![0, 1, 2, 3],
        )
        .unwrap();

        for (pos, v) in grid.indexed_iter_mut() {
            *v = (pos.x + 10 * pos.y) as u8;
        }

        assert_eq!(grid.get(UVec2::new(0, 0)), Some(&0));
        assert_eq!(grid.get(UVec2::new(1, 0)), Some(&1));
        assert_eq!(grid.get(UVec2::new(0, 1)), Some(&10));
        assert_eq!(grid.get(UVec2::new(1, 1)), Some(&11));
    }
}
