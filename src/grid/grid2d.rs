//! 2D occupancy and costmap grid.
//!
//! ## Coordinate frames
//!
//! - **Map coordinates**: Cell indices (UVec2) or continuous (Vec2) in the grid.
//! - **World coordinates**: Physical coordinates in meters. The origin in [`MapInfo`]
//!   gives the world coordinate of the lower-left corner of cell (0, 0).

use std::ops::{Index, IndexMut};
use glam::{IVec2, UVec2, UVec3, Vec2};

use crate::{
    grid::Grid,
    iterators::{
        line::{LineIterator, LineValueIterator},
        polygon::{PolygonIterator, PolygonValueIterator, PolygonValueMutIterator},
    },
};
use crate::{
    iterators::line::LineValueMutIterator,
    types::{MapInfo, VoxelError},
};

#[derive(Debug, Clone)]
pub struct Grid2d<T> {
    info: MapInfo,
    data: Vec<T>,
    fill_value: T,
}

impl<T> Grid2d<T> {
    /// Create a grid from existing data, using `T::default()` as the fill value.
    pub fn new(info: MapInfo, data: Vec<T>) -> Result<Self, VoxelError>
    where
        T: Default,
    {
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
            fill_value: T::default(),
        })
    }

    /// Create a grid from existing data with an explicit fill value.
    ///
    /// The fill value is used by [`clear`](Self::clear), [`update_origin`](Self::update_origin),
    /// [`resize_map`](Self::resize_map), and [`reset_map`](Self::reset_map) when new or
    /// reset cells need a value.
    pub fn new_with_fill(info: MapInfo, data: Vec<T>, fill_value: T) -> Result<Self, VoxelError> {
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
    pub unsafe fn get_unchecked(&self, pos: &UVec2) -> &T {
        &self.data[self.index(pos)]
    }

    /// Returns a mutable reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos.x < self.width()` and `pos.y < self.height()`.
    #[inline]
    pub unsafe fn get_unchecked_mut(&mut self, pos: &UVec2) -> &mut T {
        let idx = self.index(pos);
        &mut self.data[idx]
    }

    #[inline]
    pub fn get(&self, pos: &UVec2) -> Option<&T> {
        if pos.x >= self.info.width || pos.y >= self.info.height {
            return None;
        }
        Some(unsafe { self.get_unchecked(pos) })
    }

    #[inline]
    pub fn get_mut(&mut self, pos: &UVec2) -> &mut T {
        let idx = self.index(pos);
        &mut self.data[idx]
    }

    #[inline]
    pub fn set(&mut self, pos: &UVec2, value: T) -> Result<(), VoxelError> {
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
    pub fn map_to_world(&self, cell: &UVec2) -> Vec2 {
        self.map_to_world_continuous(&cell.as_vec2()) + Vec2::splat(0.5 * self.info.resolution)
    }

    /// Map (continuous) → world. For integer coords, returns the cell's lower-left corner.
    #[inline]
    pub fn map_to_world_continuous(&self, pos: &Vec2) -> Vec2 {
        Vec2::new(
            self.info.origin.x + pos.x * self.info.resolution,
            self.info.origin.y + pos.y * self.info.resolution,
        )
    }

    /// World → map (cell indices). Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_map(&self, pos: &Vec2) -> Option<UVec2> {
        let mp = self.world_to_map_continuous(pos)?;
        Some(UVec2::new(mp.x as u32, mp.y as u32))
    }

    /// World → map (continuous). Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_map_continuous(&self, pos: &Vec2) -> Option<Vec2> {
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

    /// Iterate over all cells in row-major order, yielding `(cell, value)`.
    ///
    /// Order: `(0,0), (1,0), ... (width-1,0), (0,1), ...`
    pub fn iter_cells(&self) -> impl Iterator<Item = (UVec2, &T)> {
        let width = self.info.width as usize;
        self.data.iter().enumerate().map(move |(idx, v)| {
            let x = (idx % width) as u32;
            let y = (idx / width) as u32;
            (UVec2::new(x, y), v)
        })
    }

    /// Iterate over all cells in row-major order, yielding `(cell, value)` mutably.
    ///
    /// Order: `(0,0), (1,0), ... (width-1,0), (0,1), ...`
    pub fn iter_cells_mut(&mut self) -> impl Iterator<Item = (UVec2, &mut T)> {
        let width = self.info.width as usize;
        self.data.iter_mut().enumerate().map(move |(idx, v)| {
            let x = (idx % width) as u32;
            let y = (idx / width) as u32;
            (UVec2::new(x, y), v)
        })
    }

    /// Create an empty grid filled with `T::default()`.
    pub fn empty(info: MapInfo) -> Self
    where
        T: Default + Clone,
    {
        let expected_len = (info.width as usize) * (info.height as usize);
        Self {
            info,
            data: vec![T::default(); expected_len],
            fill_value: T::default(),
        }
    }

    /// Create a grid where every cell is initialised to `fill_value`.
    ///
    /// The same value is used by [`clear`](Self::clear),
    /// [`update_origin`](Self::update_origin), [`resize_map`](Self::resize_map),
    /// and [`reset_map`](Self::reset_map) when new or reset cells need a value.
    pub fn filled(info: MapInfo, fill_value: T) -> Self
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

    /// Reset all cells to the grid's fill value.
    pub fn clear(&mut self)
    where
        T: Clone,
    {
        self.data.fill(self.fill_value.clone());
    }

    /// Return the fill value used for new / reset cells.
    pub fn fill_value(&self) -> &T {
        &self.fill_value
    }

    /// Resize the map and update resolution/origin.
    ///
    /// This method clears the data to the fill value.
    pub fn resize_map(&mut self, size: UVec2, resolution: f32, origin: &Vec2)
    where
        T: Clone,
    {
        self.info.width = size.x;
        self.info.height = size.y;
        self.info.resolution = resolution;
        self.info.origin = *origin;
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
    ///
    /// # Arguments
    ///
    /// * `origin` - The desired origin of the grid in world coordinates.
    pub fn update_origin(&mut self, origin: &Vec2)
    where
        T: Clone,
    {
        let resolution = self.info.resolution;
        if resolution <= 0.0 {
            self.info.origin = *origin;
            self.data = vec![self.fill_value.clone(); self.data.len()];
            return;
        }

        let old_origin = self.info.origin;
        let cell_offset = ((*origin - old_origin) / resolution).floor().as_ivec2();
        let grid_size = IVec2::new(self.info.width as i32, self.info.height as i32);

        self.data = self.copy_overlapping_region(grid_size, grid_size, cell_offset);

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
    pub fn update_center(&mut self, center: &Vec2)
    where
        T: Clone,
    {
        let origin = center
            - Vec2::new(
                self.info.width as f32 * self.info.resolution,
                self.info.height as f32 * self.info.resolution,
            ) * 0.5;
        self.update_origin(&origin);
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

    /// Clear all cells along a ray and optionally mark the endpoint.
    ///
    /// Every cell traversed by the ray from `origin` in direction `dir` for
    /// `distance` world units is set to `clear_value`. If `endpoint_value` is
    /// `Some`, the cell at the end of the ray is then set to that value.
    pub fn clear_ray(
        &mut self,
        origin: &Vec2,
        dir: &Vec2,
        distance: f32,
        clear_value: T,
        endpoint_value: Option<T>,
    ) where
        T: Clone,
    {
        if let Some(mut iter) = self.line_value_mut(origin, dir, distance) {
            for cell in &mut iter {
                *cell = clear_value.clone();
            }
        }

        if let Some(endpoint) = endpoint_value {
            let end_world = *origin + *dir * distance;
            if let Some(cell) = self.world_to_map(&end_world) {
                let _ = self.set(&cell, endpoint);
            }
        }
    }

    pub fn polygon<'a>(&'a self, points: &[Vec2]) -> Option<PolygonIterator> {
        PolygonIterator::new(self, points)
    }

    pub fn polygon_value<'a>(&'a self, points: &[Vec2]) -> Option<PolygonValueIterator<'a, T>> {
        PolygonValueIterator::new(self, points)
    }

    pub fn polygon_value_mut<'a>(
        &'a mut self,
        points: &[Vec2],
    ) -> Option<PolygonValueMutIterator<'a, T>> {
        PolygonValueMutIterator::new(self, points)
    }

    /// Get the maximum cost value under a robot footprint at a given pose.
    ///
    /// This is a fundamental operation for path planning, collision checking, and pose validation.
    /// It checks what the highest cost is anywhere under the robot's footprint when placed
    /// at the specified pose.
    ///
    /// # Arguments
    ///
    /// * `position` - Robot position (x, y) in world coordinates (meters)
    /// * `yaw` - Robot orientation in radians
    /// * `footprint` - Footprint polygon vertices relative to robot center (meters)
    ///
    /// # Returns
    ///
    /// The maximum cost value found under the footprint. Returns `T::default()` if the
    /// footprint is completely outside the map bounds.
    ///
    /// # Example
    ///
    /// ```
    /// use costmap::{Grid2d, MapInfo};
    /// use costmap::types::COST_LETHAL;
    /// use glam::Vec2;
    ///
    /// let info = MapInfo {
    ///     width: 100,
    ///     height: 100,
    ///     ..Default::default()
    /// };
    /// let costmap = Grid2d::<u8>::empty(info);
    ///
    /// // Define a rectangular robot footprint (0.6m x 0.4m)
    /// let footprint = vec![
    ///     Vec2::new(0.3, 0.2),   // front-right
    ///     Vec2::new(0.3, -0.2),  // rear-right
    ///     Vec2::new(-0.3, -0.2), // rear-left
    ///     Vec2::new(-0.3, 0.2),  // front-left
    /// ];
    ///
    /// // Check cost at a specific pose
    /// let position = Vec2::new(5.0, 5.0);
    /// let yaw = 0.0;
    /// let max_cost = costmap.footprint_cost(&position, yaw, &footprint);
    ///
    /// // Use in planning: reject poses with high cost
    /// if max_cost < COST_LETHAL {
    ///     // This pose is safe for the robot
    /// }
    /// ```
    pub fn footprint_cost(&self, position: &Vec2, yaw: f32, footprint: &[Vec2]) -> T
    where
        T: Default + Ord + Copy,
    {
        if footprint.is_empty() {
            return T::default();
        }

        // Transform footprint points from robot frame to world frame
        let cos_yaw = yaw.cos();
        let sin_yaw = yaw.sin();

        let transformed_footprint: Vec<Vec2> = footprint
            .iter()
            .map(|p| {
                // Rotate point by yaw
                let rotated =
                    Vec2::new(p.x * cos_yaw - p.y * sin_yaw, p.x * sin_yaw + p.y * cos_yaw);
                // Translate to world position
                *position + rotated
            })
            .collect();

        // Iterate over all cells inside the transformed footprint polygon
        // and find the maximum cost
        let mut max_cost = T::default();

        if let Some(iter) = self.polygon_value(&transformed_footprint) {
            for cost in iter {
                if *cost > max_cost {
                    max_cost = *cost;
                }
            }
        }

        max_cost
    }

    fn copy_overlapping_region(
        &self,
        old_size: IVec2,
        new_size: IVec2,
        cell_offset: IVec2,
    ) -> Vec<T>
    where
        T: Clone,
    {
        let overlap_min = cell_offset.clamp(IVec2::ZERO, old_size);
        let overlap_max = (cell_offset + new_size).clamp(IVec2::ZERO, old_size);
        let overlap_size = (overlap_max - overlap_min).max(IVec2::ZERO);

        let cell_size_x = overlap_size.x as usize;
        let cell_size_y = overlap_size.y as usize;

        let mut local = vec![self.fill_value.clone(); cell_size_x * cell_size_y];
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

        let mut new_data = vec![self.fill_value.clone(); new_size.x as usize * new_size.y as usize];
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

    #[inline]
    fn index(&self, pos: &UVec2) -> usize {
        (pos.y as usize) * (self.info.width as usize) + (pos.x as usize)
    }
}

impl<T> Index<UVec2> for Grid2d<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: UVec2) -> &Self::Output {
        if index.x >= self.info.width || index.y >= self.info.height {
            panic!(
                "grid index ({}, {}) out of bounds for grid {}x{}",
                index.x, index.y, self.info.width, self.info.height
            );
        }
        unsafe { self.get_unchecked(&index) }
    }
}

impl<T> IndexMut<UVec2> for Grid2d<T> {
    #[inline]
    fn index_mut(&mut self, index: UVec2) -> &mut Self::Output {
        if index.x >= self.info.width || index.y >= self.info.height {
            panic!(
                "grid index ({}, {}) out of bounds for grid {}x{}",
                index.x, index.y, self.info.width, self.info.height
            );
        }
        unsafe { self.get_unchecked_mut(&index) }
    }
}

impl<T> Grid for Grid2d<T> {
    type Cell = T;

    #[inline]
    fn info(&self) -> &MapInfo {
        self.info()
    }

    #[inline]
    fn get(&self, pos: &UVec3) -> Option<&Self::Cell> {
        Grid2d::get(self, &pos.truncate())
    }

    #[inline]
    fn set(&mut self, pos: &UVec3, value: Self::Cell) -> Result<(), VoxelError> {
        Grid2d::set(self, &pos.truncate(), value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world_to_map_to_world(grid: &Grid2d<i8>, pos: Vec2) -> Vec2 {
        let map_pos = grid.world_to_map_continuous(&pos).unwrap();
        grid.map_to_world_continuous(&map_pos)
    }

    #[test]
    fn test_world_to_map_to_world() {
        let grid = Grid2d::<i8>::new(
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
        let mut grid = Grid2d::<i8>::new(
            MapInfo {
                width: 4,
                height: 4,
                resolution: 1.0,
                ..Default::default()
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(&UVec2::new(1, 1), 7).unwrap();
        grid.update_origin(&Vec2::new(1.0, 0.0));

        assert_eq!(grid.get(&UVec2::new(0, 1)), Some(&7));
        assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn test_resize_map_clears_data() {
        let mut grid = Grid2d::<i8>::new(
            MapInfo {
                width: 4,
                height: 4,
                resolution: 1.0,
                ..Default::default()
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(&UVec2::new(1, 1), 9).unwrap();
        grid.resize_map(UVec2::new(6, 6), 1.0, &Vec2::new(1.0, 0.0));

        assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn iter_cells_row_major_order() {
        let grid = Grid2d::<u8>::new(
            MapInfo {
                width: 3,
                height: 2,
                resolution: 1.0,
                ..Default::default()
            },
            vec![10, 11, 12, 20, 21, 22],
        )
        .unwrap();

        let cells: Vec<(UVec2, u8)> = grid.iter_cells().map(|(p, v)| (p, *v)).collect();
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
        let mut grid = Grid2d::<u8>::new(
            MapInfo {
                width: 2,
                height: 2,
                ..Default::default()
            },
            vec![0, 1, 2, 3],
        )
        .unwrap();

        for (pos, v) in grid.iter_cells_mut() {
            *v = (pos.x + 10 * pos.y) as u8;
        }

        assert_eq!(grid.get(&UVec2::new(0, 0)), Some(&0));
        assert_eq!(grid.get(&UVec2::new(1, 0)), Some(&1));
        assert_eq!(grid.get(&UVec2::new(0, 1)), Some(&10));
        assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&11));
    }
}
