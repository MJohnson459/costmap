//! 2D occupancy and costmap grid.
//!
//! ## Coordinate frames
//!
//! - **Map coordinates**: Cell positions (IVec2) or continuous (Vec2) in the grid.
//!   Extents are UVec2.
//! - **World coordinates**: Physical coordinates in meters. The origin in [`MapInfo`]
//!   gives the world coordinate of the lower-left corner of cell (0, 0).

use glam::{DVec2, IVec2, USizeVec2, UVec2, Vec2};
use std::{
    ops::{Index, IndexMut},
    slice::{Iter, IterMut},
};

use crate::types::{MapInfo, VoxelError};

/// A bounded grid that stores every cell, row-major from cell (0, 0), placed in the
/// world by a [`MapInfo`]. [`TiledGrid`](crate::TiledGrid) is the sparse counterpart
/// for maps too large to hold whole.
#[derive(Debug, Clone, PartialEq)]
pub struct DenseGrid<T> {
    info: MapInfo,
    data: Vec<T>,
    fill_value: T,
}

impl<T> DenseGrid<T> {
    /// Create a grid with every cell set to [`Default::default`].
    pub fn new(info: MapInfo) -> Self
    where
        T: Default + Clone,
    {
        Self::filled(info, T::default())
    }

    /// Create a grid with every cell set to `value`, which also becomes the fill
    /// value (see [`Self::with_fill`]).
    pub fn filled(info: MapInfo, value: T) -> Self
    where
        T: Clone,
    {
        let len = (info.width as usize) * (info.height as usize);
        Self {
            data: vec![value.clone(); len],
            info,
            fill_value: value,
        }
    }

    /// Create a grid by calling `f(cell)` for each cell in row-major order.
    pub fn from_fn(info: MapInfo, mut f: impl FnMut(IVec2) -> T) -> Self
    where
        T: Default,
    {
        let mut data = Vec::with_capacity((info.width as usize) * (info.height as usize));
        for y in 0..info.height as i32 {
            for x in 0..info.width as i32 {
                data.push(f(IVec2::new(x, y)));
            }
        }
        Self {
            info,
            data,
            fill_value: T::default(),
        }
    }

    /// Create a grid from cells in row-major order. Fails if `data` does not hold
    /// `width * height` cells.
    pub fn from_vec(info: MapInfo, data: Vec<T>) -> Result<Self, VoxelError>
    where
        T: Default,
    {
        Self::from_parts(info, data, T::default())
    }

    /// Set the fill value: the value [`Self::clear`], [`Self::update_origin`],
    /// [`Self::resize_map`] and [`Self::reset_map`] give to new or reset cells.
    /// Constructors other than [`Self::filled`] start with [`Default::default`].
    pub fn with_fill(mut self, fill_value: T) -> Self {
        self.fill_value = fill_value;
        self
    }

    fn from_parts(info: MapInfo, data: Vec<T>, fill_value: T) -> Result<Self, VoxelError> {
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

    #[deprecated(since = "0.3.0", note = "renamed to `from_vec`")]
    pub fn init(info: MapInfo, data: Vec<T>) -> Result<Self, VoxelError>
    where
        T: Default,
    {
        Self::from_vec(info, data)
    }

    #[deprecated(
        since = "0.3.0",
        note = "use `from_vec(info, data)?.with_fill(fill_value)`"
    )]
    pub fn init_with_value(info: MapInfo, data: Vec<T>, fill_value: T) -> Result<Self, VoxelError> {
        Self::from_parts(info, data, fill_value)
    }

    #[deprecated(since = "0.3.0", note = "renamed to `filled`")]
    pub fn new_with_value(info: MapInfo, fill_value: T) -> Self
    where
        T: Clone,
    {
        Self::filled(info, fill_value)
    }

    #[deprecated(since = "0.3.0", note = "renamed to `from_fn`")]
    pub fn new_with(info: MapInfo, mut f: impl FnMut(u32, u32) -> T) -> Self
    where
        T: Default,
    {
        Self::from_fn(info, |cell| f(cell.x as u32, cell.y as u32))
    }

    pub fn info(&self) -> &MapInfo {
        &self.info
    }

    #[deprecated(since = "0.3.0", note = "use `size().x`")]
    #[inline]
    pub fn width(&self) -> u32 {
        self.info.width
    }

    #[deprecated(since = "0.3.0", note = "use `size().y`")]
    #[inline]
    pub fn height(&self) -> u32 {
        self.info.height
    }

    /// Width and height in cells.
    #[inline]
    pub fn size(&self) -> UVec2 {
        UVec2::new(self.info.width, self.info.height)
    }

    /// Check if this grid has the same layout (width, height, resolution, origin) as another grid.
    ///
    /// This is useful for validating that two grids can be safely merged or compared.
    pub fn layout_matches<U>(&self, other: &DenseGrid<U>) -> bool {
        self.info() == other.info()
    }

    /// Returns a reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos` is inside the grid: see [`Self::contains`].
    #[inline]
    pub unsafe fn get_unchecked(&self, pos: IVec2) -> &T {
        &self.data[self.index(pos)]
    }

    /// Returns a mutable reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos` is inside the grid: see [`Self::contains`].
    #[inline]
    pub unsafe fn get_unchecked_mut(&mut self, pos: IVec2) -> &mut T {
        let idx = self.index(pos);
        &mut self.data[idx]
    }

    /// Whether `pos` is a cell of this grid.
    #[inline]
    pub fn contains(&self, pos: IVec2) -> bool {
        // A negative coordinate is a large unsigned one, so one comparison per axis.
        (pos.x as u32) < self.info.width && (pos.y as u32) < self.info.height
    }

    #[inline]
    pub fn get(&self, pos: IVec2) -> Option<&T> {
        if !self.contains(pos) {
            return None;
        }
        Some(unsafe { self.get_unchecked(pos) })
    }

    #[inline]
    pub fn get_mut(&mut self, pos: IVec2) -> Option<&mut T> {
        if !self.contains(pos) {
            return None;
        }
        Some(unsafe { self.get_unchecked_mut(pos) })
    }

    #[inline]
    pub fn set(&mut self, pos: IVec2, value: T) -> Result<(), VoxelError> {
        if !self.contains(pos) {
            return Err(VoxelError::OutOfBounds(format!(
                "cell ({}, {}) out of bounds for map {}x{}",
                pos.x, pos.y, self.info.width, self.info.height
            )));
        }
        *unsafe { self.get_unchecked_mut(pos) } = value;
        Ok(())
    }

    /// Map (cell) → world. Returns the cell center.
    #[inline]
    pub fn map_to_world(&self, cell: IVec2) -> DVec2 {
        self.info.origin + (cell.as_dvec2() + DVec2::splat(0.5)) * self.info.resolution
    }

    /// Map (continuous) → world. For integer coords, returns the cell's lower-left corner.
    #[inline]
    pub fn map_to_world_continuous(&self, pos: Vec2) -> DVec2 {
        self.info.origin + pos.as_dvec2() * self.info.resolution
    }

    /// World → map (cell). Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_map(&self, pos: DVec2) -> Option<IVec2> {
        let mp = self.world_to_map_continuous(pos)?;
        Some(IVec2::new(mp.x as i32, mp.y as i32))
    }

    /// World → map (continuous). Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_map_continuous(&self, pos: DVec2) -> Option<Vec2> {
        let cell = (pos - self.info.origin) / self.info.resolution;
        let size = self.size().as_dvec2();
        if cell.x < 0.0 || cell.y < 0.0 || cell.x >= size.x || cell.y >= size.y {
            return None;
        }
        Some(cell.as_vec2())
    }

    /// All cells as a slice in row-major order, starting at cell (0, 0).
    ///
    /// Row `y` occupies `[y * width, (y + 1) * width)`.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// All cells as a mutable slice in row-major order, starting at cell (0, 0).
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data
    }

    /// Iterate over all cells in row-major order.
    #[deprecated(since = "0.3.0", note = "use `as_slice().iter()`")]
    pub fn iter(&self) -> Iter<'_, T> {
        self.data.iter()
    }

    /// Iterate over all cells in row-major order, yielding mutably.
    #[deprecated(since = "0.3.0", note = "use `as_mut_slice().iter_mut()`")]
    pub fn iter_mut(&mut self) -> IterMut<'_, T> {
        self.data.iter_mut()
    }

    /// Iterate over all cells in row-major order, yielding `(cell, value)`.
    ///
    /// Order: `(0,0), (1,0), ... (width-1,0), (0,1), ...`
    pub fn indexed_iter(&self) -> impl Iterator<Item = (IVec2, &T)> {
        let width = self.info.width as usize;
        self.as_slice().iter().enumerate().map(move |(idx, v)| {
            let x = (idx % width) as i32;
            let y = (idx / width) as i32;
            (IVec2::new(x, y), v)
        })
    }

    /// Iterate over all cells in row-major order, yielding `(cell, value)` mutably.
    ///
    /// Order: `(0,0), (1,0), ... (width-1,0), (0,1), ...`
    pub fn indexed_iter_mut(&mut self) -> impl Iterator<Item = (IVec2, &mut T)> {
        let width = self.info.width as usize;
        self.as_mut_slice()
            .iter_mut()
            .enumerate()
            .map(move |(idx, v)| {
                let x = (idx % width) as i32;
                let y = (idx / width) as i32;
                (IVec2::new(x, y), v)
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
    pub fn resize_map(&mut self, size: UVec2, resolution: f64, origin: DVec2)
    where
        T: Clone,
    {
        self.info.width = size.x;
        self.info.height = size.y;
        self.info.resolution = resolution;
        self.info.origin = origin;
        self.data = vec![self.fill_value.clone(); size.x as usize * size.y as usize];
    }

    /// Reset the cells of `[lower, upper)` that lie in the grid to the fill value.
    pub fn reset_map(&mut self, lower: IVec2, upper: IVec2)
    where
        T: Clone,
    {
        let size = self.size().as_ivec2();
        let width = self.info.width;
        let start_x = lower.x.clamp(0, size.x) as u32;
        let end_x = upper.x.clamp(0, size.x) as u32;
        let start_y = lower.y.clamp(0, size.y) as u32;
        let end_y = upper.y.clamp(0, size.y) as u32;

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
    pub fn update_origin(&mut self, origin: DVec2)
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

        self.info.origin = old_origin + cell_offset.as_dvec2() * resolution;
    }

    /// Updates the origin based on the desired center of the map.
    ///
    /// # Arguments
    ///
    /// * `center` - The desired center of the map in world coordinates.
    pub fn update_center(&mut self, center: DVec2)
    where
        T: Clone,
    {
        let origin = center - self.size().as_dvec2() * self.info.resolution * 0.5;
        self.update_origin(origin);
    }

    /// Offset of an in-grid cell. Going through `u32` zero-extends; a cast
    /// straight from `i32` would sign-extend.
    #[inline]
    fn index(&self, pos: IVec2) -> usize {
        (pos.y as u32 as usize) * (self.info.width as usize) + (pos.x as u32 as usize)
    }
}

impl<T> Index<IVec2> for DenseGrid<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: IVec2) -> &Self::Output {
        self.get(index).unwrap_or_else(|| {
            panic!(
                "grid index ({}, {}) out of bounds for grid {}x{}",
                index.x, index.y, self.info.width, self.info.height
            )
        })
    }
}

impl<T> IndexMut<IVec2> for DenseGrid<T> {
    #[inline]
    fn index_mut(&mut self, index: IVec2) -> &mut Self::Output {
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

    fn world_to_map_to_world(grid: &DenseGrid<i8>, pos: DVec2) -> DVec2 {
        let map_pos = grid.world_to_map_continuous(pos).unwrap();
        grid.map_to_world_continuous(map_pos)
    }

    #[test]
    fn test_world_to_map_to_world() {
        let grid = DenseGrid::<i8>::from_vec(
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
            world_to_map_to_world(&grid, DVec2::new(0.0, 0.0)),
            DVec2::new(0.0, 0.0)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(1.0, 1.0)),
            DVec2::new(1.0, 1.0)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(0.5, 0.5)),
            DVec2::new(0.5, 0.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(1.5, 1.5)),
            DVec2::new(1.5, 1.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(0.5, 1.5)),
            DVec2::new(0.5, 1.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(1.5, 0.5)),
            DVec2::new(1.5, 0.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(0.5, 0.5)),
            DVec2::new(0.5, 0.5)
        );
        assert_eq!(
            world_to_map_to_world(&grid, DVec2::new(1.5, 1.5)),
            DVec2::new(1.5, 1.5)
        );
    }

    #[test]
    fn test_update_origin_shifts_data() {
        let mut grid = DenseGrid::<i8>::from_vec(
            MapInfo {
                width: 4,
                height: 4,
                resolution: 1.0,
                ..Default::default()
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(IVec2::new(1, 1), 7).unwrap();
        grid.update_origin(DVec2::new(1.0, 0.0));

        assert_eq!(grid.get(IVec2::new(0, 1)), Some(&7));
        assert_eq!(grid.get(IVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn test_resize_map_clears_data() {
        let mut grid = DenseGrid::<i8>::from_vec(
            MapInfo {
                width: 4,
                height: 4,
                resolution: 1.0,
                ..Default::default()
            },
            vec![0; 16],
        )
        .unwrap();

        grid.set(IVec2::new(1, 1), 9).unwrap();
        grid.resize_map(UVec2::new(6, 6), 1.0, DVec2::new(1.0, 0.0));

        assert_eq!(grid.get(IVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn iter_cells_row_major_order() {
        let grid = DenseGrid::<u8>::from_vec(
            MapInfo {
                width: 3,
                height: 2,
                resolution: 1.0,
                ..Default::default()
            },
            vec![10, 11, 12, 20, 21, 22],
        )
        .unwrap();

        let cells: Vec<(IVec2, u8)> = grid.indexed_iter().map(|(p, v)| (p, *v)).collect();
        assert_eq!(
            cells,
            vec![
                (IVec2::new(0, 0), 10),
                (IVec2::new(1, 0), 11),
                (IVec2::new(2, 0), 12),
                (IVec2::new(0, 1), 20),
                (IVec2::new(1, 1), 21),
                (IVec2::new(2, 1), 22),
            ]
        );
    }

    #[test]
    fn iter_cells_mut_writes_values() {
        let mut grid = DenseGrid::<u8>::from_vec(
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

        assert_eq!(grid.get(IVec2::new(0, 0)), Some(&0));
        assert_eq!(grid.get(IVec2::new(1, 0)), Some(&1));
        assert_eq!(grid.get(IVec2::new(0, 1)), Some(&10));
        assert_eq!(grid.get(IVec2::new(1, 1)), Some(&11));
    }

    #[test]
    fn test_layout_matches_equal() {
        let grid_u8 = DenseGrid::<u8>::from_vec(
            MapInfo {
                width: 10,
                height: 20,
                resolution: 0.5,
                origin: DVec2::new(1.0, 2.0),
            },
            vec![0; 200],
        )
        .unwrap();

        let grid_f32 = DenseGrid::<f32>::from_vec(
            MapInfo {
                width: 10,
                height: 20,
                resolution: 0.5,
                origin: DVec2::new(1.0, 2.0),
            },
            vec![0.0; 200],
        )
        .unwrap();

        assert!(grid_u8.layout_matches(&grid_f32));
        assert!(grid_f32.layout_matches(&grid_u8));
    }

    #[test]
    fn test_layout_matches_different_width() {
        let grid_u8 = DenseGrid::<u8>::from_vec(
            MapInfo {
                width: 10,
                height: 20,
                resolution: 0.5,
                origin: DVec2::ZERO,
            },
            vec![0; 200],
        )
        .unwrap();

        let grid_f32 = DenseGrid::<f32>::from_vec(
            MapInfo {
                width: 15,
                height: 20,
                resolution: 0.5,
                origin: DVec2::ZERO,
            },
            vec![0.0; 300],
        )
        .unwrap();

        assert!(!grid_u8.layout_matches(&grid_f32));
    }

    #[test]
    fn test_layout_matches_different_resolution() {
        let grid_u8 = DenseGrid::<u8>::from_vec(
            MapInfo {
                width: 10,
                height: 20,
                resolution: 0.5,
                origin: DVec2::ZERO,
            },
            vec![0; 200],
        )
        .unwrap();

        let grid_f32 = DenseGrid::<f32>::from_vec(
            MapInfo {
                width: 10,
                height: 20,
                resolution: 0.1,
                origin: DVec2::ZERO,
            },
            vec![0.0; 200],
        )
        .unwrap();

        assert!(!grid_u8.layout_matches(&grid_f32));
    }

    #[test]
    fn rolling_origin_stays_on_the_cell_lattice_after_a_long_drive() {
        let info = MapInfo {
            width: 200,
            height: 200,
            resolution: 0.05,
            origin: DVec2::new(500_000.0, 6_200_000.0),
        };
        let start = info.origin;
        let mut grid = DenseGrid::<u8>::new(info);
        let step = DVec2::new(0.137, 0.061);
        let mut centre = info.world_center();
        for _ in 0..10_000 {
            centre += step;
            grid.update_center(centre);
        }
        let travelled = (grid.info().origin - start).length();
        assert!(travelled > 1_400.0, "{travelled}");
        let cells = (grid.info().origin - start) / info.resolution;
        let off_lattice = (cells - cells.round()).abs().max_element();
        assert!(off_lattice < 1e-4, "{off_lattice} cells off the lattice");
    }
}
