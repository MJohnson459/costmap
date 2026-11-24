//! 2D occupancy and costmap grid.
//!
//! ## Coordinate frames
//!
//! - **Cells**: positions (IVec2) on the grid's [`Lattice`], counted from the
//!   lattice's anchor. A grid holds the rectangle [`DenseGrid::cells`] of them, and
//!   a cell names the same place in every grid on that lattice. Extents are UVec2.
//! - **World coordinates**: Physical coordinates in meters.
//!   [`DenseGrid::lattice`] converts between the two.

use glam::{DVec2, IVec2, UVec2};
use std::{
    ops::{Index, IndexMut},
    slice::{Iter, IterMut},
};

use crate::types::{Bounds, CellRegion, CostmapError, Lattice};

/// A bounded grid that stores every cell of a rectangle of a [`Lattice`], row-major
/// from the rectangle's first cell. [`TiledGrid`](crate::TiledGrid) is the sparse
/// counterpart for maps too large to hold whole.
///
/// Moving the grid with [`Self::move_to`] changes which cells it holds and
/// leaves the lattice alone, so a cell keeps its name.
///
/// ```
/// use costmap::{Costmap, Lattice, types::{COST_FREE, CellRegion}};
/// use glam::{DVec2, UVec2};
///
/// // 200 x 200 cells of 0.05 m with the corner of cell (0, 0) at (-5, -5).
/// let lattice = Lattice::new(DVec2::new(-5.0, -5.0), 0.05);
/// let map = Costmap::filled(lattice, CellRegion::from_size(UVec2::splat(200)), COST_FREE);
/// assert_eq!(map.world_bounds().max, DVec2::new(5.0, 5.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct DenseGrid<T> {
    lattice: Lattice,
    /// First cell held.
    min: IVec2,
    size: UVec2,
    data: Vec<T>,
    fill_value: T,
}

impl<T> DenseGrid<T> {
    /// Create a grid holding `cells` of `lattice`, every cell set to
    /// [`Default::default`].
    pub fn new(lattice: Lattice, cells: CellRegion) -> Self
    where
        T: Default + Clone,
    {
        Self::filled(lattice, cells, T::default())
    }

    /// Create a grid holding `cells` of `lattice`, every cell set to `value`, which
    /// also becomes the fill value (see [`Self::with_fill`]).
    pub fn filled(lattice: Lattice, cells: CellRegion, value: T) -> Self
    where
        T: Clone,
    {
        Self::filled_cells(lattice, cells.min, cells.size(), value)
    }

    /// Like [`Self::filled`], holding `size` cells from `min`. Cells beyond the `i32`
    /// range are stored but have no name; `cells()` saturates.
    pub(crate) fn filled_cells(lattice: Lattice, min: IVec2, size: UVec2, value: T) -> Self
    where
        T: Clone,
    {
        Self {
            data: vec![value.clone(); size.x as usize * size.y as usize],
            lattice,
            min,
            size,
            fill_value: value,
        }
    }

    /// Create a grid holding `cells` of `lattice` by calling `f(cell)` for each cell
    /// in row-major order.
    pub fn from_fn(lattice: Lattice, cells: CellRegion, mut f: impl FnMut(IVec2) -> T) -> Self
    where
        T: Default,
    {
        let size = cells.size();
        let mut data = Vec::with_capacity(size.x as usize * size.y as usize);
        for y in 0..size.y as i32 {
            for x in 0..size.x as i32 {
                data.push(f(cells.min + IVec2::new(x, y)));
            }
        }
        Self {
            lattice,
            min: cells.min,
            size,
            data,
            fill_value: T::default(),
        }
    }

    /// Create a grid holding `cells` of `lattice` from values in row-major order.
    /// Fails if `data` does not hold one value per cell.
    pub fn from_vec(lattice: Lattice, cells: CellRegion, data: Vec<T>) -> Result<Self, CostmapError>
    where
        T: Default,
    {
        let size = cells.size();
        let expected_len = size.x as usize * size.y as usize;
        if data.len() != expected_len {
            return Err(CostmapError::InvalidMetadata(format!(
                "data length {} does not match map size {}",
                data.len(),
                expected_len
            )));
        }
        Ok(Self {
            lattice,
            min: cells.min,
            size,
            data,
            fill_value: T::default(),
        })
    }

    /// Set the fill value: the value [`Self::clear`], [`Self::move_to`],
    /// [`Self::reset_to`] and [`Self::clear_region`] give to new or reset cells.
    /// Constructors other than [`Self::filled`] start with [`Default::default`].
    pub fn with_fill(mut self, fill_value: T) -> Self {
        self.fill_value = fill_value;
        self
    }

    #[deprecated(since = "0.3.0", note = "use `size().x`")]
    #[inline]
    pub fn width(&self) -> u32 {
        self.size.x
    }

    #[deprecated(since = "0.3.0", note = "use `size().y`")]
    #[inline]
    pub fn height(&self) -> u32 {
        self.size.y
    }

    /// Width and height in cells.
    #[inline]
    pub fn size(&self) -> UVec2 {
        self.size
    }

    /// The cells this grid holds.
    #[inline]
    pub fn cells(&self) -> CellRegion {
        CellRegion::sized(self.min, self.size)
    }

    /// The world rectangle the grid covers: from the lower-left corner of its first
    /// cell to the upper-right corner of its last.
    #[inline]
    pub fn world_bounds(&self) -> Bounds {
        let cells = self.cells();
        Bounds {
            min: self.lattice.cell_corner(cells.min),
            max: self.lattice.cell_corner(cells.max),
        }
    }

    /// Whether this grid holds the same cells of the same lattice as `other`.
    pub fn layout_matches<U>(&self, other: &DenseGrid<U>) -> bool {
        self.lattice == other.lattice && self.min == other.min && self.size() == other.size()
    }

    /// Returns a reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos` is inside the grid: see [`Self::contains`].
    #[inline]
    pub unsafe fn get_unchecked(&self, pos: IVec2) -> &T {
        self.at_held(self.offset_of(pos))
    }

    /// Returns a mutable reference to the cell at `pos` without bounds checking.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pos` is inside the grid: see [`Self::contains`].
    #[inline]
    pub unsafe fn get_unchecked_mut(&mut self, pos: IVec2) -> &mut T {
        self.at_held_mut(self.offset_of(pos))
    }

    /// Whether `pos` is a cell of this grid.
    #[inline]
    pub fn contains(&self, pos: IVec2) -> bool {
        self.holds_offset(self.offset_of(pos))
    }

    #[inline]
    pub fn get(&self, pos: IVec2) -> Option<&T> {
        self.at(self.offset_of(pos))
    }

    #[inline]
    pub fn get_mut(&mut self, pos: IVec2) -> Option<&mut T> {
        self.at_mut(self.offset_of(pos))
    }

    #[inline]
    pub fn set(&mut self, pos: IVec2, value: T) -> Result<(), CostmapError> {
        match self.get_mut(pos) {
            Some(cell) => {
                *cell = value;
                Ok(())
            }
            None => Err(CostmapError::OutOfBounds(format!(
                "cell ({}, {}) is outside the grid's cells {:?}",
                pos.x,
                pos.y,
                self.cells()
            ))),
        }
    }

    /// Where the cells sit in the world. [`Lattice::cell_at`] gives a cell for any
    /// position; [`Self::get`] or [`Self::contains`] tells whether it is in the grid.
    #[inline]
    pub fn lattice(&self) -> Lattice {
        self.lattice
    }

    /// Array position of `cell`: its distance from the first cell held. The public
    /// methods take cells; code that walks the array works in offsets and converts
    /// where it meets the caller.
    #[inline]
    pub(crate) fn offset_of(&self, cell: IVec2) -> IVec2 {
        cell.wrapping_sub(self.min)
    }

    /// The cell at array position `offset`.
    #[inline]
    pub(crate) fn cell_of(&self, offset: IVec2) -> IVec2 {
        offset.wrapping_add(self.min)
    }

    #[inline]
    fn holds_offset(&self, offset: IVec2) -> bool {
        // A negative coordinate is a large unsigned one, so one comparison per axis.
        (offset.x as u32) < self.size.x && (offset.y as u32) < self.size.y
    }

    /// The value at array position `offset`, if the grid holds it.
    #[inline]
    pub(crate) fn at(&self, offset: IVec2) -> Option<&T> {
        if !self.holds_offset(offset) {
            return None;
        }
        Some(&self.data[self.index(offset)])
    }

    #[inline]
    pub(crate) fn at_mut(&mut self, offset: IVec2) -> Option<&mut T> {
        if !self.holds_offset(offset) {
            return None;
        }
        let idx = self.index(offset);
        Some(&mut self.data[idx])
    }

    /// The value at array position `offset`, which the caller knows the grid holds.
    /// An offset outside the grid reads another cell or panics.
    #[inline]
    pub(crate) fn at_held(&self, offset: IVec2) -> &T {
        &self.data[self.index(offset)]
    }

    #[inline]
    pub(crate) fn at_held_mut(&mut self, offset: IVec2) -> &mut T {
        let idx = self.index(offset);
        &mut self.data[idx]
    }

    /// The part of `region` this grid holds, as array offsets, or `None` if it holds
    /// none of it.
    pub(crate) fn offsets_in(&self, region: CellRegion) -> Option<CellRegion> {
        let size = self.size().as_ivec2();
        let min = region.min.saturating_sub(self.min).clamp(IVec2::ZERO, size);
        let max = region.max.saturating_sub(self.min).clamp(IVec2::ZERO, size);
        (min.x < max.x && min.y < max.y).then_some(CellRegion::new(min, max))
    }

    /// A grid over the same cells as this one, holding `data` in row-major order.
    pub(crate) fn with_cells_of<U>(other: &DenseGrid<U>, data: Vec<T>, fill_value: T) -> Self {
        debug_assert_eq!(data.len(), other.data.len());
        Self {
            lattice: other.lattice,
            min: other.min,
            size: other.size,
            data,
            fill_value,
        }
    }

    /// `position` in cell units from the lower-left corner of the first cell held,
    /// not rounded: the continuous form of [`Self::offset_of`].
    #[inline]
    pub(crate) fn to_offsets(&self, position: DVec2) -> DVec2 {
        self.lattice.to_cells(position) - self.min.as_dvec2()
    }

    /// Whether `offsets` (see [`Self::to_offsets`]) lies in `[0, size)` on both axes.
    /// NaN coordinates pass.
    #[inline]
    pub(crate) fn spans(&self, offsets: DVec2) -> bool {
        let size = self.size().as_dvec2();
        !(offsets.x < 0.0 || offsets.y < 0.0 || offsets.x >= size.x || offsets.y >= size.y)
    }

    /// All cells as a slice in row-major order, starting at the first cell of
    /// [`Self::cells`].
    ///
    /// Row `y` of the rectangle occupies `[y * width, (y + 1) * width)`.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// All cells as a mutable slice in row-major order, starting at the first cell
    /// of [`Self::cells`].
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
    /// Order: the first row of [`Self::cells`] left to right, then the next row up.
    pub fn indexed_iter(&self) -> impl Iterator<Item = (IVec2, &T)> {
        let width = self.size.x as usize;
        let min = self.min;
        self.as_slice().iter().enumerate().map(move |(idx, v)| {
            let x = (idx % width) as i32;
            let y = (idx / width) as i32;
            (min.wrapping_add(IVec2::new(x, y)), v)
        })
    }

    /// Iterate over all cells in row-major order, yielding `(cell, value)` mutably.
    ///
    /// Order: the first row of [`Self::cells`] left to right, then the next row up.
    pub fn indexed_iter_mut(&mut self) -> impl Iterator<Item = (IVec2, &mut T)> {
        let width = self.size.x as usize;
        let min = self.min;
        self.as_mut_slice()
            .iter_mut()
            .enumerate()
            .map(move |(idx, v)| {
                let x = (idx % width) as i32;
                let y = (idx / width) as i32;
                (min.wrapping_add(IVec2::new(x, y)), v)
            })
    }

    /// Reset all cells to the grid's fill value.
    pub fn clear(&mut self)
    where
        T: Clone,
    {
        self.data.fill(self.fill_value.clone());
    }

    /// Replace the grid with the `cells` of `lattice`, every cell set to the fill
    /// value.
    pub fn reset_to(&mut self, lattice: Lattice, cells: CellRegion)
    where
        T: Clone,
    {
        *self = Self::filled(lattice, cells, self.fill_value.clone());
    }

    /// Replace the grid with `size` cells from (0, 0) of a new lattice anchored at
    /// `origin`, every cell set to the fill value.
    #[deprecated(
        since = "0.3.0",
        note = "use `reset_to(Lattice::new(origin, resolution), CellRegion::from_size(size))`"
    )]
    pub fn resize_map(&mut self, size: UVec2, resolution: f64, origin: DVec2)
    where
        T: Clone,
    {
        self.reset_to(
            Lattice::new(origin, resolution),
            CellRegion::from_size(size),
        );
    }

    /// Set the cells of `region` that the grid holds to the fill value.
    pub fn clear_region(&mut self, region: CellRegion)
    where
        T: Clone,
    {
        let Some(region) = self.offsets_in(region) else {
            return;
        };
        let stride = self.size.x as usize;
        let columns = region.min.x as usize..region.max.x as usize;
        for y in region.min.y as usize..region.max.y as usize {
            self.data[y * stride..][columns.clone()].fill(self.fill_value.clone());
        }
    }

    /// Reset the cells of `[lower, upper)` that lie in the grid to the fill value.
    #[deprecated(
        since = "0.3.0",
        note = "use `clear_region(CellRegion::new(lower, upper))`"
    )]
    pub fn reset_map(&mut self, lower: IVec2, upper: IVec2)
    where
        T: Clone,
    {
        self.clear_region(CellRegion::new(lower, upper));
    }

    /// Move the grid so that its first cell is the one containing `position`. Cells
    /// held before and after keep their values; cells that come into the grid take
    /// the fill value. The lattice does not change.
    pub fn move_to(&mut self, position: DVec2)
    where
        T: Clone,
    {
        let grid_size = self.size().as_ivec2();
        // Keeps the last cell held within the `i32` range.
        let min = self.lattice.cell_at(position).min(IVec2::MAX - grid_size);
        let cell_offset = min.saturating_sub(self.min);
        let grid_size = self.size.as_usizevec2();

        let overlap_min = cell_offset.clamp(IVec2::ZERO, grid_size.as_ivec2());
        let overlap_max = cell_offset
            .saturating_add(grid_size.as_ivec2())
            .clamp(IVec2::ZERO, grid_size.as_ivec2());
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
        self.min = min;
    }

    /// Move the grid so that `position` lies at the middle of the rectangle it holds,
    /// as [`Self::move_to`] does for its first cell.
    pub fn center_on(&mut self, position: DVec2)
    where
        T: Clone,
    {
        let first = position - self.size().as_dvec2() * self.lattice.resolution * 0.5;
        self.move_to(first);
    }

    /// Move the grid so that its first cell is the one containing `origin`.
    #[deprecated(since = "0.3.0", note = "renamed to `move_to`")]
    pub fn update_origin(&mut self, origin: DVec2)
    where
        T: Clone,
    {
        self.move_to(origin);
    }

    /// Move the grid so that it is centred on `center`.
    #[deprecated(since = "0.3.0", note = "renamed to `center_on`")]
    pub fn update_center(&mut self, center: DVec2)
    where
        T: Clone,
    {
        self.center_on(center);
    }

    /// Position in `data` of an in-grid `offset`. Going through `u32` zero-extends;
    /// a cast straight from `i32` would sign-extend.
    #[inline]
    fn index(&self, offset: IVec2) -> usize {
        (offset.y as u32 as usize) * (self.size.x as usize) + (offset.x as u32 as usize)
    }
}

impl<T> Index<IVec2> for DenseGrid<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: IVec2) -> &Self::Output {
        self.get(index).unwrap_or_else(|| {
            panic!(
                "cell ({}, {}) is outside the grid's cells {:?}",
                index.x,
                index.y,
                self.cells()
            )
        })
    }
}

impl<T> IndexMut<IVec2> for DenseGrid<T> {
    #[inline]
    fn index_mut(&mut self, index: IVec2) -> &mut Self::Output {
        self.get_mut(index).unwrap_or_else(|| {
            panic!("cell ({}, {}) is outside the grid", index.x, index.y);
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world_to_map_to_world(grid: &DenseGrid<i8>, pos: DVec2) -> DVec2 {
        let lattice = grid.lattice();
        let map_pos = lattice.to_cells(pos);
        assert!(grid.spans(map_pos));
        lattice.from_cells(map_pos.as_vec2().as_dvec2())
    }

    #[test]
    fn test_world_to_map_to_world() {
        let grid = DenseGrid::<i8>::from_vec(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::new(10, 10)),
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
    fn move_to_moves_the_grid_and_cells_keep_their_names() {
        let mut grid = DenseGrid::<i8>::from_vec(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::new(4, 4)),
            vec![0; 16],
        )
        .unwrap();

        let lattice = grid.lattice();
        grid.set(IVec2::new(1, 1), 7).unwrap();
        grid.move_to(DVec2::new(1.0, 0.0));

        assert_eq!(grid.lattice(), lattice);
        assert_eq!(
            grid.cells(),
            CellRegion::new(IVec2::new(1, 0), IVec2::new(5, 4))
        );
        assert_eq!(grid.world_bounds().min, DVec2::new(1.0, 0.0));
        assert_eq!(grid.get(IVec2::new(1, 1)), Some(&7));
        assert_eq!(grid.get(IVec2::new(0, 1)), None);
        assert_eq!(grid.get(IVec2::new(4, 1)), Some(&0));
        // The cell is now the first of its row.
        assert_eq!(grid.as_slice()[4], 7);
    }

    #[test]
    fn reset_to_clears_data() {
        let mut grid = DenseGrid::<i8>::from_vec(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::new(4, 4)),
            vec![0; 16],
        )
        .unwrap();

        grid.set(IVec2::new(1, 1), 9).unwrap();
        grid.reset_to(
            Lattice::new(DVec2::new(1.0, 0.0), 1.0),
            CellRegion::from_size(UVec2::new(6, 6)),
        );

        assert_eq!(grid.get(IVec2::new(1, 1)), Some(&0));
    }

    #[test]
    fn iter_cells_row_major_order() {
        let grid = DenseGrid::<u8>::from_vec(
            Lattice::new(DVec2::ZERO, 1.0),
            CellRegion::from_size(UVec2::new(3, 2)),
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
            Lattice::new(DVec2::ZERO, 0.05),
            CellRegion::from_size(UVec2::new(2, 2)),
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
            Lattice::new(DVec2::new(1.0, 2.0), 0.5),
            CellRegion::from_size(UVec2::new(10, 20)),
            vec![0; 200],
        )
        .unwrap();

        let grid_f32 = DenseGrid::<f32>::from_vec(
            Lattice::new(DVec2::new(1.0, 2.0), 0.5),
            CellRegion::from_size(UVec2::new(10, 20)),
            vec![0.0; 200],
        )
        .unwrap();

        assert!(grid_u8.layout_matches(&grid_f32));
        assert!(grid_f32.layout_matches(&grid_u8));
    }

    #[test]
    fn test_layout_matches_different_width() {
        let grid_u8 = DenseGrid::<u8>::from_vec(
            Lattice::new(DVec2::ZERO, 0.5),
            CellRegion::from_size(UVec2::new(10, 20)),
            vec![0; 200],
        )
        .unwrap();

        let grid_f32 = DenseGrid::<f32>::from_vec(
            Lattice::new(DVec2::ZERO, 0.5),
            CellRegion::from_size(UVec2::new(15, 20)),
            vec![0.0; 300],
        )
        .unwrap();

        assert!(!grid_u8.layout_matches(&grid_f32));
    }

    #[test]
    fn test_layout_matches_different_resolution() {
        let grid_u8 = DenseGrid::<u8>::from_vec(
            Lattice::new(DVec2::ZERO, 0.5),
            CellRegion::from_size(UVec2::new(10, 20)),
            vec![0; 200],
        )
        .unwrap();

        let grid_f32 = DenseGrid::<f32>::from_vec(
            Lattice::new(DVec2::ZERO, 0.1),
            CellRegion::from_size(UVec2::new(10, 20)),
            vec![0.0; 200],
        )
        .unwrap();

        assert!(!grid_u8.layout_matches(&grid_f32));
    }

    #[test]
    fn rolling_origin_stays_on_the_cell_lattice_after_a_long_drive() {
        let lattice = Lattice::new(DVec2::new(500_000.0, 6_200_000.0), 0.05);
        let mut grid = DenseGrid::<u8>::new(lattice, CellRegion::from_size(UVec2::splat(200)));
        let start = grid.world_bounds().min;
        let step = DVec2::new(0.137, 0.061);
        let bounds = grid.world_bounds();
        let mut centre = (bounds.min + bounds.max) * 0.5;
        for _ in 0..10_000 {
            centre += step;
            grid.center_on(centre);
        }
        let travelled = (grid.world_bounds().min - start).length();
        assert!(travelled > 1_400.0, "{travelled}");
        let cells = (grid.world_bounds().min - start) / lattice.resolution;
        let off_lattice = (cells - cells.round()).abs().max_element();
        assert!(off_lattice < 1e-4, "{off_lattice} cells off the lattice");
    }
}
