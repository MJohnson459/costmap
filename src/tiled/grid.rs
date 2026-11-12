use std::collections::HashMap;

use glam::{IVec2, UVec2, Vec2};

use crate::{
    DenseGrid,
    types::{COST_UNKNOWN, MapInfo, VoxelError},
};

/// Largest allowed chunk edge. Keeps `chunk_size * chunk_size` and chunk-local
/// offsets inside `i32`.
const MAX_CHUNK_SIZE: u32 = 1 << 15;

/// Relative tolerance when comparing the resolution of a window to the grid's.
const RESOLUTION_TOLERANCE: f32 = 1e-4;

/// Sparse grid of fixed-size square chunks addressed by signed global cell coordinates.
///
/// Cell `(0, 0)` has its lower-left corner at the local-frame origin; x points
/// east and y north. Chunk `(cx, cy)` holds cells
/// `[cx * chunk_size, (cx + 1) * chunk_size)` on x and likewise on y, stored
/// row-major. Chunks that were never written do not exist and read as the fill value.
#[derive(Debug, Clone)]
pub struct TiledGrid<T> {
    chunks: HashMap<IVec2, Box<[T]>>,
    chunk_size: u32,
    resolution: f32,
    fill: T,
}

/// A contiguous run of cells inside one chunk row, with its position in a dense window.
struct RowRun {
    chunk: IVec2,
    chunk_offset: usize,
    window_offset: usize,
    len: usize,
}

/// Split the cell rectangle `[min, min + size)` into per-chunk row runs.
///
/// The arithmetic is in `i64` because `min + size` can exceed `i32`. Cells
/// beyond the `i32` range have no chunk and are omitted.
fn row_runs(chunk_size: u32, min: IVec2, size: UVec2) -> impl Iterator<Item = RowRun> {
    let cs = i64::from(chunk_size);
    let x0 = i64::from(min.x);
    let y0 = i64::from(min.y);
    let x1 = (x0 + i64::from(size.x)).min(i64::from(i32::MAX) + 1);
    let y1 = if x0 < x1 {
        (y0 + i64::from(size.y)).min(i64::from(i32::MAX) + 1)
    } else {
        y0
    };
    let width = size.x as usize;

    (y0..y1).flat_map(move |y| {
        let chunk_y = y.div_euclid(cs);
        let local_y = y.rem_euclid(cs);
        (x0.div_euclid(cs)..=(x1 - 1).div_euclid(cs)).map(move |chunk_x| {
            let start = x0.max(chunk_x * cs);
            let end = x1.min(chunk_x * cs + cs);
            RowRun {
                chunk: IVec2::new(chunk_x as i32, chunk_y as i32),
                chunk_offset: (local_y * cs + (start - chunk_x * cs)) as usize,
                window_offset: (y - y0) as usize * width + (start - x0) as usize,
                len: (end - start) as usize,
            }
        })
    })
}

/// Maximum of two costs where [`COST_UNKNOWN`] means "no data" and loses to any known cost.
pub(crate) fn max_known(a: u8, b: u8) -> u8 {
    match (a, b) {
        (COST_UNKNOWN, other) | (other, COST_UNKNOWN) => other,
        (a, b) => a.max(b),
    }
}

impl<T: Clone> TiledGrid<T> {
    /// Create an empty grid.
    ///
    /// Fails if `chunk_size` is zero or above 32768, or if `resolution` is not
    /// finite and positive.
    pub fn new(resolution: f32, chunk_size: u32, fill: T) -> Result<Self, VoxelError> {
        if chunk_size == 0 || chunk_size > MAX_CHUNK_SIZE {
            return Err(VoxelError::InvalidArgument(format!(
                "chunk size {chunk_size} must be in 1..={MAX_CHUNK_SIZE}"
            )));
        }
        if !resolution.is_finite() || resolution <= 0.0 {
            return Err(VoxelError::InvalidArgument(format!(
                "resolution {resolution} must be finite and positive"
            )));
        }
        Ok(Self {
            chunks: HashMap::new(),
            chunk_size,
            resolution,
            fill,
        })
    }

    /// Cell edge length in metres.
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Chunk edge length in cells.
    pub fn chunk_size(&self) -> u32 {
        self.chunk_size
    }

    /// Value read from cells in chunks that are not loaded.
    pub fn fill_value(&self) -> &T {
        &self.fill
    }

    /// Value at `cell`. Reads the fill value if the chunk is absent; never allocates.
    pub fn get(&self, cell: IVec2) -> &T {
        match self.chunks.get(&self.chunk_coord(cell)) {
            Some(chunk) => &chunk[self.local_index(cell)],
            None => &self.fill,
        }
    }

    /// Write `value` at `cell`, allocating the chunk (filled with the fill value) if absent.
    pub fn set(&mut self, cell: IVec2, value: T) {
        *self.cell_mut(cell) = value;
    }

    pub(crate) fn cell_mut(&mut self, cell: IVec2) -> &mut T {
        let index = self.local_index(cell);
        &mut self.chunk_mut(self.chunk_coord(cell))[index]
    }

    fn chunk_mut(&mut self, coord: IVec2) -> &mut [T] {
        let len = self.chunk_len();
        let fill = &self.fill;
        self.chunks
            .entry(coord)
            .or_insert_with(|| vec![fill.clone(); len].into_boxed_slice())
    }

    fn chunk_len(&self) -> usize {
        (self.chunk_size as usize).pow(2)
    }

    /// Index of `cell` inside its chunk. `rem_euclid` keeps negative cells in `0..chunk_size`.
    fn local_index(&self, cell: IVec2) -> usize {
        let local = cell.rem_euclid(IVec2::splat(self.chunk_size as i32));
        local.y as usize * self.chunk_size as usize + local.x as usize
    }

    /// Cell containing the local-frame position `pos` (floor, so negative positions round down).
    pub fn world_to_cell(&self, pos: Vec2) -> IVec2 {
        (pos / self.resolution).floor().as_ivec2()
    }

    /// Lower-left corner of `cell` in the local frame.
    pub fn cell_origin(&self, cell: IVec2) -> Vec2 {
        cell.as_vec2() * self.resolution
    }

    /// Centre of `cell` in the local frame.
    pub fn cell_center(&self, cell: IVec2) -> Vec2 {
        self.cell_origin(cell) + Vec2::splat(0.5 * self.resolution)
    }

    /// Coordinate of the chunk containing `cell`.
    ///
    /// Uses floor division, so cell -1 is in chunk -1 rather than chunk 0.
    pub fn chunk_coord(&self, cell: IVec2) -> IVec2 {
        cell.div_euclid(IVec2::splat(self.chunk_size as i32))
    }

    /// Whether `resolution` is within a relative 1e-4 of this grid's.
    pub fn resolution_matches(&self, resolution: f32) -> bool {
        (resolution - self.resolution).abs() <= RESOLUTION_TOLERANCE * self.resolution
    }

    /// Cells of chunk `coord` in row-major order, or `None` if it is not loaded.
    pub fn chunk(&self, coord: IVec2) -> Option<&[T]> {
        self.chunks.get(&coord).map(|chunk| &chunk[..])
    }

    /// Insert or replace chunk `coord`. `data` must hold `chunk_size * chunk_size` cells.
    pub fn insert_chunk(&mut self, coord: IVec2, data: Box<[T]>) -> Result<(), VoxelError> {
        if data.len() != self.chunk_len() {
            return Err(VoxelError::InvalidArgument(format!(
                "chunk data length {} does not match chunk size {size}x{size} = {}",
                data.len(),
                self.chunk_len(),
                size = self.chunk_size
            )));
        }
        self.chunks.insert(coord, data);
        Ok(())
    }

    /// Remove chunk `coord` and return its data.
    pub fn remove_chunk(&mut self, coord: IVec2) -> Option<Box<[T]>> {
        self.chunks.remove(&coord)
    }

    /// Whether chunk `coord` is loaded.
    pub fn is_loaded(&self, coord: IVec2) -> bool {
        self.chunks.contains_key(&coord)
    }

    /// Coordinates of all loaded chunks, in arbitrary order.
    pub fn loaded_chunks(&self) -> impl Iterator<Item = IVec2> + '_ {
        self.chunks.keys().copied()
    }

    /// Cell extent of all loaded chunks as `(min inclusive, max exclusive)`, or `None` if empty.
    ///
    /// Components saturate at the `i32` limits.
    pub fn loaded_cell_bounds(&self) -> Option<(IVec2, IVec2)> {
        let mut coords = self.chunks.keys().copied();
        let first = coords.next()?;
        let (min, max) = coords.fold((first, first), |(min, max), c| (min.min(c), max.max(c)));
        let cs = self.chunk_size as i32;
        let lower = IVec2::new(min.x.saturating_mul(cs), min.y.saturating_mul(cs));
        let upper = IVec2::new(
            max.x.saturating_add(1).saturating_mul(cs),
            max.y.saturating_add(1).saturating_mul(cs),
        );
        Some((lower, upper))
    }

    /// Drop chunks that lie entirely outside the square of cells
    /// `[center - radius_cells, center + radius_cells]`, and return how many were dropped.
    ///
    /// Edits in dropped chunks that were not saved elsewhere are lost.
    pub fn evict_outside(&mut self, center: IVec2, radius_cells: u32) -> usize {
        let cs = i64::from(self.chunk_size);
        let radius = i64::from(radius_cells);
        let overlaps = |chunk: i32, center: i32| {
            let first = i64::from(chunk) * cs;
            first + cs > i64::from(center) - radius && first <= i64::from(center) + radius
        };
        let before = self.chunks.len();
        self.chunks
            .retain(|c, _| overlaps(c.x, center.x) && overlaps(c.y, center.y));
        before - self.chunks.len()
    }

    /// Copy the cells `[min_cell, min_cell + size)` into a dense grid.
    ///
    /// The grid's origin is `cell_origin(min_cell)` and its resolution matches this
    /// grid. Cells in absent chunks, and cells beyond the `i32` cell range, hold the
    /// fill value, which is also the returned grid's fill value.
    pub fn window(&self, min_cell: IVec2, size: UVec2) -> DenseGrid<T> {
        let info = MapInfo {
            width: size.x,
            height: size.y,
            resolution: self.resolution,
            origin: self.cell_origin(min_cell),
        };
        let mut grid = DenseGrid::new_with_value(info, self.fill.clone());
        self.copy_out(min_cell, size, grid.as_mut_slice());
        grid
    }

    /// Like [`window`](Self::window), covering `size` cells with the cell containing `center` in the middle.
    pub fn window_centered(&self, center: Vec2, size: UVec2) -> DenseGrid<T> {
        let min_cell = self.world_to_cell(center) - (size / 2).as_ivec2();
        self.window(min_cell, size)
    }

    /// Fill `target` from the tiles covering its [`MapInfo`].
    ///
    /// The target origin is snapped to the nearest cell boundary. Cells without a
    /// loaded chunk are set to the fill value. Fails if the resolutions differ by
    /// more than a relative 1e-4.
    pub fn read_into(&self, target: &mut DenseGrid<T>) -> Result<(), VoxelError> {
        let (min_cell, size) = self.window_extent(target.info())?;
        self.copy_out(min_cell, size, target.as_mut_slice());
        Ok(())
    }

    /// Store the cells of `source` into the tiles it covers, allocating chunks as needed.
    ///
    /// Inverse of [`read_into`](Self::read_into), with the same origin snapping and
    /// resolution check. Cells beyond the `i32` cell range are ignored.
    pub fn write_window(&mut self, source: &DenseGrid<T>) -> Result<(), VoxelError> {
        let (min_cell, size) = self.window_extent(source.info())?;
        for run in row_runs(self.chunk_size, min_cell, size) {
            let chunk = self.chunk_mut(run.chunk);
            chunk[run.chunk_offset..run.chunk_offset + run.len].clone_from_slice(
                &source.as_slice()[run.window_offset..run.window_offset + run.len],
            );
        }
        Ok(())
    }

    /// First cell and size of the dense window described by `info`.
    fn window_extent(&self, info: &MapInfo) -> Result<(IVec2, UVec2), VoxelError> {
        if !self.resolution_matches(info.resolution) {
            return Err(VoxelError::InvalidArgument(format!(
                "window resolution {} differs from tiled grid resolution {}",
                info.resolution, self.resolution
            )));
        }
        let min_cell = (info.origin / self.resolution).round().as_ivec2();
        Ok((min_cell, UVec2::new(info.width, info.height)))
    }

    /// Copy `[min_cell, min_cell + size)` row by row into `dst`, writing the fill value for absent chunks.
    pub(crate) fn copy_out(&self, min_cell: IVec2, size: UVec2, dst: &mut [T]) {
        for run in row_runs(self.chunk_size, min_cell, size) {
            let out = &mut dst[run.window_offset..run.window_offset + run.len];
            match self.chunks.get(&run.chunk) {
                Some(chunk) => {
                    out.clone_from_slice(&chunk[run.chunk_offset..run.chunk_offset + run.len])
                }
                None => out.fill(self.fill.clone()),
            }
        }
    }
}

impl TiledGrid<u8> {
    /// Lower-resolution copy where each cell is the maximum of a `factor` x `factor` block.
    ///
    /// [`COST_UNKNOWN`] is "no data": an output cell is unknown only if every input
    /// cell in its block is unknown, otherwise it holds the maximum of the known
    /// ones. The result has resolution `resolution * factor`, the same chunk size,
    /// fill value [`COST_UNKNOWN`], and chunks only where some input cell is known.
    pub fn overview_max(&self, factor: u32) -> Result<TiledGrid<u8>, VoxelError> {
        let divisor = i32::try_from(factor)
            .ok()
            .filter(|&f| f >= 1)
            .ok_or_else(|| {
                VoxelError::InvalidArgument(format!("overview factor {factor} must be >= 1"))
            })?;
        let mut out = TiledGrid::new(
            self.resolution * factor as f32,
            self.chunk_size,
            COST_UNKNOWN,
        )?;
        let cs = self.chunk_size as usize;
        for (&coord, chunk) in &self.chunks {
            let first = coord * self.chunk_size as i32;
            for (i, &value) in chunk.iter().enumerate() {
                if value == COST_UNKNOWN {
                    continue;
                }
                let cell = first + IVec2::new((i % cs) as i32, (i / cs) as i32);
                let slot = out.cell_mut(cell.div_euclid(IVec2::splat(divisor)));
                *slot = max_known(*slot, value);
            }
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(chunk_size: u32) -> TiledGrid<u8> {
        TiledGrid::new(0.5, chunk_size, COST_UNKNOWN).unwrap()
    }

    fn pattern(cell: IVec2) -> u8 {
        (cell.x.wrapping_mul(7).wrapping_add(cell.y.wrapping_mul(13)) & 0x7f) as u8
    }

    #[test]
    fn new_rejects_bad_arguments() {
        assert!(TiledGrid::new(0.05, 0, 0u8).is_err());
        assert!(TiledGrid::new(0.05, MAX_CHUNK_SIZE + 1, 0u8).is_err());
        assert!(TiledGrid::new(0.0, 8, 0u8).is_err());
        assert!(TiledGrid::new(-1.0, 8, 0u8).is_err());
        assert!(TiledGrid::new(f32::NAN, 8, 0u8).is_err());
        assert!(TiledGrid::new(f32::INFINITY, 8, 0u8).is_err());
    }

    #[test]
    fn chunk_coord_floors_at_boundaries() {
        let g = grid(16);
        let cases = [(-17, -2), (-16, -1), (-1, -1), (0, 0), (15, 0), (16, 1)];
        for (cell, chunk) in cases {
            assert_eq!(g.chunk_coord(IVec2::new(cell, cell)), IVec2::splat(chunk));
        }
    }

    #[test]
    fn negative_cells_round_trip_through_chunks() {
        let mut g = grid(16);
        for cell in [
            IVec2::new(-1, -1),
            IVec2::new(-16, -16),
            IVec2::new(-17, 3),
            IVec2::new(0, 0),
            IVec2::new(15, 16),
        ] {
            g.set(cell, pattern(cell));
        }
        assert_eq!(*g.get(IVec2::new(-1, -1)), pattern(IVec2::new(-1, -1)));
        assert_eq!(*g.get(IVec2::new(-17, 3)), pattern(IVec2::new(-17, 3)));
        assert_eq!(*g.get(IVec2::new(-16, -16)), pattern(IVec2::new(-16, -16)));
        assert_eq!(*g.get(IVec2::new(-2, -1)), COST_UNKNOWN);
    }

    #[test]
    fn get_on_absent_chunk_returns_fill_without_allocating() {
        let g = grid(16);
        assert_eq!(*g.get(IVec2::new(1000, -1000)), COST_UNKNOWN);
        assert_eq!(g.loaded_chunks().count(), 0);
    }

    #[test]
    fn set_allocates_exactly_one_chunk() {
        let mut g = grid(16);
        g.set(IVec2::new(-1, 20), 9);
        assert_eq!(g.loaded_chunks().collect::<Vec<_>>(), [IVec2::new(-1, 1)]);
        assert_eq!(g.chunk(IVec2::new(-1, 1)).unwrap().len(), 256);
        g.set(IVec2::new(-2, 21), 5);
        assert_eq!(g.loaded_chunks().count(), 1);
    }

    #[test]
    fn insert_chunk_checks_length() {
        let mut g = grid(4);
        assert!(g.insert_chunk(IVec2::ZERO, vec![0; 15].into()).is_err());
        assert!(g.insert_chunk(IVec2::ZERO, vec![3; 16].into()).is_ok());
        assert_eq!(*g.get(IVec2::new(3, 3)), 3);
        assert!(g.is_loaded(IVec2::ZERO));
        assert!(g.remove_chunk(IVec2::ZERO).is_some());
        assert!(!g.is_loaded(IVec2::ZERO));
    }

    #[test]
    fn world_cell_conversions() {
        let g = grid(16);
        assert_eq!(g.world_to_cell(Vec2::new(0.49, -0.01)), IVec2::new(0, -1));
        assert_eq!(g.world_to_cell(Vec2::new(-0.5, 0.5)), IVec2::new(-1, 1));
        assert_eq!(g.cell_origin(IVec2::new(-2, 3)), Vec2::new(-1.0, 1.5));
        assert_eq!(g.cell_center(IVec2::new(-2, 3)), Vec2::new(-0.75, 1.75));
    }

    #[test]
    fn loaded_cell_bounds_cover_loaded_chunks() {
        let mut g = grid(16);
        assert_eq!(g.loaded_cell_bounds(), None);
        g.set(IVec2::new(-1, 0), 1);
        g.set(IVec2::new(20, 40), 1);
        assert_eq!(
            g.loaded_cell_bounds(),
            Some((IVec2::new(-16, 0), IVec2::new(32, 48)))
        );
    }

    #[test]
    fn window_spanning_four_chunks_with_one_absent_matches_reference() {
        let mut g = grid(8);
        for chunk in [IVec2::new(-1, -1), IVec2::new(0, -1), IVec2::new(0, 0)] {
            let first = chunk * 8;
            for y in 0..8 {
                for x in 0..8 {
                    let cell = first + IVec2::new(x, y);
                    g.set(cell, pattern(cell));
                }
            }
        }
        let min = IVec2::new(-5, -6);
        let size = UVec2::new(11, 9);
        let window = g.window(min, size);

        assert_eq!(window.info().origin, g.cell_origin(min));
        assert_eq!(window.info().resolution, 0.5);
        for (pos, value) in window.indexed_iter() {
            assert_eq!(value, g.get(min + pos.as_ivec2()), "at {pos}");
        }
        assert_eq!(window[UVec2::new(0, 8)], COST_UNKNOWN);
        assert_eq!(window[UVec2::new(10, 0)], pattern(IVec2::new(5, -6)));
    }

    #[test]
    fn window_leaves_cells_beyond_i32_as_fill() {
        let mut g = grid(8);
        g.set(IVec2::splat(i32::MAX), 7);
        let window = g.window(IVec2::splat(i32::MAX - 1), UVec2::splat(4));
        assert_eq!(window[UVec2::new(1, 1)], 7);
        assert_eq!(window[UVec2::new(2, 2)], COST_UNKNOWN);
        assert_eq!(window[UVec2::new(3, 0)], COST_UNKNOWN);
    }

    #[test]
    fn window_centered_places_center_cell_in_middle() {
        let mut g = grid(8);
        let cell = IVec2::new(-3, 12);
        g.set(cell, 42);
        let window = g.window_centered(g.cell_center(cell), UVec2::new(7, 5));
        assert_eq!(window[UVec2::new(3, 2)], 42);
    }

    #[test]
    fn window_write_window_round_trip() {
        let mut g = grid(8);
        let info = MapInfo {
            width: 20,
            height: 13,
            resolution: 0.5,
            origin: g.cell_origin(IVec2::new(-9, -4)),
        };
        let source = DenseGrid::new_with(info, |x, y| pattern(IVec2::new(x as i32, y as i32)));
        g.write_window(&source).unwrap();

        let back = g.window(IVec2::new(-9, -4), UVec2::new(20, 13));
        assert_eq!(back.as_slice(), source.as_slice());
        assert_eq!(g.loaded_chunks().count(), 12);
    }

    #[test]
    fn read_into_rejects_resolution_mismatch() {
        let g = grid(8);
        let mut target = DenseGrid::<u8>::new_with_value(MapInfo::square(4, 0.25), 0);
        assert!(matches!(
            g.read_into(&mut target),
            Err(VoxelError::InvalidArgument(_))
        ));
        let mut g = g;
        assert!(g.write_window(&target).is_err());
    }

    #[test]
    fn read_into_snaps_origin_and_overwrites_absent_cells() {
        let mut g = grid(8);
        g.set(IVec2::new(2, 3), 11);
        let info = MapInfo {
            width: 4,
            height: 4,
            resolution: 0.5,
            origin: Vec2::new(0.52, 0.98),
        };
        let mut target = DenseGrid::new_with_value(info, 99u8);
        g.read_into(&mut target).unwrap();
        // The origin snaps to cell (1, 2), so cell (2, 3) lands at (1, 1).
        assert_eq!(target[UVec2::new(1, 1)], 11);
        assert_eq!(target[UVec2::new(0, 0)], COST_UNKNOWN);
    }

    #[test]
    fn evict_outside_drops_far_chunks() {
        let mut g = grid(16);
        for cx in -3..=3 {
            g.set(IVec2::new(cx * 16, 0), 1);
        }
        let dropped = g.evict_outside(IVec2::new(0, 0), 16);
        // The square [-16, 16] touches chunks -1, 0 and 1.
        assert_eq!(dropped, 4);
        let mut kept: Vec<_> = g.loaded_chunks().map(|c| c.x).collect();
        kept.sort_unstable();
        assert_eq!(kept, [-1, 0, 1]);
        assert_eq!(g.evict_outside(IVec2::ZERO, 0), 2);
        assert_eq!(g.loaded_chunks().collect::<Vec<_>>(), [IVec2::ZERO]);
    }

    #[test]
    fn overview_max_uses_max_of_known_cells() {
        let mut g = grid(4);
        // One 2x2 block straddling the origin: cells (-2..0, -2..0).
        g.set(IVec2::new(-2, -2), 10);
        g.set(IVec2::new(-1, -2), 30);
        g.set(IVec2::new(-1, -1), 20);
        // A block with a single known cell next to unknown ones.
        g.set(IVec2::new(2, 2), 5);
        // A block whose only value is lethal.
        g.set(IVec2::new(6, 0), 254);

        let o = g.overview_max(2).unwrap();
        assert_eq!(o.resolution(), 1.0);
        assert_eq!(o.chunk_size(), 4);
        assert_eq!(*o.get(IVec2::new(-1, -1)), 30);
        assert_eq!(*o.get(IVec2::new(1, 1)), 5);
        assert_eq!(*o.get(IVec2::new(3, 0)), 254);
        assert_eq!(*o.get(IVec2::new(0, 0)), COST_UNKNOWN);
        assert_eq!(*o.get(IVec2::new(-2, -1)), COST_UNKNOWN);
    }

    #[test]
    fn overview_max_unknown_only_if_all_unknown_and_skips_empty_chunks() {
        let mut g = grid(4);
        g.set(IVec2::new(0, 0), COST_UNKNOWN);
        g.set(IVec2::new(1, 1), 0);
        g.insert_chunk(IVec2::new(5, 5), vec![COST_UNKNOWN; 16].into())
            .unwrap();

        let o = g.overview_max(2).unwrap();
        assert_eq!(*o.get(IVec2::ZERO), 0);
        assert_eq!(o.loaded_chunks().collect::<Vec<_>>(), [IVec2::ZERO]);
    }

    #[test]
    fn overview_max_factor_not_dividing_chunk_size() {
        let mut g = grid(4);
        g.set(IVec2::new(2, 0), 8);
        g.set(IVec2::new(4, 0), 9);
        let o = g.overview_max(3).unwrap();
        assert_eq!(*o.get(IVec2::new(0, 0)), 8);
        assert_eq!(*o.get(IVec2::new(1, 0)), 9);
        assert!(g.overview_max(0).is_err());
        assert_eq!(g.overview_max(1).unwrap().resolution(), 0.5);
    }
}
