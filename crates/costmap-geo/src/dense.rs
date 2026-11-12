use glam::{DVec2, UVec2};

/// A dense row-major grid. Row 0 is the southern row and x points east, like [`costmap::TiledGrid`].
#[derive(Debug, Clone, PartialEq)]
pub struct DenseGrid<T> {
    size: UVec2,
    data: Vec<T>,
}

impl<T: Copy> DenseGrid<T> {
    /// A grid of `size` cells all holding `value`.
    pub fn new(size: UVec2, value: T) -> Self {
        Self {
            size,
            data: vec![value; size.x as usize * size.y as usize],
        }
    }

    /// Build a grid by evaluating `f(x, y)` for every cell.
    pub fn from_fn(size: UVec2, mut f: impl FnMut(u32, u32) -> T) -> Self {
        let mut data = Vec::with_capacity(size.x as usize * size.y as usize);
        for y in 0..size.y {
            for x in 0..size.x {
                data.push(f(x, y));
            }
        }
        Self { size, data }
    }

    pub fn size(&self) -> UVec2 {
        self.size
    }

    /// Value at `(x, y)`; the caller keeps the coordinates inside the grid.
    pub fn at(&self, x: u32, y: u32) -> T {
        self.data[self.index(x, y)]
    }

    pub fn set(&mut self, x: u32, y: u32, value: T) {
        let index = self.index(x, y);
        self.data[index] = value;
    }

    /// The cells as one row-major slice.
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// One row of cells, from west to east.
    pub fn row_mut(&mut self, y: u32) -> &mut [T] {
        let width = self.size.x as usize;
        let start = y as usize * width;
        &mut self.data[start..start + width]
    }

    fn index(&self, x: u32, y: u32) -> usize {
        y as usize * self.size.x as usize + x as usize
    }
}

/// Where a [`DenseGrid`] sits in projected coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellFrame {
    /// Projected position of the lower-left corner of cell `(0, 0)`.
    pub origin: DVec2,
    /// Cell edge length in metres.
    pub resolution: f64,
}

impl CellFrame {
    /// Projected position of the centre of cell `(x, y)`; the cell may lie outside the grid.
    pub fn centre(&self, x: i64, y: i64) -> DVec2 {
        self.origin + DVec2::new(x as f64 + 0.5, y as f64 + 0.5) * self.resolution
    }
}
