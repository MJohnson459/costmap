pub mod dense;
pub mod helpers;

pub use dense::DenseGrid;

/// The name of [`DenseGrid`] before 0.3.
#[deprecated(since = "0.3.0", note = "renamed to `DenseGrid`")]
pub type Grid2d<T> = DenseGrid<T>;
