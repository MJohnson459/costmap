//! Sparse tiled grids for maps too large for a dense [`DenseGrid`](crate::DenseGrid).
//!
//! [`TiledGrid`] stores fixed-size chunks addressed by signed cell coordinates.
//! Existing algorithms run on a dense window copied out of the tiles, and
//! [`GeoReference`] ties the local metric frame to projected coordinates.

mod georef;
mod grid;

pub use georef::GeoReference;
pub use grid::TiledGrid;
#[cfg(feature = "geotiff")]
pub(crate) use grid::max_known;
