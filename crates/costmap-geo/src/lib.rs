//! Navigation cost maps from OpenDroneMap orthophoto, DSM and DTM rasters.
//!
//! The rules in [`rules`], [`slope`] and [`vegetation`] and the polygon fill in [`overrides`] are pure
//! functions over in-memory grids. [`raster`] reads the GeoTIFF inputs through a chunk
//! cache and [`pipeline`] ties everything together and writes the cost GeoTIFF.

pub mod dense;
pub mod error;
pub mod overrides;
pub mod pipeline;
pub mod raster;
pub mod rules;
pub mod slope;
pub mod vegetation;

pub use error::GeoError;
pub use pipeline::{Job, Summary, run, run_in_blocks};
pub use rules::CostRules;
