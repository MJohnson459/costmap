//! 2D costmaps for robot navigation: grids, raycasting, obstacle and inflation
//! layers, planning, and maps too large to hold in memory. No ROS dependency.
//!
//! - [`DenseGrid`] stores every cell of a bounded map. [`Costmap`] is a `DenseGrid<u8>`
//!   of costs and [`OccupancyGrid`] a `DenseGrid<i8>` of occupancy; the values in
//!   [`types`] follow Nav2's costmap_2d.
//! - [`LayeredCostmap`] combines [`Layer`]s into one costmap that can follow a robot:
//!   [`ObstacleLayer`] from sensor beams, [`WavefrontInflationLayer`], and
//!   [`TiledMapLayer`] for a static map of any size.
//! - [`raycast`] and [`iterators`] walk cells along rays and inside polygons.
//! - [`plan`] is an 8-connected, cost-weighted A*. [`TileGraph`] plans across a map a
//!   tile at a time and reads only the tiles on the route.
//! - [`TiledGrid`] stores a map as sparse chunks; `window` cuts a [`DenseGrid`] from it.
//! - [`RosMapLoader`] reads ROS map YAML files. With the `geotiff` feature,
//!   `CostGeoTiff` reads the chunks of a cost GeoTIFF that a region needs.
//! - With the `rerun` feature, `rerun_viz` logs grids to the Rerun viewer.
//!
//! World positions, resolutions and extents are `f64`; quantities local to a grid or
//! a robot are `f32`. A grid's [`Lattice`] converts between world positions and cells.

pub mod costmap;
pub mod grid;
pub mod iterators;
pub mod layers;
pub mod loaders;
pub mod planning;
pub mod raycast;
pub mod tiled;
pub mod types;

#[cfg(feature = "rerun")]
pub mod rerun_viz;

pub use costmap::{
    Costmap, Layer, LayerId, LayeredCostmap, MergePolicy, cost_from_occupancy,
    costmap_from_occupancy, occupancy_from_cost, project_into,
};
pub use grid::DenseGrid;
#[allow(deprecated)]
pub use grid::Grid2d;
pub use layers::{
    Beam, InflationConfig, ObstacleConfig, ObstacleLayer, ProjectionLayer, TiledMapLayer,
    WavefrontInflationLayer,
};
#[cfg(feature = "geotiff")]
pub use loaders::geotiff::{CostGeoTiff, write_cost_geotiff, write_cost_geotiff_region};
pub use loaders::ros2::{RosMapLoader, RosMapMetadata};
pub use planning::{
    DenseTiles, Path, PlanOptions, TileCache, TileGraph, TileGraphOptions, TileLayout, TileSource,
    line_passable, plan,
};
pub use tiled::{GeoReference, TiledGrid};
pub use types::{Lattice, MapInfo, OccupancyGrid, VoxelError, cost_from_range, cost_from_unit};
