//! 2D costmaps for robot navigation: grids, raycasting, obstacle and inflation
//! layers, planning, and maps too large to hold in memory. No ROS dependency.
//!
//! - [`DenseGrid`] stores every cell of a bounded map. [`Costmap`] is a `DenseGrid<u8>`
//!   of costs and [`OccupancyGrid`] a `DenseGrid<i8>` of occupancy; the values in
//!   [`types`] follow Nav2's costmap_2d.
//! - [`LayeredCostmap`] combines [`Layer`]s into one costmap that can follow a robot:
//!   [`ObstacleLayer`] from sensor beams, [`WavefrontInflationLayer`], and
//!   [`StaticLayer`] for a map of any size, on any lattice and in any frame.
//! - [`resample`] copies a map onto another lattice, given the transform between
//!   their frames.
//! - [`raycast`] and [`iterators`] walk cells along rays and inside polygons.
//! - [`plan`] is an 8-connected, cost-weighted A*. [`TileGraph`] plans across a map a
//!   tile at a time and reads only the tiles on the route. [`RoutePlanner`] plans
//!   between world positions on a whole map, on a graph it stores beside the map.
//! - [`TiledGrid`] stores a map as sparse chunks.
//! - [`CostWindows`] is the one request every cost map answers: the cells of a
//!   rectangle as a dense window. [`Costmap`], [`TiledGrid`] and, with the `geotiff`
//!   feature, `GeoTiffWindows` answer it. [`InflatedWindows`] inflates each window
//!   with the margin it needs.
//! - [`RosMapLoader`] reads ROS map YAML files. With the `geotiff` feature,
//!   `CostGeoTiff` reads the chunks of a cost GeoTIFF that a region needs.
//!   [`MapFile`] opens either kind of map by its extension.
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

pub use costmap::resample::{Resample, resample};
pub use costmap::{
    Costmap, Layer, LayerId, LayeredCostmap, MergePolicy, cost_from_occupancy,
    costmap_from_occupancy, occupancy_from_cost, project_into,
};
#[allow(deprecated)]
pub use grid::Grid2d;
pub use grid::{CostWindows, DenseGrid, InflatedWindows};
pub use layers::{
    Beam, InflationConfig, ObstacleConfig, ObstacleLayer, ProjectionLayer, StaticLayer,
    WavefrontInflationLayer,
};
#[cfg(feature = "geotiff")]
pub use loaders::geotiff::{CostGeoTiff, write_cost_geotiff, write_cost_geotiff_region};
#[cfg(feature = "geotiff")]
pub use loaders::geotiff_windows::{ChunkStats, GeoTiffWindows};
pub use loaders::map_file::{GeoTiffReading, MapFile, Margin};
pub use loaders::ros2::{RosMapLoader, RosMapMetadata};
pub use planning::{
    GraphOrigin, Path, PlanOptions, Route, RouteError, RouteOptions, RoutePlanner, TileCache,
    TileGraph, TileGraphOptions, TileLayout, line_passable, plan,
};
pub use tiled::{GeoReference, TiledGrid};
#[allow(deprecated)]
pub use types::VoxelError;
pub use types::{CostmapError, Lattice, OccupancyGrid, cost_from_range, cost_from_unit};
