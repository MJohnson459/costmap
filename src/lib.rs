pub mod grid;
pub mod inflation;
pub mod iterators;
pub mod loaders;
pub mod raycast;
pub mod types;
pub mod visualization;

#[cfg(feature = "rerun")]
pub mod rerun_viz;

#[cfg(feature = "nav2_compat")]
pub mod nav2_compat;

pub use grid::{Bounds, CellRegion, Footprint, Grid, Grid2d, Layer, LayeredGrid2d, Pose2};
pub use inflation::{InflationCostCurve, InflationLayer};
pub use loaders::ros2::{RosMapLoader, RosMapMetadata};
pub use types::{MapInfo, VoxelError};

pub type OccupancyGrid = Grid2d<i8>;
