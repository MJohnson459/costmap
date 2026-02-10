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

pub use grid::{Grid, Grid2d, Layer, LayeredGrid2d};
pub use inflation::{InflationCostCurve, InflationLayer};
pub use loaders::ros2::{RosMapLoader, RosMapMetadata};
pub use types::{Bounds, CellRegion, Footprint, MapInfo, Pose2, VoxelError};

pub type OccupancyGrid = Grid2d<i8>;
