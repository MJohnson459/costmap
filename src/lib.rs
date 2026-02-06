pub mod grid;
pub mod iterators;
pub mod loaders;
pub mod raycast;
pub mod types;
pub mod visualization;

#[cfg(feature = "rerun")]
pub mod rerun_viz;

#[cfg(feature = "nav2_compat")]
pub mod nav2_compat;

pub use grid::{AggregatePolicy, Grid, Grid2d, LayerMeta, LayeredGrid, OccupiedDominant};
pub use loaders::ros2::{RosMapLoader, RosMapMetadata};
pub use types::{MapInfo, VoxelError};

pub type OccupancyGrid = Grid2d<i8>;
