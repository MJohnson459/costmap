pub mod grid;
pub mod iterators;
pub mod layers;
pub mod loaders;
pub mod raycast;
pub mod types;
pub mod visualization;

#[cfg(feature = "rerun")]
pub mod rerun_viz;

pub use grid::{Grid, Grid2d, Layer, LayeredGrid2d};
pub use layers::{InflationConfig, WavefrontInflationLayer};
pub use loaders::ros2::{RosMapLoader, RosMapMetadata};
pub use types::{Bounds, CellRegion, Footprint, MapInfo, Pose2, VoxelError};

pub type OccupancyGrid = Grid2d<i8>;
