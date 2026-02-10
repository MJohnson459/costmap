pub mod grid2d;
pub mod layered;
pub mod merge;
pub mod traits;

pub use grid2d::Grid2d;
pub use layered::{Layer, LayeredGrid2d};
pub use merge::{
    update_master_max, update_master_max_preserve_master_unknown,
    update_master_overwrite_valid_only,
};
pub use traits::Grid;

// Re-export geometry types from types for convenience (Layer/LayeredGrid2d use them).
pub use crate::types::{Bounds, CellRegion, Footprint, Pose2};
