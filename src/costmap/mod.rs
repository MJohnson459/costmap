pub mod layered;
pub mod merge;
pub mod project;

pub use layered::{Layer, LayerId, LayeredCostmap};
pub use merge::{merge_max, merge_max_keep_unknown, merge_overwrite};
pub use project::{MergePolicy, project_into};

use crate::Grid2d;

pub type Costmap = Grid2d<u8>;
