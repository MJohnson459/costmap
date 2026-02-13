pub mod layered;
pub mod merge;

pub use layered::{Layer, LayeredCostmap};
pub use merge::{merge_max, merge_max_keep_unknown, merge_overwrite};

use crate::Grid2d;

pub type Costmap = Grid2d<u8>;
