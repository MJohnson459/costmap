pub mod layered;
pub mod merge;
pub mod occupancy;
pub mod project;

pub use layered::{Layer, LayerId, LayeredCostmap};
pub use merge::{merge_max, merge_max_keep_unknown, merge_overwrite};
pub use occupancy::{cost_from_occupancy, costmap_from_occupancy, occupancy_from_cost};
pub use project::{MergePolicy, project_into};

use crate::DenseGrid;

pub type Costmap = DenseGrid<u8>;
