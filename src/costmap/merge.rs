//! Merge policies for writing a layer grid into the master costmap.
//!
//! **Assumption:** `master` and `source` share the same dimensions and alignment so that
//! cell `(x, y)` in `region` is valid in both grids.

use crate::types::{COST_UNKNOWN, CellRegion};

use super::{Costmap, MergePolicy, project_into};

/// Copies source into master only where source is not unknown.
pub fn merge_overwrite(master: &mut Costmap, source: &Costmap, region: CellRegion) {
    project_into(master, source, region, MergePolicy::Overwrite, |cost| {
        (*cost != COST_UNKNOWN).then_some(*cost)
    });
}

/// Merges source into master by taking the maximum cost; never writes unknown from the layer.
pub fn merge_max(master: &mut Costmap, source: &Costmap, region: CellRegion) {
    project_into(master, source, region, MergePolicy::Max, |cost| {
        (*cost != COST_UNKNOWN).then_some(*cost)
    });
}

/// Like [`merge_max`] but does not overwrite master cells that are unknown.
pub fn merge_max_keep_unknown(master: &mut Costmap, source: &Costmap, region: CellRegion) {
    project_into(
        master,
        source,
        region,
        MergePolicy::MaxKeepUnknown,
        |cost| (*cost != COST_UNKNOWN).then_some(*cost),
    );
}
