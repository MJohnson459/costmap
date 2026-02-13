//! Merge policies for writing a layer grid into the master costmap.
//!
//! **Assumption:** `master` and `source` share the same dimensions and alignment so that
//! cell `(x, y)` in `region` is valid in both grids.

use glam::UVec2;

use crate::types::{COST_UNKNOWN, CellRegion};

use super::Costmap;

/// Copies source into master only where source is not unknown.
pub fn merge_overwrite(master: &mut Costmap, source: &Costmap, region: CellRegion) {
    for y in region.min.y..region.max.y {
        for x in region.min.x..region.max.x {
            let cell = UVec2::new(x, y);
            if let Some(&cost) = source.get(cell)
                && cost != COST_UNKNOWN
            {
                let _ = master.set(cell, cost);
            }
        }
    }
}

/// Merges source into master by taking the maximum cost; never writes unknown from the layer.
pub fn merge_max(master: &mut Costmap, source: &Costmap, region: CellRegion) {
    for y in region.min.y..region.max.y {
        for x in region.min.x..region.max.x {
            let cell = UVec2::new(x, y);
            let Some(&src_cost) = source.get(cell) else {
                continue;
            };
            if src_cost == COST_UNKNOWN {
                continue;
            }
            let old = master.get(cell).copied().unwrap_or(COST_UNKNOWN);
            if old == COST_UNKNOWN || old < src_cost {
                let _ = master.set(cell, src_cost);
            }
        }
    }
}

/// Like [`merge_max`] but does not overwrite master cells that are unknown.
pub fn merge_max_keep_unknown(master: &mut Costmap, source: &Costmap, region: CellRegion) {
    for y in region.min.y..region.max.y {
        for x in region.min.x..region.max.x {
            let cell = UVec2::new(x, y);
            let Some(&src_cost) = source.get(cell) else {
                continue;
            };
            if src_cost == COST_UNKNOWN {
                continue;
            }
            let old = master.get(cell).copied().unwrap_or(COST_UNKNOWN);
            if old != COST_UNKNOWN && old < src_cost {
                let _ = master.set(cell, src_cost);
            }
        }
    }
}
