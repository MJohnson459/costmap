//! Merge policies for writing a layer grid into the master costmap.
//!
//! **Assumption:** `master` and `source` share the same dimensions and alignment so that
//! cell `(x, y)` in `region` is valid in both grids.

use glam::UVec2;

use crate::grid::Grid2d;
use crate::types::{COST_UNKNOWN, CellRegion};

/// Copies source into master only where source is not unknown.
pub fn merge_overwrite(master: &mut Grid2d<u8>, source: &Grid2d<u8>, region: CellRegion) {
    for y in region.min.y..region.max.y {
        for x in region.min.x..region.max.x {
            let cell = UVec2::new(x, y);
            if let Some(&cost) = source.get(cell) {
                if cost != COST_UNKNOWN {
                    let _ = master.set(cell, cost);
                }
            }
        }
    }
}

/// Merges source into master by taking the maximum cost; never writes unknown from the layer.
pub fn merge_max(master: &mut Grid2d<u8>, source: &Grid2d<u8>, region: CellRegion) {
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
pub fn merge_max_keep_unknown(master: &mut Grid2d<u8>, source: &Grid2d<u8>, region: CellRegion) {
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
