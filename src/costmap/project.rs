//! Generic projection of semantic grids into the costmap.
//!
//! Allows any `Grid2d<T>` with a projection closure `T → Option<u8>` to be merged
//! into the master costmap using a specified merge policy.

use glam::UVec2;

use crate::types::{COST_UNKNOWN, CellRegion};

use super::Costmap;

/// Merge policy for projecting a source grid into the master costmap.
///
/// These policies mirror the behavior of Nav2's cost-combining layers,
/// respecting the special semantics of COST_UNKNOWN as "no information."
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MergePolicy {
    /// Overwrite: set master to the projected cost (do not copy COST_UNKNOWN).
    Overwrite,
    /// Max: keep the maximum of master and source cost; never writes COST_UNKNOWN.
    Max,
    /// MaxKeepUnknown: like Max, but does not overwrite unknown cells in master.
    MaxKeepUnknown,
}

/// Project a source grid into the master costmap within a region.
///
/// For each cell in `region`, applies the projection function `f` to the source value.
/// If `f` returns `Some(cost)`, the cost is merged into the master according to the policy.
/// If `f` returns `None`, the master cell is left untouched.
///
/// # Panics
/// In debug builds, panics if the master and source grids do not have matching layouts
/// (width, height, resolution, origin).
pub fn project_into<T>(
    master: &mut Costmap,
    source: &crate::Grid2d<T>,
    region: CellRegion,
    policy: MergePolicy,
    f: impl Fn(&T) -> Option<u8>,
) {
    debug_assert!(
        master.layout_matches(source),
        "master and source grids must have matching layouts"
    );

    for y in region.min.y..region.max.y {
        for x in region.min.x..region.max.x {
            let cell = UVec2::new(x, y);

            // Read from source and apply projection.
            let Some(src_val) = source.get(cell) else {
                continue;
            };
            let Some(src_cost) = f(src_val) else {
                continue;
            };

            // Read current master cell (defaults to COST_UNKNOWN if not set).
            let old = master.get(cell).copied().unwrap_or(COST_UNKNOWN);

            // Apply the policy.
            let new_cost = match policy {
                MergePolicy::Overwrite => Some(src_cost),
                MergePolicy::Max => {
                    if old == COST_UNKNOWN || old < src_cost {
                        Some(src_cost)
                    } else {
                        None
                    }
                }
                MergePolicy::MaxKeepUnknown => {
                    if old != COST_UNKNOWN && old < src_cost {
                        Some(src_cost)
                    } else {
                        None
                    }
                }
            };

            if let Some(cost) = new_cost {
                let _ = master.set(cell, cost);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Grid2d;
    use crate::types::{COST_FREE, MapInfo};

    fn default_info() -> MapInfo {
        MapInfo {
            width: 10,
            height: 10,
            resolution: 0.1,
            ..Default::default()
        }
    }

    fn default_region() -> CellRegion {
        CellRegion {
            min: UVec2::new(0, 0),
            max: UVec2::new(10, 10),
        }
    }

    #[test]
    fn test_project_into_none_leaves_master_untouched() {
        let mut master = Costmap::new_with_value(default_info(), COST_FREE);
        let source = Grid2d::<u8>::new_with_value(default_info(), 200);

        // Set a specific value to verify it's not changed
        master.set(UVec2::new(5, 5), 100).unwrap();

        // Projection that always returns None
        project_into(
            &mut master,
            &source,
            default_region(),
            MergePolicy::Overwrite,
            |_| None,
        );

        assert_eq!(master.get(UVec2::new(5, 5)), Some(&100));
        assert_eq!(master.get(UVec2::new(0, 0)), Some(&COST_FREE));
    }

    #[test]
    fn test_project_into_overwrite() {
        let mut master = Costmap::new_with_value(default_info(), COST_FREE);
        let source = Grid2d::<u8>::new_with_value(default_info(), 100);

        let region = CellRegion {
            min: UVec2::new(2, 2),
            max: UVec2::new(5, 5),
        };

        project_into(&mut master, &source, region, MergePolicy::Overwrite, |v| {
            Some(*v)
        });

        // Inside region
        assert_eq!(master.get(UVec2::new(2, 2)), Some(&100));
        assert_eq!(master.get(UVec2::new(4, 4)), Some(&100));
        // Outside region
        assert_eq!(master.get(UVec2::new(0, 0)), Some(&COST_FREE));
        assert_eq!(master.get(UVec2::new(9, 9)), Some(&COST_FREE));
    }

    #[test]
    fn test_project_into_max() {
        let mut master = Costmap::new_with_value(default_info(), COST_FREE);
        master.set(UVec2::new(3, 3), 50).unwrap();
        master.set(UVec2::new(4, 4), 150).unwrap();

        let source = Grid2d::<u8>::new_with_value(default_info(), 100);

        let region = CellRegion {
            min: UVec2::new(2, 2),
            max: UVec2::new(5, 5),
        };

        project_into(&mut master, &source, region, MergePolicy::Max, |v| Some(*v));

        // 50 < 100 → update to 100
        assert_eq!(master.get(UVec2::new(3, 3)), Some(&100));
        // 150 > 100 → keep 150
        assert_eq!(master.get(UVec2::new(4, 4)), Some(&150));
        // COST_FREE (0) < 100 → update to 100
        assert_eq!(master.get(UVec2::new(2, 2)), Some(&100));
    }

    #[test]
    fn test_project_into_max_keep_unknown() {
        let mut master = Costmap::new_with_value(default_info(), COST_UNKNOWN);
        master.set(UVec2::new(3, 3), 50).unwrap();
        master.set(UVec2::new(4, 4), 150).unwrap();

        let source = Grid2d::<u8>::new_with_value(default_info(), 100);

        let region = CellRegion {
            min: UVec2::new(2, 2),
            max: UVec2::new(5, 5),
        };

        project_into(
            &mut master,
            &source,
            region,
            MergePolicy::MaxKeepUnknown,
            |v| Some(*v),
        );

        // 50 < 100 → update to 100
        assert_eq!(master.get(UVec2::new(3, 3)), Some(&100));
        // 150 > 100 → keep 150
        assert_eq!(master.get(UVec2::new(4, 4)), Some(&150));
        // COST_UNKNOWN → keep COST_UNKNOWN (do not update)
        assert_eq!(master.get(UVec2::new(2, 2)), Some(&COST_UNKNOWN));
    }

    #[test]
    fn test_project_into_generic_type() {
        let mut master = Costmap::new_with_value(default_info(), COST_FREE);
        let source = Grid2d::<f32>::new_with_value(default_info(), 0.5);

        let region = default_region();

        // Project f32 to u8 cost
        project_into(&mut master, &source, region, MergePolicy::Overwrite, |v| {
            Some((*v * 254.0).round() as u8)
        });

        assert_eq!(master.get(UVec2::new(5, 5)), Some(&127)); // 0.5 * 254 ≈ 127
    }

    #[test]
    fn test_project_into_respects_unknown_in_source() {
        let mut master = Costmap::new_with_value(default_info(), COST_FREE);
        let mut source = Grid2d::<u8>::new_with_value(default_info(), 100);
        source.set(UVec2::new(5, 5), COST_UNKNOWN).unwrap();

        let region = default_region();

        // Projection that treats COST_UNKNOWN as None
        project_into(&mut master, &source, region, MergePolicy::Overwrite, |v| {
            if *v == COST_UNKNOWN { None } else { Some(*v) }
        });

        // COST_UNKNOWN cell should not be updated
        assert_eq!(master.get(UVec2::new(5, 5)), Some(&COST_FREE));
        // Other cells should be updated
        assert_eq!(master.get(UVec2::new(3, 3)), Some(&100));
    }
}
