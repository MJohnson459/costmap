//! Occupancy values as Nav2's `StaticLayer` reads them.
//!
//! `StaticLayer::interpretValue` (navigation2 1.5.2,
//! `nav2_costmap_2d/plugins/static_layer.cpp`) turns each occupancy value `v`
//! into a cost. With `trinary_costmap: false`, `track_unknown_space: true` and
//! the default `unknown_cost_value: 255`, `inscribed_obstacle_cost_value: 99`
//! and `lethal_cost_threshold: 100`, and `v` read as an unsigned byte, the
//! rule is:
//!
//! | occupancy `v` | Nav2 cost |
//! | --- | --- |
//! | -1 (byte 255) | 255 `NO_INFORMATION` |
//! | 99 | 253 `INSCRIBED_INFLATED_OBSTACLE` |
//! | >= 100, or any other negative value | 254 `LETHAL_OBSTACLE` |
//! | 0..=98 | `floor(v / 100.0 * 254)` |
//!
//! [`cost_from_occupancy`] is the rule itself and [`occupancy_from_cost`] is
//! its inverse.

use crate::types::{COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN, VoxelError};
use crate::{Costmap, DenseGrid, OccupancyGrid};

/// Occupancy value Nav2 reads as `NO_INFORMATION`.
const OCC_UNKNOWN: i8 = -1;
/// Occupancy value Nav2 reads as `INSCRIBED_INFLATED_OBSTACLE`.
const OCC_INSCRIBED: i8 = 99;
/// Occupancy value Nav2 reads as `LETHAL_OBSTACLE`.
const OCC_LETHAL: i8 = 100;
/// Highest occupancy Nav2 scales linearly; 99 is reserved for inscribed.
const OCC_MAX_SCALED: i8 = 98;

/// Occupancy value whose Nav2 cost is the smallest cost not below `cost`.
///
/// Costs 1..=252 quantise onto occupancy 1..=99 in steps of 2.54. The
/// occupancy is rounded up, so Nav2 receives a cost between `cost` and
/// `cost + 2` for costs up to 248. Costs 249..=252 have no scaled occupancy
/// at or above them and map to 99, which Nav2 reads as inscribed (253). No
/// cost below 254 becomes lethal and no cost is lowered.
pub fn occupancy_from_cost(cost: u8) -> i8 {
    match cost {
        COST_UNKNOWN => OCC_UNKNOWN,
        COST_LETHAL => OCC_LETHAL,
        COST_INSCRIBED => OCC_INSCRIBED,
        _ => {
            // ceil(cost * 100 / 254) in integer arithmetic.
            let occ = (u16::from(cost) * 100).div_ceil(u16::from(COST_LETHAL));
            if occ > OCC_MAX_SCALED as u16 {
                OCC_INSCRIBED
            } else {
                occ as i8
            }
        }
    }
}

/// Cost Nav2's `StaticLayer` assigns to an occupancy value.
///
/// This is `StaticLayer::interpretValue` for the parameters in the module
/// documentation. Negative values other than -1 read as bytes 128..=254 and
/// are lethal there too.
pub fn cost_from_occupancy(occ: i8) -> u8 {
    match occ {
        OCC_UNKNOWN => COST_UNKNOWN,
        OCC_INSCRIBED => COST_INSCRIBED,
        0..=OCC_MAX_SCALED => (occ as u16 * u16::from(COST_LETHAL) / 100) as u8,
        _ => COST_LETHAL,
    }
}

/// Cost grid with the same layout as `grid`, through [`cost_from_occupancy`].
pub fn costmap_from_occupancy(grid: &OccupancyGrid) -> Result<Costmap, VoxelError> {
    let data = grid
        .as_slice()
        .iter()
        .map(|&occ| cost_from_occupancy(occ))
        .collect();
    Ok(DenseGrid::from_vec(*grid.info(), data)?.with_fill(COST_UNKNOWN))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MapInfo;
    use crate::types::COST_FREE;
    use glam::DVec2;

    /// `StaticLayer::interpretValue` transcribed from the C++, including its
    /// double arithmetic and truncating conversion back to a byte.
    fn nav2_interpret(occ: i8) -> u8 {
        const UNKNOWN_COST_VALUE: u8 = 255;
        const INSCRIBED_OBSTACLE_COST_VALUE: u8 = 99;
        const LETHAL_THRESHOLD: u8 = 100;
        let value = occ as u8;
        if value == UNKNOWN_COST_VALUE {
            255
        } else if value == INSCRIBED_OBSTACLE_COST_VALUE {
            253
        } else if value >= LETHAL_THRESHOLD {
            254
        } else {
            (f64::from(value) / f64::from(LETHAL_THRESHOLD) * 254.0) as u8
        }
    }

    #[test]
    fn cost_from_occupancy_matches_nav2_for_every_value() {
        for occ in i8::MIN..=i8::MAX {
            assert_eq!(cost_from_occupancy(occ), nav2_interpret(occ), "occ {occ}");
        }
    }

    #[test]
    fn special_costs_map_exactly() {
        assert_eq!(occupancy_from_cost(COST_FREE), 0);
        assert_eq!(occupancy_from_cost(COST_INSCRIBED), 99);
        assert_eq!(occupancy_from_cost(COST_LETHAL), 100);
        assert_eq!(occupancy_from_cost(COST_UNKNOWN), -1);
        for cost in [COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN] {
            assert_eq!(nav2_interpret(occupancy_from_cost(cost)), cost);
        }
    }

    #[test]
    fn round_trip_through_nav2_never_lowers_a_cost_or_invents_lethal() {
        for cost in 0..=u8::MAX {
            let received = nav2_interpret(occupancy_from_cost(cost));
            assert_eq!(cost == COST_LETHAL, received == COST_LETHAL, "cost {cost}");
            assert_eq!(
                cost == COST_UNKNOWN,
                received == COST_UNKNOWN,
                "cost {cost}"
            );
            assert!(received >= cost, "cost {cost} arrived as {received}");
        }
    }

    #[test]
    fn round_trip_error_is_within_one_quantisation_step() {
        for cost in 0..=248u8 {
            let received = nav2_interpret(occupancy_from_cost(cost));
            assert!(received - cost <= 2, "cost {cost} arrived as {received}");
        }
        for cost in 249..=COST_INSCRIBED {
            assert_eq!(nav2_interpret(occupancy_from_cost(cost)), COST_INSCRIBED);
        }
    }

    #[test]
    fn trinary_map_values_become_costs() {
        let info = MapInfo {
            width: 3,
            height: 1,
            resolution: 0.05,
            origin: DVec2::ZERO,
        };
        let grid = OccupancyGrid::from_vec(info, vec![100, 0, -1]).unwrap();
        let costs = costmap_from_occupancy(&grid).unwrap();
        assert_eq!(costs.as_slice(), &[COST_LETHAL, COST_FREE, COST_UNKNOWN]);
    }
}
