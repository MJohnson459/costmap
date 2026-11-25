//! Conversion between `costmap` cost grids and `nav_msgs/OccupancyGrid`.
//!
//! Cells are converted with [`costmap::occupancy_from_cost`] and
//! [`costmap::cost_from_occupancy`], the rule Nav2's `StaticLayer` applies.

use costmap::types::{COST_UNKNOWN, CellRegion, CostmapError};
use costmap::{Costmap, DenseGrid, Lattice, cost_from_occupancy, occupancy_from_cost};
use glam::{DVec2, UVec2};
use hiroz_msgs::builtin_interfaces::Time;
use hiroz_msgs::geometry_msgs::{Point, Pose, Quaternion};
use hiroz_msgs::nav_msgs::{MapMetaData, OccupancyGrid as OccupancyGridMsg};
use hiroz_msgs::std_msgs::Header;

/// `nav_msgs/OccupancyGrid` holding `costmap` through [`occupancy_from_cost`].
///
/// Both layouts are row-major from the cell at the lower-left corner, so cells
/// copy across in order. The message origin is the lower-left corner of the
/// grid's first cell. The message holds the resolution as `f32`.
pub fn occupancy_grid_from_costmap(
    costmap: &Costmap,
    frame_id: &str,
    stamp: Time,
) -> OccupancyGridMsg {
    let (size, origin) = (costmap.size(), costmap.world_bounds().min);
    OccupancyGridMsg {
        header: Header {
            stamp,
            frame_id: frame_id.to_owned(),
        },
        info: MapMetaData {
            map_load_time: stamp,
            resolution: costmap.lattice().resolution as f32,
            width: size.x,
            height: size.y,
            origin: Pose {
                position: Point {
                    x: origin.x,
                    y: origin.y,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        },
        data: costmap
            .as_slice()
            .iter()
            .map(|&cost| occupancy_from_cost(cost))
            .collect(),
    }
}

/// Cost grid read from a `nav_msgs/OccupancyGrid` through [`cost_from_occupancy`].
///
/// Fails if the data length does not match the dimensions, or if the origin
/// is rotated: a [`Lattice`] has no yaw. The grid holds the cells from (0, 0)
/// of a lattice anchored at the message origin.
pub fn costmap_from_occupancy_grid(msg: &OccupancyGridMsg) -> Result<Costmap, CostmapError> {
    let q = &msg.info.origin.orientation;
    if q.x != 0.0 || q.y != 0.0 || q.z != 0.0 {
        return Err(CostmapError::InvalidMetadata(format!(
            "rotated map origin ({}, {}, {}, {}) is not supported",
            q.x, q.y, q.z, q.w
        )));
    }
    let lattice = Lattice::new(
        DVec2::new(msg.info.origin.position.x, msg.info.origin.position.y),
        f64::from(msg.info.resolution),
    );
    let cells = CellRegion::from_size(UVec2::new(msg.info.width, msg.info.height));
    let data = msg
        .data
        .iter()
        .map(|&occ| cost_from_occupancy(occ))
        .collect();
    Ok(DenseGrid::from_vec(lattice, cells, data)?.with_fill(COST_UNKNOWN))
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::types::{COST_FREE, COST_INSCRIBED, COST_LETHAL};

    fn sample_costmap() -> Costmap {
        let lattice = Lattice::new(DVec2::new(-1.5, 2.0), 0.25);
        let cells = CellRegion::from_size(UVec2::new(3, 2));
        let data = vec![
            COST_FREE,
            10,
            COST_INSCRIBED,
            COST_LETHAL,
            COST_UNKNOWN,
            128,
        ];
        Costmap::from_vec(lattice, cells, data).unwrap()
    }

    #[test]
    fn message_keeps_layout_and_cell_order() {
        let stamp = Time {
            sec: 12,
            nanosec: 34,
        };
        let msg = occupancy_grid_from_costmap(&sample_costmap(), "map", stamp);
        assert_eq!(msg.header.frame_id, "map");
        assert_eq!((msg.header.stamp.sec, msg.header.stamp.nanosec), (12, 34));
        assert_eq!((msg.info.width, msg.info.height), (3, 2));
        assert_eq!(msg.info.resolution, 0.25);
        assert_eq!(msg.info.origin.position.x, -1.5);
        assert_eq!(msg.info.origin.position.y, 2.0);
        assert_eq!(msg.info.origin.orientation.w, 1.0);
        assert_eq!(msg.data, vec![0, 4, 99, 100, -1, 51]);
    }

    #[test]
    fn message_round_trip_keeps_layout_and_special_costs() {
        let costmap = sample_costmap();
        let msg = occupancy_grid_from_costmap(&costmap, "map", Time::default());
        let back = costmap_from_occupancy_grid(&msg).unwrap();
        assert_eq!(
            (back.lattice(), back.cells()),
            (costmap.lattice(), costmap.cells())
        );
        let special = [COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN];
        for (sent, received) in costmap.as_slice().iter().zip(back.as_slice()) {
            if special.contains(sent) {
                assert_eq!(sent, received);
            }
        }
    }

    #[test]
    fn rotated_origin_is_rejected() {
        let mut msg = occupancy_grid_from_costmap(&sample_costmap(), "map", Time::default());
        msg.info.origin.orientation.z = std::f64::consts::FRAC_1_SQRT_2;
        msg.info.origin.orientation.w = std::f64::consts::FRAC_1_SQRT_2;
        assert!(costmap_from_occupancy_grid(&msg).is_err());
    }

    #[test]
    fn data_length_mismatch_is_rejected() {
        let mut msg = occupancy_grid_from_costmap(&sample_costmap(), "map", Time::default());
        msg.data.pop();
        assert!(costmap_from_occupancy_grid(&msg).is_err());
    }
}
