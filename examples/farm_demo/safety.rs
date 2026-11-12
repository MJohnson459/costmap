//! Safety measured against ground truth rather than against the planner's own map.
//!
//! Ground truth is the full-resolution aerial map, read through the paged tiles, plus
//! the surprise obstacles. The robot is a disc of [`ROBOT_RADIUS_M`](crate::ROBOT_RADIUS_M).
//! Its clearance at a point is the distance from the disc's edge to the nearest lethal
//! map cell or surprise obstacle; a negative clearance means the disc overlaps one,
//! and a centre inside an obstacle gives minus the radius.
//! Unknown map cells are not obstacles: the map has no data there.

use costmap::{TiledGrid, types::COST_LETHAL};
use glam::{IVec2, Vec2};

use crate::{route::resample, world::Obstacle};

/// Obstacles are searched for this far beyond the footprint's edge. A clearance larger
/// than this is reported as this.
pub const CLEARANCE_HORIZON_M: f32 = 2.0;
/// Largest gap between the points checked along a driven path, in cells of the map.
const SAMPLE_SPACING_CELLS: f32 = 0.5;

/// Running record of the footprint's clearance along everything driven so far.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyMonitor {
    radius_m: f32,
    /// Paths checked whose footprint overlapped an obstacle somewhere.
    violations: usize,
    min_clearance_m: f32,
}

impl SafetyMonitor {
    pub fn new(radius_m: f32) -> Self {
        Self {
            radius_m,
            violations: 0,
            min_clearance_m: CLEARANCE_HORIZON_M,
        }
    }

    /// Number of checked paths along which the footprint overlapped an obstacle.
    pub fn violations(&self) -> usize {
        self.violations
    }

    /// Smallest clearance seen, capped at [`CLEARANCE_HORIZON_M`].
    pub fn min_clearance_m(&self) -> f32 {
        self.min_clearance_m
    }

    /// Check the footprint along the polyline `path`, at points no more than half a map
    /// cell apart including both ends, against the lethal cells of `tiles` and
    /// `obstacles`. `tiles` must hold every chunk within the clearance horizon of the
    /// path; cells of unloaded chunks read as unknown. Returns the path's smallest
    /// clearance, capped at [`CLEARANCE_HORIZON_M`].
    pub fn check_path(
        &mut self,
        tiles: &TiledGrid<u8>,
        obstacles: &[Obstacle],
        path: &[Vec2],
    ) -> f32 {
        let resolution = tiles.resolution();
        let points = resample(path, SAMPLE_SPACING_CELLS * resolution);
        let reach = self.radius_m + CLEARANCE_HORIZON_M;
        let lethal = lethal_cells_near(tiles, &points, reach);
        let clearance = points
            .iter()
            .map(|&p| {
                let to_cells = lethal
                    .iter()
                    .map(|&cell| distance_to_square(p, cell, resolution));
                let to_obstacles = obstacles.iter().map(|o| o.distance_to(p));
                to_cells.chain(to_obstacles).fold(reach, f32::min) - self.radius_m
            })
            .fold(CLEARANCE_HORIZON_M, f32::min);
        if clearance < 0.0 {
            self.violations += 1;
        }
        self.min_clearance_m = self.min_clearance_m.min(clearance);
        clearance
    }
}

/// Lower-left corners of the lethal cells of `tiles` within `reach` of the bounding box
/// of `points`.
fn lethal_cells_near(tiles: &TiledGrid<u8>, points: &[Vec2], reach: f32) -> Vec<Vec2> {
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    let (min, max) = points
        .iter()
        .fold((first, first), |(lo, hi), &p| (lo.min(p), hi.max(p)));
    let lo = tiles.world_to_cell(min - reach);
    let hi = tiles.world_to_cell(max + reach);
    let size = (hi - lo + IVec2::ONE).as_uvec2();
    let window = tiles.window(lo, size);
    window
        .indexed_iter()
        .filter(|&(_, &cost)| cost == COST_LETHAL)
        .map(|(cell, _)| tiles.cell_origin(lo + cell.as_ivec2()))
        .collect()
}

/// Distance from `point` to the square of side `side` whose lower-left corner is `corner`;
/// zero inside it.
fn distance_to_square(point: Vec2, corner: Vec2, side: f32) -> f32 {
    let outside = (corner - point)
        .max(point - (corner + side))
        .max(Vec2::ZERO);
    outside.length()
}

#[cfg(test)]
mod tests {
    use costmap::types::{COST_FREE, COST_UNKNOWN};

    use super::*;

    const RADIUS_M: f32 = 0.4;

    /// 0.1 m tiles, free over 0..20 m on both axes.
    fn free_tiles() -> TiledGrid<u8> {
        let mut tiles = TiledGrid::new(0.1, 64, COST_UNKNOWN).unwrap();
        for y in 0..200 {
            for x in 0..200 {
                tiles.set(IVec2::new(x, y), COST_FREE);
            }
        }
        tiles
    }

    fn trailer() -> Obstacle {
        Obstacle {
            centre: Vec2::new(10.0, 10.0),
            axis: Vec2::X,
            half_size: Vec2::new(1.5, 1.0),
        }
    }

    #[test]
    fn driving_through_a_surprise_obstacle_is_a_violation() {
        let mut monitor = SafetyMonitor::new(RADIUS_M);
        let path = [Vec2::new(5.0, 10.0), Vec2::new(15.0, 10.0)];
        let clearance = monitor.check_path(&free_tiles(), &[trailer()], &path);
        assert_eq!(clearance, -RADIUS_M);
        assert_eq!(monitor.violations(), 1);
    }

    #[test]
    fn grazing_past_an_obstacle_is_measured_to_the_footprint_edge() {
        let mut monitor = SafetyMonitor::new(RADIUS_M);
        // 1.5 m from the trailer's side, which is 1 m from its centre line.
        let path = [Vec2::new(5.0, 12.5), Vec2::new(15.0, 12.5)];
        let clearance = monitor.check_path(&free_tiles(), &[trailer()], &path);
        assert!((clearance - 1.1).abs() < 1e-4, "{clearance}");
        assert_eq!(monitor.violations(), 0);
        assert_eq!(monitor.min_clearance_m(), clearance);
    }

    #[test]
    fn a_lethal_map_cell_under_the_footprint_is_a_violation() {
        let mut tiles = free_tiles();
        // Cell (100, 103) spans x 10.0..10.1 and y 10.3..10.4: 0.3 m from the path.
        tiles.set(IVec2::new(100, 103), COST_LETHAL);
        let mut monitor = SafetyMonitor::new(RADIUS_M);
        let path = [Vec2::new(5.0, 10.0), Vec2::new(15.0, 10.0)];
        let clearance = monitor.check_path(&tiles, &[], &path);
        assert!((clearance + 0.1).abs() < 1e-4, "{clearance}");
        assert_eq!(monitor.violations(), 1);
    }

    #[test]
    fn a_single_pose_on_open_ground_has_the_horizon_as_clearance() {
        let mut monitor = SafetyMonitor::new(RADIUS_M);
        let path = [Vec2::new(5.0, 5.0)];
        assert_eq!(
            monitor.check_path(&free_tiles(), &[], &path),
            CLEARANCE_HORIZON_M
        );
        assert_eq!(monitor.violations(), 0);
    }
}
