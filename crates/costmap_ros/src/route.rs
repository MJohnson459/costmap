//! Routes across a whole cost GeoTIFF.
//!
//! Nav2 holds a window of the map, so its own planner cannot reach a goal
//! outside it. [`RoutePlanner`] holds a max-pooled overview of the whole
//! raster, which is small enough to keep in memory, and plans on that.

use std::path::Path;

use costmap::types::{COST_UNKNOWN, VoxelError};
use costmap::{CostGeoTiff, Costmap, PlanOptions, plan};
use glam::{DVec2, IVec2, UVec2};
use thiserror::Error;

/// Chunk edge of the tiled grid the overview is read into.
const CHUNK_CELLS: u32 = 256;
const COST_SCALING: f32 = 2.0;

/// How a [`RoutePlanner`] builds its overview.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteOptions {
    /// Largest overview cell, in metres. The overview cell is the map cell
    /// times the largest power of two that keeps it at or below this.
    pub overview_m: f64,
    /// Robot radius in metres. Overview cells nearer than this to an obstacle
    /// are impassable.
    pub robot_radius_m: f32,
    /// Distance from obstacles over which cost falls off, in metres. Routes
    /// keep away from obstacles by up to this much where there is room.
    pub inflation_m: f32,
    /// Farthest a start or goal is moved to reach a passable overview cell,
    /// in metres.
    pub snap_m: f64,
    /// Largest gap between consecutive route points, in metres.
    pub point_spacing_m: f64,
}

impl Default for RouteOptions {
    fn default() -> Self {
        Self {
            overview_m: 0.25,
            robot_radius_m: 0.3,
            inflation_m: 1.5,
            snap_m: 1.0,
            point_spacing_m: 0.1,
        }
    }
}

/// A route could not be planned.
#[derive(Debug, Error, PartialEq)]
pub enum RouteError {
    #[error("{name} {point} has no passable cell within {snap_m} m")]
    Blocked {
        name: &'static str,
        point: DVec2,
        snap_m: f64,
    },
    #[error("no route from {start} to {goal}")]
    NoRoute { start: DVec2, goal: DVec2 },
}

/// A route in the map frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// Points from start to goal, at most `point_spacing_m` apart.
    pub points: Vec<DVec2>,
    /// Length along the points, in metres.
    pub length_m: f64,
}

/// Plans routes on an overview of a whole cost GeoTIFF.
pub struct RoutePlanner {
    /// The overview, inflated for the robot.
    overview: Costmap,
    options: RouteOptions,
}

impl RoutePlanner {
    /// Read the overview of the raster at `path`.
    ///
    /// The raster is streamed one strip or tile at a time, so memory use is
    /// the overview and not the raster.
    pub fn open(path: impl AsRef<Path>, options: RouteOptions) -> Result<Self, VoxelError> {
        let mut map = CostGeoTiff::open(path)?;
        let mut factor = 1;
        while map.resolution() * f64::from(2 * factor) <= options.overview_m {
            factor *= 2;
        }
        let tiles = map.read_overview_max(factor, CHUNK_CELLS)?;
        let size = map.size().map(|s| s.div_ceil(factor));
        let window = tiles.window(IVec2::ZERO, size);
        let overview =
            Costmap::from_vec(*window.info(), window.as_slice().to_vec())?.with_fill(COST_UNKNOWN);
        Ok(Self::from_overview(overview, options))
    }

    /// Plan on `costs`, which is inflated here.
    pub fn from_overview(mut costs: Costmap, options: RouteOptions) -> Self {
        costs.inflate(options.inflation_m, options.robot_radius_m, COST_SCALING);
        Self {
            overview: costs,
            options,
        }
    }

    /// Overview cell size in metres.
    pub fn resolution(&self) -> f64 {
        self.overview.info().resolution
    }

    /// Overview size in cells.
    pub fn size(&self) -> UVec2 {
        self.overview.size()
    }

    /// The route from `start` to `goal`, both in the map frame.
    ///
    /// An endpoint on an impassable overview cell moves to the nearest
    /// passable one within `snap_m`. The route starts and ends at the given
    /// points; between them it runs through overview cell centres, shortcut
    /// into straight segments where the overview allows.
    pub fn plan(&self, start: DVec2, goal: DVec2) -> Result<Route, RouteError> {
        let plan_options = PlanOptions::default();
        let start_cell = self.snap(start, "start", &plan_options)?;
        let goal_cell = self.snap(goal, "goal", &plan_options)?;
        let path = plan(&self.overview, start_cell, goal_cell, &plan_options)
            .ok_or(RouteError::NoRoute { start, goal })?;

        let mut corners = vec![start];
        corners.extend(
            path.shortcut(&self.overview, &plan_options)
                .into_iter()
                .map(|cell| self.overview.map_to_world(cell)),
        );
        corners.push(goal);

        let mut points = vec![start];
        let mut length_m = 0.0;
        for pair in corners.windows(2) {
            let (from, to) = (pair[0], pair[1]);
            let distance = from.distance(to);
            if distance == 0.0 {
                continue;
            }
            length_m += distance;
            let steps = (distance / self.options.point_spacing_m).ceil().max(1.0) as u32;
            points.extend((1..=steps).map(|i| from.lerp(to, f64::from(i) / f64::from(steps))));
        }
        Ok(Route { points, length_m })
    }

    /// The passable overview cell nearest `point`, searched in square rings.
    fn snap(
        &self,
        point: DVec2,
        name: &'static str,
        plan_options: &PlanOptions,
    ) -> Result<UVec2, RouteError> {
        let blocked = RouteError::Blocked {
            name,
            point,
            snap_m: self.options.snap_m,
        };
        let Some(centre) = self.overview.world_to_map(point) else {
            return Err(blocked);
        };
        let reach = (self.options.snap_m / self.resolution()).ceil() as i32;
        let mut best: Option<(i64, UVec2)> = None;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let Ok(cell) = UVec2::try_from(centre.as_ivec2() + IVec2::new(dx, dy)) else {
                    continue;
                };
                if costmap::planning::astar::passable_cost(&self.overview, cell, plan_options)
                    .is_none()
                {
                    continue;
                }
                let distance_sq = i64::from(dx * dx + dy * dy);
                if distance_sq <= i64::from(reach * reach)
                    && best.is_none_or(|(d, _)| distance_sq < d)
                {
                    best = Some((distance_sq, cell));
                }
            }
        }
        best.map(|(_, cell)| cell).ok_or(blocked)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::MapInfo;
    use costmap::types::{COST_FREE, COST_LETHAL};

    const OPTIONS: RouteOptions = RouteOptions {
        overview_m: 0.25,
        robot_radius_m: 0.3,
        inflation_m: 1.0,
        snap_m: 1.0,
        point_spacing_m: 0.1,
    };

    /// 20 x 20 m at 0.25 m with a wall along x = 10 m, open only for y in 15..17 m.
    fn walled_planner() -> RoutePlanner {
        let info = MapInfo::square(80, 0.25);
        let costs = Costmap::from_fn(info, |x, y| {
            if x == 40 && !(60..68).contains(&y) {
                COST_LETHAL
            } else {
                COST_FREE
            }
        });
        RoutePlanner::from_overview(costs, OPTIONS)
    }

    #[test]
    fn route_goes_through_the_gap_and_ends_at_the_given_points() {
        let planner = walled_planner();
        let (start, goal) = (DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0));
        let route = planner.plan(start, goal).unwrap();
        assert_eq!(route.points.first(), Some(&start));
        assert_eq!(route.points.last(), Some(&goal));
        let crossing = route
            .points
            .iter()
            .find(|p| p.x >= 10.0)
            .expect("the route crosses the wall line");
        assert!((15.0..17.0).contains(&crossing.y), "crossed at {crossing}");
        // Up to the gap and back down is far longer than the 16 m straight line.
        assert!(route.length_m > 30.0, "length {}", route.length_m);
    }

    #[test]
    fn points_are_no_farther_apart_than_the_spacing() {
        let route = walled_planner()
            .plan(DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0))
            .unwrap();
        let widest = route
            .points
            .windows(2)
            .map(|p| p[0].distance(p[1]))
            .fold(0.0, f64::max);
        assert!(widest <= OPTIONS.point_spacing_m + 1e-9, "gap {widest}");
        let summed: f64 = route.points.windows(2).map(|p| p[0].distance(p[1])).sum();
        assert!((summed - route.length_m).abs() < 1e-6);
    }

    #[test]
    fn an_endpoint_beside_the_wall_snaps_to_a_passable_cell() {
        let planner = walled_planner();
        // 0.2 m from the wall: inside the robot radius, so its own cell is impassable.
        let start = DVec2::new(9.8, 5.0);
        let route = planner.plan(start, DVec2::new(2.0, 2.0)).unwrap();
        assert_eq!(route.points[0], start);
    }

    #[test]
    fn an_endpoint_with_no_passable_cell_nearby_is_an_error() {
        let info = MapInfo::square(80, 0.25);
        let costs = Costmap::from_fn(info, |x, _| if x < 40 { COST_LETHAL } else { COST_FREE });
        let planner = RoutePlanner::from_overview(costs, OPTIONS);
        let start = DVec2::new(2.0, 2.0);
        let err = planner.plan(start, DVec2::new(18.0, 2.0)).unwrap_err();
        assert_eq!(
            err,
            RouteError::Blocked {
                name: "start",
                point: start,
                snap_m: 1.0
            }
        );
        let outside = planner.plan(DVec2::new(-5.0, 2.0), DVec2::new(18.0, 2.0));
        assert!(matches!(
            outside,
            Err(RouteError::Blocked { name: "start", .. })
        ));
    }

    #[test]
    fn a_closed_wall_gives_no_route() {
        let info = MapInfo::square(80, 0.25);
        let costs = Costmap::from_fn(info, |x, _| if x == 40 { COST_LETHAL } else { COST_FREE });
        let planner = RoutePlanner::from_overview(costs, OPTIONS);
        let (start, goal) = (DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0));
        assert_eq!(
            planner.plan(start, goal),
            Err(RouteError::NoRoute { start, goal })
        );
    }
}
