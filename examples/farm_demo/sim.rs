//! The robot loop: page tiles, sense, update the local costmap, plan, drive.

use std::{collections::VecDeque, fmt, sync::Arc};

use costmap::{
    CostGeoTiff, Costmap, InflationConfig, LayerId, LayeredCostmap, MapInfo, MergePolicy,
    ObstacleConfig, ObstacleLayer, PlanOptions, TiledGrid, TiledMapLayer, VoxelError,
    WavefrontInflationLayer, line_passable, plan,
    planning::Regions,
    types::{COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN, Pose2},
};
use glam::{DVec2, IVec2, UVec2};

use crate::{
    ROBOT_RADIUS_M,
    route::{Overview, Route},
    safety::SafetyMonitor,
    world::{self, LIDAR_RANGE_M, Obstacle},
};

/// Fixed simulation time step.
pub const STEP_S: f32 = 0.1;
/// Edge of the square rolling local costmap.
const LOCAL_WINDOW_M: f32 = 30.0;
/// Target edge of a tile chunk; the cell count is rounded to the map resolution.
const CHUNK_EDGE_M: f32 = 12.8;
/// Tiles are loaded within this distance beyond the local window's edge...
const LOAD_MARGIN_M: f32 = 5.0;
/// ...and evicted only beyond this one, so a robot moving along a chunk boundary
/// does not load and evict the same chunks over and over.
const EVICT_MARGIN_M: f32 = 15.0;
/// A hit sits at the centre of the cell the beam entered, up to √2/2 ≈ 0.71 cells
/// beyond the beam's range, so the obstacle layer accepts hits this many cells further
/// out; the margin over √2/2 absorbs `f32` rounding.
const HIT_RANGE_SLACK_CELLS: f32 = 0.75;
/// Lidar marks are forgotten this long after they were last seen.
const OBSTACLE_DECAY_S: f32 = 10.0;
const LOCAL_INFLATION_RADIUS_M: f32 = 1.5;
const LOCAL_COST_SCALING: f32 = 3.0;
const LOCAL_PLAN: PlanOptions = PlanOptions {
    lethal_threshold: COST_INSCRIBED,
    allow_unknown: false,
    cost_weight: 3.0,
};
/// The local plan is refreshed at least this often, and sooner if it becomes blocked.
const REPLAN_PERIOD_STEPS: u64 = 5;
/// The carrot stays this far inside the local window, so the plan to it has room
/// to bend round an obstacle at the window's edge.
const CARROT_EDGE_MARGIN_M: f32 = 2.0;
/// A carrot other than the goal is at least this far from the robot.
const MIN_CARROT_DISTANCE_M: f32 = 1.0;
/// Progress along the route is searched this far ahead of the last known progress.
const PROGRESS_SEARCH_M: f32 = 10.0;
/// A local plan longer by more than this than the route to the same carrot is a detour.
/// Lateral distance from the route is no measure: plans to a carrot cut the route's
/// corners, and the overview and the local costmap resolve the same gap differently.
pub const DETOUR_EXTRA_LENGTH_M: f32 = 0.5;
const GOAL_TOLERANCE_M: f32 = 0.5;
/// The run times out after this multiple of the route's driving time, plus the slack,
/// counted from the start or from the latest route replan.
const TIMEOUT_FACTOR: f32 = 3.0;
const TIMEOUT_SLACK_S: f32 = 60.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Status {
    Driving,
    Reached,
    /// The robot can reach no point of the route and the recovery failed.
    Blocked {
        at: DVec2,
        reason: BlockReason,
    },
    TimedOut,
}

/// Why the recovery from an unreachable route failed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlockReason {
    /// The robot has observed no obstacle since the route was planned that the overview
    /// did not already hold, so a new route would be the same.
    NothingNewObserved,
    /// The overview, with the observed obstacles marked, has no route to the goal.
    NoRouteAroundObserved,
    /// A new route was planned, and the robot can reach no point of it either.
    NoLocalPathToNewRoute,
}

impl fmt::Display for BlockReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NothingNewObserved => {
                "no local path to the route, and nothing observed that the route did not allow for"
            }
            Self::NoRouteAroundObserved => "no route to the goal around the observed obstacles",
            Self::NoLocalPathToNewRoute => "no local path to the replanned route",
        })
    }
}

/// Counters for the summary.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Stats {
    pub tile_loads: usize,
    pub tile_evictions: usize,
    pub peak_chunks: usize,
    pub replans: usize,
    /// Routes replanned on the overview around observed obstacles.
    pub global_replans: usize,
    /// Local plans longer than the route by more than [`DETOUR_EXTRA_LENGTH_M`].
    pub detours: usize,
    /// Detours planned while a surprise obstacle was within lidar range.
    pub detours_near_surprise: usize,
    pub driven_m: f32,
}

pub struct Simulation {
    map: CostGeoTiff,
    costmap: LayeredCostmap,
    tiles_layer: LayerId,
    obstacle_layer: LayerId,
    /// The global planner's map; obstacles the lidar observes are marked on it when the
    /// route is replanned.
    overview: Overview,
    route: Route,
    surprises: Vec<Obstacle>,
    speed_mps: f32,
    pose: Pose2,
    steps: u64,
    /// Index of the route point the robot has progressed to.
    progress: usize,
    local_plan: VecDeque<DVec2>,
    steps_since_plan: u64,
    driven: Vec<DVec2>,
    hits: Vec<DVec2>,
    status: Status,
    stats: Stats,
    safety: SafetyMonitor,
    timeout_s: f32,
}

impl Simulation {
    /// Set up the local costmap at the start of `route` over the tiles of `map`, whose
    /// overview is `overview`.
    pub fn new(
        map: CostGeoTiff,
        overview: Overview,
        route: Route,
        surprises: Vec<Obstacle>,
        speed_mps: f32,
    ) -> Result<Self, VoxelError> {
        let resolution = map.resolution();
        let chunk_cells = (f64::from(CHUNK_EDGE_M) / resolution).round().max(1.0) as u32;
        let tiles = TiledGrid::new(resolution, chunk_cells, COST_UNKNOWN)?;
        let start = route.points()[0];
        let side = (f64::from(LOCAL_WINDOW_M) / resolution).round() as u32;
        // The window origin sits on a tile cell boundary, and the rolling window moves
        // in whole cells, so the tiles stamp into the window without resampling.
        let half = DVec2::splat(f64::from(side) * resolution / 2.0);
        let info = MapInfo {
            width: side,
            height: side,
            resolution,
            origin: ((start - half) / resolution).floor() * resolution,
        };
        let mut costmap = LayeredCostmap::new(info, COST_UNKNOWN, true);
        let hit_range_m = LIDAR_RANGE_M + HIT_RANGE_SLACK_CELLS * resolution as f32;
        let tiles_layer = costmap.add_layer(Box::new(TiledMapLayer::new(
            Arc::new(tiles),
            MergePolicy::Overwrite,
        )))?;
        let obstacle_layer = costmap.add_layer(Box::new(ObstacleLayer::new(
            info,
            ObstacleConfig {
                rolling_window: true,
                obstacle_range_m: hit_range_m,
                raytrace_range_m: hit_range_m,
                decay_s: Some(OBSTACLE_DECAY_S),
                policy: MergePolicy::Max,
            },
        )))?;
        costmap.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
            inflation_radius_m: LOCAL_INFLATION_RADIUS_M,
            inscribed_radius_m: ROBOT_RADIUS_M,
            cost_scaling_factor: LOCAL_COST_SCALING,
            ..Default::default()
        })))?;
        let timeout_s = driving_timeout_s(&route, speed_mps);
        let heading = route.direction_at(0);
        Ok(Self {
            map,
            costmap,
            tiles_layer,
            obstacle_layer,
            overview,
            pose: Pose2::new(start, heading.to_angle()),
            route,
            surprises,
            speed_mps,
            steps: 0,
            progress: 0,
            local_plan: VecDeque::new(),
            steps_since_plan: 0,
            driven: vec![start],
            hits: Vec::new(),
            status: Status::Driving,
            stats: Stats::default(),
            safety: SafetyMonitor::new(ROBOT_RADIUS_M),
            timeout_s,
        })
    }

    /// Advance one time step. Does nothing once the run has ended.
    pub fn step(&mut self) -> Result<(), VoxelError> {
        if self.status != Status::Driving {
            return Ok(());
        }
        self.page_tiles()?;
        self.sense();
        self.costmap.update_map(self.pose);
        self.update_progress();
        if self.needs_replan()
            && let Err(reason) = self.replan()
        {
            self.status = Status::Blocked {
                at: self.pose.position,
                reason,
            };
            return Ok(());
        }
        let driven = self.drive();
        let tiles = tiles_of(&self.costmap, self.tiles_layer);
        self.safety.check_path(tiles, &self.surprises, &driven);
        self.steps += 1;
        if self.pose.position.distance(self.route.goal()) <= f64::from(GOAL_TOLERANCE_M) {
            self.status = Status::Reached;
        } else if self.time_s() > self.timeout_s {
            self.status = Status::TimedOut;
        }
        Ok(())
    }

    /// Load the chunks near the robot and evict those far from it.
    fn page_tiles(&mut self) -> Result<(), VoxelError> {
        let position = self.pose.position;
        let half_window = LOCAL_WINDOW_M / 2.0;
        let layer = self
            .costmap
            .layer_mut::<TiledMapLayer>(self.tiles_layer)
            .expect("layer added as TiledMapLayer");
        let tiles = layer.grid_mut();
        let resolution = tiles.resolution();
        let centre = tiles.world_to_cell(position);
        let load =
            IVec2::splat((f64::from(half_window + LOAD_MARGIN_M) / resolution).ceil() as i32);
        let evict = (f64::from(half_window + EVICT_MARGIN_M) / resolution).ceil() as u32;
        self.stats.tile_loads += self
            .map
            .load_region(tiles, centre - load, centre + load + 1)?;
        // Counted before eviction: the chunks just loaded and those about to be evicted
        // are in memory together.
        self.stats.peak_chunks = self.stats.peak_chunks.max(tiles.loaded_chunks().count());
        self.stats.tile_evictions += tiles.evict_outside(centre, evict);
        Ok(())
    }

    /// Scan the ground truth and feed the returns to the obstacle layer.
    fn sense(&mut self) {
        let position = self.pose.position;
        let truth = world::ground_truth(self.tiles(), position, &self.surprises);
        let beams = world::scan(&truth, position);
        self.hits = beams.iter().filter(|b| b.hit).map(|b| b.end).collect();
        let now = f64::from(self.time_s());
        let layer = self
            .costmap
            .layer_mut::<ObstacleLayer>(self.obstacle_layer)
            .expect("layer added as ObstacleLayer");
        layer.insert_beams(position, beams, now);
        layer.expire(now);
    }

    /// Move the progress index to the nearest route point a short way ahead.
    fn update_progress(&mut self) {
        let horizon = self.route.distance_to_point(self.progress) + PROGRESS_SEARCH_M;
        let last = self.route.index_at(horizon);
        let position = self.pose.position;
        self.progress = (self.progress..=last)
            .min_by(|&a, &b| {
                let da = self.route.points()[a].distance_squared(position);
                let db = self.route.points()[b].distance_squared(position);
                da.total_cmp(&db)
            })
            .unwrap_or(self.progress);
    }

    fn needs_replan(&self) -> bool {
        self.local_plan.is_empty()
            || self.steps_since_plan >= REPLAN_PERIOD_STEPS
            || self.plan_blocked()
    }

    /// Whether the rest of the local plan, from the robot's cell on, crosses a cell that
    /// is no longer passable in the latest local costmap.
    fn plan_blocked(&self) -> bool {
        let master = self.costmap.master();
        let cells: Option<Vec<UVec2>> = std::iter::once(self.pose.position)
            .chain(self.local_plan.iter().copied())
            .map(|p| master.world_to_map(p))
            .collect();
        cells.is_none_or(|cells| {
            cells
                .windows(2)
                .any(|pair| !line_passable(master, pair[0], pair[1], &LOCAL_PLAN))
        })
    }

    /// Plan from the robot to a carrot on the route, and replan the route if the robot
    /// can reach none.
    ///
    /// The recovery is bounded: the route is replanned only after the robot has
    /// observed an obstacle the overview did not hold, and the overview's cells are
    /// finite.
    fn replan(&mut self) -> Result<(), BlockReason> {
        if self.plan_to_carrot() {
            return Ok(());
        }
        self.replan_route()?;
        if self.plan_to_carrot() {
            Ok(())
        } else {
            Err(BlockReason::NoLocalPathToNewRoute)
        }
    }

    /// Plan to the furthest carrot or, if the robot cannot reach it, to the furthest
    /// carrot in the robot's region of the local costmap. Returns whether a plan was made.
    fn plan_to_carrot(&mut self) -> bool {
        let carrots = self.carrots();
        let Some(&furthest) = carrots.first() else {
            return false;
        };
        if self.plan_to(furthest) {
            return true;
        }
        let master = self.costmap.master();
        let regions = Regions::new(master, &LOCAL_PLAN);
        let region_of = |p: DVec2| master.world_to_map(p).and_then(|c| regions.region(c));
        let Some(robot) = region_of(self.pose.position) else {
            return false;
        };
        let points = self.route.points();
        let reachable = carrots[1..]
            .iter()
            .copied()
            .find(|&i| region_of(points[i]) == Some(robot));
        reachable.is_some_and(|carrot| self.plan_to(carrot))
    }

    /// Mark the obstacles the lidar currently holds on the overview and plan a new
    /// route from the robot to the goal on it.
    ///
    /// Fails without replanning if marking changed no overview cell: the route would be
    /// planned on the same map as before.
    fn replan_route(&mut self) -> Result<(), BlockReason> {
        let layer = self
            .costmap
            .layer::<ObstacleLayer>(self.obstacle_layer)
            .expect("layer added as ObstacleLayer");
        let grid = layer.grid();
        let observed = grid
            .indexed_iter()
            .filter(|&(_, &cost)| cost == COST_LETHAL)
            .map(|(cell, _)| grid.map_to_world(cell));
        if self.overview.mark_lethal(observed) == 0 {
            return Err(BlockReason::NothingNewObserved);
        }
        let route = self
            .overview
            .plan_route(self.pose.position, self.route.goal())
            .map_err(|_| BlockReason::NoRouteAroundObserved)?;
        self.timeout_s = self.time_s() + driving_timeout_s(&route, self.speed_mps);
        self.route = route;
        self.progress = 0;
        self.local_plan.clear();
        self.stats.global_replans += 1;
        Ok(())
    }

    /// Plan from the robot to route point `carrot`. Returns `false` if no plan exists.
    fn plan_to(&mut self, carrot: usize) -> bool {
        let master = self.costmap.master();
        let (Some(start), Some(goal)) = (
            master.world_to_map(self.pose.position),
            master.world_to_map(self.route.points()[carrot]),
        ) else {
            return false;
        };
        let Some(path) = plan(master, start, goal, &LOCAL_PLAN) else {
            return false;
        };
        let mut points: Vec<DVec2> = path
            .shortcut(master, &LOCAL_PLAN)
            .into_iter()
            .map(|cell| master.map_to_world(cell))
            .collect();
        if carrot == self.route.points().len() - 1
            && let Some(last) = points.last_mut()
        {
            *last = self.route.goal();
        }
        self.stats.replans += 1;
        if polyline_length(&points) - self.route_length_to(carrot) > DETOUR_EXTRA_LENGTH_M {
            self.stats.detours += 1;
            let position = self.pose.position;
            if self
                .surprises
                .iter()
                .any(|s| s.distance_to(position) <= LIDAR_RANGE_M)
            {
                self.stats.detours_near_surprise += 1;
            }
        }
        self.local_plan = points.into();
        self.steps_since_plan = 0;
        true
    }

    /// Length of the way to route point `index` along the route: from the robot to the
    /// progress point, then along the route.
    fn route_length_to(&self, index: usize) -> f32 {
        let to_route = self
            .pose
            .position
            .distance(self.route.points()[self.progress]) as f32;
        to_route + self.route.distance_to_point(index) - self.route.distance_to_point(self.progress)
    }

    /// Indices of the candidate carrots, furthest along the route first: route points
    /// ahead of the robot that are inside the local window, before the route first
    /// leaves it, and passable. Points within [`MIN_CARROT_DISTANCE_M`] of the robot are
    /// left out, except the goal, so a robot that can reach nothing further ahead does
    /// not plan to where it stands.
    fn carrots(&self) -> Vec<usize> {
        let master = self.costmap.master();
        let info = master.info();
        let margin = DVec2::splat(f64::from(CARROT_EDGE_MARGIN_M));
        let min = info.origin + margin;
        let max = info.origin + DVec2::new(info.world_width(), info.world_height()) - margin;
        let passable = |p: DVec2| {
            master
                .world_to_map(p)
                .is_some_and(|c| master[c] < COST_INSCRIBED)
        };
        let goal = self.route.points().len() - 1;
        let position = self.pose.position;
        let mut carrots: Vec<usize> = (self.route.points()[self.progress..].iter())
            .take_while(|p| p.cmpge(min).all() && p.cmple(max).all())
            .enumerate()
            .map(|(i, &p)| (self.progress + i, p))
            .filter(|&(i, p)| {
                passable(p)
                    && (i == goal || p.distance(position) >= f64::from(MIN_CARROT_DISTANCE_M))
            })
            .map(|(i, _)| i)
            .collect();
        carrots.reverse();
        carrots
    }

    /// Follow the local plan for one step at the set speed.
    ///
    /// The robot moves along the straight segments between the plan's waypoints, which
    /// cross only cells that were passable when the plan was made and that
    /// [`plan_blocked`](Self::plan_blocked) rechecks every step. Returns the path driven
    /// in this step: the start, each waypoint reached, and the end.
    fn drive(&mut self) -> Vec<DVec2> {
        let mut budget = f64::from(self.speed_mps * STEP_S);
        let start = self.pose.position;
        let mut path = vec![start];
        let mut position = start;
        while budget > 0.0 {
            let Some(&target) = self.local_plan.front() else {
                break;
            };
            let gap = position.distance(target);
            if gap <= budget {
                position = target;
                budget -= gap;
                self.local_plan.pop_front();
            } else {
                position += (target - position) / gap * budget;
                budget = 0.0;
            }
            path.push(position);
        }
        if let Some(heading) = (position - start).try_normalize() {
            self.pose = Pose2::new(position, heading.to_angle() as f32);
        }
        self.stats.driven_m += polyline_length(&path);
        self.driven.push(position);
        self.steps_since_plan += 1;
        path
    }

    pub fn status(&self) -> Status {
        self.status
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    pub fn steps(&self) -> u64 {
        self.steps
    }

    pub fn time_s(&self) -> f32 {
        self.steps as f32 * STEP_S
    }

    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    pub fn route(&self) -> &Route {
        &self.route
    }

    pub fn overview(&self) -> &Overview {
        &self.overview
    }

    pub fn surprises(&self) -> &[Obstacle] {
        &self.surprises
    }

    pub fn driven(&self) -> &[DVec2] {
        &self.driven
    }

    /// Lidar hit points of the latest scan.
    pub fn hits(&self) -> &[DVec2] {
        &self.hits
    }

    /// The rest of the local plan, starting at the robot.
    pub fn local_plan(&self) -> Vec<DVec2> {
        std::iter::once(self.pose.position)
            .chain(self.local_plan.iter().copied())
            .collect()
    }

    /// The rolling local costmap.
    pub fn local_costmap(&self) -> &Costmap {
        self.costmap.master()
    }

    /// The tiles currently paged in.
    pub fn tiles(&self) -> &TiledGrid<u8> {
        tiles_of(&self.costmap, self.tiles_layer)
    }

    /// Footprint clearance against ground truth over everything driven so far.
    pub fn safety(&self) -> SafetyMonitor {
        self.safety
    }

    /// Number of chunks in the raster's extent, with or without data.
    pub fn extent_chunks(&self) -> usize {
        let chunk = self.tiles().chunk_size();
        let chunks = self.map.size().map(|s| s.div_ceil(chunk));
        chunks.x as usize * chunks.y as usize
    }

    /// Map size in cells.
    pub fn map_size(&self) -> UVec2 {
        self.map.size()
    }
}

/// Time allowed for driving `route` at `speed_mps`.
fn driving_timeout_s(route: &Route, speed_mps: f32) -> f32 {
    route.length_m() / speed_mps * TIMEOUT_FACTOR + TIMEOUT_SLACK_S
}

/// The tiles of the tiled-map layer `id` of `costmap`.
fn tiles_of(costmap: &LayeredCostmap, id: LayerId) -> &TiledGrid<u8> {
    costmap
        .layer::<TiledMapLayer>(id)
        .expect("layer added as TiledMapLayer")
        .grid()
}

fn polyline_length(points: &[DVec2]) -> f32 {
    points
        .windows(2)
        .map(|pair| pair[0].distance(pair[1]) as f32)
        .sum()
}

#[cfg(test)]
mod tests {
    use costmap::types::COST_FREE;
    use glam::Vec2;

    use super::*;
    use crate::{
        route::{ROUTE_POINT_SPACING_M, resample},
        testing::{TempDir, write_map},
    };

    const SPEED_MPS: f32 = 1.5;

    /// A map `size_m` across at 0.1 m with costs `cost`, written to `dir`, and its overview.
    fn open_map(
        dir: &TempDir,
        size_m: DVec2,
        cost: impl Fn(DVec2) -> u8,
    ) -> (CostGeoTiff, Overview) {
        let path = write_map(dir, "map.tif", size_m, 0.1, cost);
        let mut map = CostGeoTiff::open(path).unwrap();
        let overview =
            Overview::load(&mut map, crate::route::DEFAULT_OVERVIEW_RESOLUTION_M).unwrap();
        (map, overview)
    }

    /// A simulation on a straight route from `from` to `to`, whatever the overview holds.
    fn straight_run(
        dir: &TempDir,
        size_m: DVec2,
        cost: impl Fn(DVec2) -> u8,
        from: DVec2,
        to: DVec2,
    ) -> Simulation {
        let (map, overview) = open_map(dir, size_m, cost);
        let route = Route::new(resample(&[from, to], ROUTE_POINT_SPACING_M));
        Simulation::new(map, overview, route, Vec::new(), SPEED_MPS).unwrap()
    }

    /// A simulation on the route the overview plans from `from` to `to`.
    fn planned_run(
        dir: &TempDir,
        size_m: DVec2,
        cost: impl Fn(DVec2) -> u8,
        (from, to): (DVec2, DVec2),
        surprises: Vec<Obstacle>,
    ) -> Simulation {
        let (map, overview) = open_map(dir, size_m, cost);
        let route = overview.plan_route(from, to).unwrap();
        Simulation::new(map, overview, route, surprises, SPEED_MPS).unwrap()
    }

    /// A wall `length` long and 0.5 m thick, centred on `centre`, along `axis`.
    fn wall(centre: DVec2, axis: Vec2, length: f32) -> Obstacle {
        Obstacle {
            centre,
            axis,
            half_size: Vec2::new(length / 2.0, 0.25),
        }
    }

    /// Two east-west corridors, 6 m wide, at y 8..14 and 26..32, joined at both ends;
    /// everything else is lethal.
    fn two_corridors(p: DVec2) -> u8 {
        let between = |v: f64, lo: f64, hi: f64| (lo..hi).contains(&v);
        let along = between(p.x, 10.0, 110.0);
        let corridor = along && (between(p.y, 8.0, 14.0) || between(p.y, 26.0, 32.0));
        let link =
            (between(p.x, 10.0, 16.0) || between(p.x, 104.0, 110.0)) && between(p.y, 8.0, 32.0);
        if corridor || link {
            COST_FREE
        } else {
            COST_LETHAL
        }
    }

    #[test]
    fn a_blocked_corridor_leads_to_a_new_route_through_the_other() {
        let dir = TempDir::new("corridors");
        let ends = (DVec2::new(20.0, 11.0), DVec2::new(100.0, 11.0));
        // Fills the southern corridor's width at x = 60.
        let block = wall(DVec2::new(60.0, 11.0), Vec2::Y, 8.0);
        let mut sim = planned_run(
            &dir,
            DVec2::new(120.0, 40.0),
            two_corridors,
            ends,
            vec![block],
        );
        assert!(sim.route().points().iter().all(|p| p.y < 20.0));
        run_to_end(&mut sim, 5000);

        assert_eq!(sim.status(), Status::Reached, "at {}", sim.pose().position);
        assert!(sim.stats().global_replans >= 1);
        assert!(sim.route().points().iter().any(|p| p.y > 26.0));
        assert_eq!(sim.safety().violations(), 0);
    }

    #[test]
    fn an_enclosed_robot_ends_blocked() {
        let dir = TempDir::new("enclosed");
        let ends = (DVec2::new(10.0, 20.0), DVec2::new(35.0, 20.0));
        let box_around = |c: DVec2| {
            vec![
                wall(c + DVec2::new(0.0, 3.0), Vec2::X, 7.0),
                wall(c - DVec2::new(0.0, 3.0), Vec2::X, 7.0),
                wall(c + DVec2::new(3.0, 0.0), Vec2::Y, 7.0),
                wall(c - DVec2::new(3.0, 0.0), Vec2::Y, 7.0),
            ]
        };
        let surprises = box_around(ends.0);
        let mut sim = planned_run(&dir, DVec2::new(40.0, 40.0), |_| COST_FREE, ends, surprises);
        run_to_end(&mut sim, 1000);

        assert!(
            matches!(
                sim.status(),
                Status::Blocked {
                    reason: BlockReason::NoRouteAroundObserved,
                    ..
                }
            ),
            "{:?} after {} steps",
            sim.status(),
            sim.steps()
        );
        assert!(sim.steps() < 100, "{} steps", sim.steps());
        assert_eq!(sim.safety().violations(), 0);
    }

    /// Step until the run ends, at most `limit` steps.
    fn run_to_end(sim: &mut Simulation, limit: u64) {
        while sim.status() == Status::Driving && sim.steps() < limit {
            sim.step().unwrap();
        }
    }

    #[test]
    fn the_robot_does_not_plan_across_a_band_of_no_data() {
        let dir = TempDir::new("no_data");
        let band = 48.0..52.0;
        let cost = |p: DVec2| {
            if band.contains(&p.x) {
                COST_UNKNOWN
            } else {
                COST_FREE
            }
        };
        let (from, to) = (DVec2::new(40.0, 20.0), DVec2::new(60.0, 20.0));
        let mut sim = straight_run(&dir, DVec2::new(100.0, 40.0), cost, from, to);
        run_to_end(&mut sim, 1000);

        assert!(
            matches!(sim.status(), Status::Blocked { .. }),
            "{:?} at {} after {} steps",
            sim.status(),
            sim.pose().position,
            sim.steps()
        );
        assert!(
            sim.driven().iter().all(|p| p.x < band.start),
            "entered the band"
        );
        let local = sim.local_costmap();
        let in_band = local.world_to_map(DVec2::new(50.0, 20.0)).unwrap();
        assert_eq!(local[in_band], COST_UNKNOWN);
    }

    #[test]
    fn a_hit_beyond_lidar_range_by_most_of_a_cell_is_marked() {
        // At 1 m cells, the 45 degree beam from (10.5, 10.5) enters lethal cell (21, 21)
        // at its corner, 14.85 m out, and the hit sits at the cell's centre, 15.56 m out.
        let dir = TempDir::new("hit_range");
        let block = |p: DVec2| {
            if p.x > 21.0 && p.y > 21.0 {
                COST_LETHAL
            } else {
                COST_FREE
            }
        };
        let path = write_map(&dir, "map.tif", DVec2::splat(40.0), 1.0, block);
        let mut map = CostGeoTiff::open(path).unwrap();
        let overview =
            Overview::load(&mut map, crate::route::DEFAULT_OVERVIEW_RESOLUTION_M).unwrap();
        let route = Route::new(vec![DVec2::splat(10.5), DVec2::new(10.5, 12.5)]);
        let mut sim = Simulation::new(map, overview, route, Vec::new(), SPEED_MPS).unwrap();
        sim.page_tiles().unwrap();
        sim.sense();

        let hit = DVec2::splat(21.5);
        assert!(sim.hits().contains(&hit), "{:?}", sim.hits());
        let layer = sim
            .costmap
            .layer::<ObstacleLayer>(sim.obstacle_layer)
            .unwrap();
        let cell = layer.grid().world_to_map(hit).unwrap();
        assert_eq!(layer.grid()[cell], COST_LETHAL);
    }

    #[test]
    fn peak_chunks_count_chunks_loaded_before_eviction() {
        let dir = TempDir::new("paging");
        let (from, to) = (DVec2::new(20.0, 25.0), DVec2::new(280.0, 25.0));
        let mut sim = straight_run(&dir, DVec2::new(300.0, 50.0), |_| COST_FREE, from, to);
        sim.page_tiles().unwrap();
        let held_at_start = sim.tiles().loaded_chunks().count();

        // A jump far along the route loads a new set before the old one is evicted.
        sim.pose = Pose2::new(to, 0.0);
        sim.page_tiles().unwrap();
        let stats = sim.stats();
        assert!(stats.tile_evictions > 0);
        assert_eq!(stats.peak_chunks, stats.tile_loads);
        assert!(stats.peak_chunks > held_at_start);
    }
}
