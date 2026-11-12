//! Sensor-driven obstacle layer: marks hits, clears free space along beams and
//! optionally forgets obstacles that are no longer observed.
//!
//! The layer owns a `u8` grid that starts out [`COST_UNKNOWN`]. A beam writes
//! [`COST_FREE`] along its path and [`COST_LETHAL`] at its endpoint. Each insert call
//! first clears along every beam and then marks every hit, so the hits of one scan all
//! survive whatever the beam geometry; a later call clears earlier marks it crosses.
//! When the layer is merged into the master, unknown cells are skipped so lower layers
//! stay visible, and observed cells are merged with [`ObstacleConfig::policy`].
//!
//! The crate has no transform support yet: every coordinate passed to the layer
//! must already be in the costmap's world frame. Callers transform sensor data
//! (origin, beam endpoints, sensor pose) before inserting it.

use glam::{UVec2, Vec2};

use crate::{
    Grid2d,
    costmap::{Costmap, Layer, MergePolicy, project_into},
    iterators::line::LineIterator,
    types::{Bounds, COST_FREE, COST_LETHAL, COST_UNKNOWN, CellRegion, MapInfo, Pose2, VoxelError},
};

/// Distance, in cells, that a clipped ray start is kept inside the grid edge.
/// `Grid2d::world_to_map_continuous` rejects the upper edge itself, and the world
/// round trip of a point on that edge can land on either side of it.
const CLIP_MARGIN_CELLS: f32 = 0.01;

/// Configuration for an [`ObstacleLayer`].
#[derive(Debug, Clone, Copy)]
pub struct ObstacleConfig {
    /// Re-centre the layer's grid on the robot each update. Must match the
    /// `rolling_window` setting of the owning [`LayeredCostmap`](crate::LayeredCostmap).
    pub rolling_window: bool,
    /// Hits further than this from the sensor are not marked.
    pub obstacle_range_m: f32,
    /// Clearing stops at this range from the sensor. Hits beyond it are not marked.
    pub raytrace_range_m: f32,
    /// A marked cell reverts to unknown this long after it was last observed.
    /// `None` keeps obstacles until a beam clears them.
    pub decay_s: Option<f32>,
    /// How observed cells merge into the master.
    pub policy: MergePolicy,
}

impl Default for ObstacleConfig {
    fn default() -> Self {
        Self {
            rolling_window: false,
            obstacle_range_m: 2.5,
            raytrace_range_m: 3.0,
            decay_s: None,
            policy: MergePolicy::Max,
        }
    }
}

/// One sensor return, with its endpoint in the costmap's world frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Beam {
    /// World-frame endpoint of the beam.
    pub end: Vec2,
    /// `true` if the beam hit an obstacle at `end`; `false` for a maximum-range
    /// return, which only clears.
    pub hit: bool,
}

/// A layer that builds obstacle and free-space knowledge from sensor beams.
///
/// Beams are inserted with [`insert_beams`](Self::insert_beams) or
/// [`insert_laser_scan`](Self::insert_laser_scan) between updates. In rolling-window
/// mode the grid is re-centred during [`Layer::update_bounds`], so beams inserted
/// before an update are placed on the grid as it was centred at the previous update.
///
/// One insert call clears along all of its beams, then marks all of their hits, so no
/// beam erases a hit of the same call. A later call clears any cell its beams cross,
/// including cells marked by earlier calls.
pub struct ObstacleLayer {
    grid: Costmap,
    /// Time each marked cell was last observed. Allocated only when decay is enabled.
    stamps: Option<Grid2d<Option<f64>>>,
    config: ObstacleConfig,
    /// World area changed since the last `update_costs`; used in fixed mode.
    touched: Bounds,
    /// Hit endpoints collected while clearing, marked once every beam has cleared.
    /// Kept between calls so its allocation is reused.
    pending_hits: Vec<Vec2>,
}

/// One ray to trace: `range` metres from `origin` along the unit vector `dir`.
struct Ray {
    origin: Vec2,
    dir: Vec2,
    range: f32,
    hit: bool,
}

impl ObstacleLayer {
    /// Create a layer whose grid has the layout `info`, with every cell unknown.
    pub fn new(info: MapInfo, config: ObstacleConfig) -> Self {
        Self {
            grid: Grid2d::new_with_value(info, COST_UNKNOWN),
            stamps: config.decay_s.map(|_| Grid2d::new_with_value(info, None)),
            config,
            touched: Bounds::empty(),
            pending_hits: Vec::new(),
        }
    }

    /// The layer's own grid: [`COST_UNKNOWN`] where never observed.
    pub fn grid(&self) -> &Costmap {
        &self.grid
    }

    /// Insert beams that share one sensor `origin`, all in the costmap's world frame.
    ///
    /// Beams with a non-finite or zero-length extent are skipped. `stamp_s` is the
    /// observation time used for decay.
    pub fn insert_beams(
        &mut self,
        origin: Vec2,
        beams: impl IntoIterator<Item = Beam>,
        stamp_s: f64,
    ) {
        let rays = beams.into_iter().map(|beam| {
            let offset = beam.end - origin;
            let range = offset.length();
            Ray {
                origin,
                dir: offset / range,
                range,
                hit: beam.hit,
            }
        });
        self.insert_rays(rays, stamp_s);
    }

    /// Insert a planar laser scan taken by a sensor at `sensor` (world frame).
    ///
    /// `ranges[i]` is measured along `sensor.yaw + angle_min + i * angle_increment`.
    /// NaN, zero and negative ranges are skipped. `+inf` and ranges beyond
    /// `raytrace_range_m` clear up to that range and mark nothing. Ranges beyond
    /// `obstacle_range_m` clear without marking.
    pub fn insert_laser_scan(
        &mut self,
        sensor: Pose2,
        angle_min: f32,
        angle_increment: f32,
        ranges: &[f32],
        stamp_s: f64,
    ) {
        let rays = ranges.iter().enumerate().map(|(i, &range)| {
            let angle = sensor.yaw + angle_min + i as f32 * angle_increment;
            Ray {
                origin: sensor.position,
                dir: Vec2::new(angle.cos(), angle.sin()),
                range,
                hit: true,
            }
        });
        self.insert_rays(rays, stamp_s);
    }

    /// Revert every marked cell last observed more than `decay_s` before `now_s`
    /// to unknown. Does nothing when decay is disabled.
    ///
    /// Time is supplied by the caller, so obstacles expire even when the sensor
    /// stream has stopped. Scans the whole grid.
    pub fn expire(&mut self, now_s: f64) {
        let (Some(decay_s), Some(stamps)) = (self.config.decay_s, self.stamps.as_mut()) else {
            return;
        };
        let resolution = self.grid.info().resolution;
        for (cell, stamp) in stamps.indexed_iter_mut() {
            if stamp.is_some_and(|t| now_s - t > f64::from(decay_s)) {
                *stamp = None;
                let _ = self.grid.set(cell, COST_UNKNOWN);
                touch(&mut self.touched, self.grid.map_to_world(cell), resolution);
            }
        }
    }

    /// Clear along every ray, then mark the hit endpoints, so that a ray cannot erase
    /// a hit of the same call.
    fn insert_rays(&mut self, rays: impl Iterator<Item = Ray>, stamp_s: f64) {
        let mut hits = std::mem::take(&mut self.pending_hits);
        hits.extend(rays.filter_map(|ray| self.clear_ray(&ray)));
        for end in hits.drain(..) {
            self.mark_hit(end, stamp_s);
        }
        self.pending_hits = hits;
    }

    /// Clear the cells `ray` crosses. Returns the endpoint to mark if the ray is a hit
    /// within the obstacle and raytrace ranges.
    fn clear_ray(&mut self, ray: &Ray) -> Option<Vec2> {
        let Ray {
            origin,
            dir,
            range,
            hit,
        } = *ray;
        if range.is_nan() || range <= 0.0 || !origin.is_finite() || !dir.is_finite() {
            return None;
        }
        let mark =
            hit && range <= self.config.obstacle_range_m && range <= self.config.raytrace_range_m;
        let end = origin + dir * range.min(self.config.raytrace_range_m);
        if !end.is_finite() {
            return None;
        }

        self.clear_along(origin, end);
        mark.then_some(end)
    }

    /// Mark the cell containing `end` lethal, if it lies inside the grid.
    fn mark_hit(&mut self, end: Vec2, stamp_s: f64) {
        if let Some(cell) = self.grid.world_to_map(end) {
            self.set_cell(cell, COST_LETHAL, Some(stamp_s));
            touch(&mut self.touched, end, self.grid.info().resolution);
        }
    }

    /// Clear the part of the segment `origin`..`end` that lies inside the grid.
    fn clear_along(&mut self, origin: Vec2, end: Vec2) {
        let info = *self.grid.info();
        let Some((start, exit)) = clip_segment(&info, origin, end) else {
            return;
        };
        touch(&mut self.touched, start, info.resolution);
        touch(&mut self.touched, exit, info.resolution);

        let offset = end - start;
        let Some(cells) = LineIterator::new(&self.grid, start, offset, offset.length()) else {
            return;
        };
        for step in cells {
            self.set_cell(step.cell, COST_FREE, None);
        }
    }

    /// Write `cost` and the observation stamp; `None` drops the stamp.
    fn set_cell(&mut self, cell: UVec2, cost: u8, stamp: Option<f64>) {
        let _ = self.grid.set(cell, cost);
        if let Some(stamps) = self.stamps.as_mut() {
            let _ = stamps.set(cell, stamp);
        }
    }

    /// Move the grid, and the stamps with it, so that it is centred on `center`.
    fn recenter(&mut self, center: Vec2) {
        self.grid.update_center(center);
        if let Some(stamps) = self.stamps.as_mut() {
            stamps.update_center(center);
        }
    }

    /// World extent of the whole grid.
    fn extent(&self) -> Bounds {
        let info = self.grid.info();
        Bounds {
            min: info.origin,
            max: info.origin + Vec2::new(info.world_width(), info.world_height()),
        }
    }
}

/// Grow `bounds` to include the cell-sized neighbourhood of `point`. Padding keeps
/// the bounds of a single point from being empty.
fn touch(bounds: &mut Bounds, point: Vec2, pad: f32) {
    bounds.expand_to_include(point - Vec2::splat(pad));
    bounds.expand_to_include(point + Vec2::splat(pad));
}

/// Clip the segment `origin`..`end` to the grid described by `info`.
///
/// Returns the world-frame entry and exit points, or `None` if the segment misses
/// the grid. The entry point lies at least [`CLIP_MARGIN_CELLS`] inside the grid,
/// which makes it a valid start for [`LineIterator`].
fn clip_segment(info: &MapInfo, origin: Vec2, end: Vec2) -> Option<(Vec2, Vec2)> {
    let a = (origin - info.origin) / info.resolution;
    let delta = (end - info.origin) / info.resolution - a;
    let size = Vec2::new(info.width as f32, info.height as f32);

    let (mut t_in, mut t_out) = (0.0_f32, 1.0_f32);
    for axis in 0..2 {
        if delta[axis] == 0.0 {
            if a[axis] < 0.0 || a[axis] >= size[axis] {
                return None;
            }
            continue;
        }
        let t0 = -a[axis] / delta[axis];
        let t1 = (size[axis] - a[axis]) / delta[axis];
        t_in = t_in.max(t0.min(t1));
        t_out = t_out.min(t0.max(t1));
    }
    if t_in > t_out {
        return None;
    }

    let margin = Vec2::splat(CLIP_MARGIN_CELLS);
    let entry = (a + delta * t_in).clamp(margin, size - margin);
    let exit = a + delta * t_out;
    let to_world = |cell: Vec2| info.origin + cell * info.resolution;
    Some((to_world(entry), to_world(exit)))
}

impl Layer for ObstacleLayer {
    fn reset(&mut self) {
        self.grid.clear();
        if let Some(stamps) = self.stamps.as_mut() {
            stamps.clear();
        }
        self.touched = self.extent();
    }

    fn is_clearable(&self) -> bool {
        true
    }

    fn update_bounds(&mut self, robot: Pose2, bounds: &mut Bounds) {
        if self.config.rolling_window {
            self.recenter(robot.position);
            let extent = self.extent();
            bounds.expand_to_include(extent.min);
            bounds.expand_to_include(extent.max);
        } else if !self.touched.is_empty() {
            bounds.expand_to_include(self.touched.min);
            bounds.expand_to_include(self.touched.max);
        }
    }

    /// Merges observed cells into `master`. Does nothing if the layer's layout
    /// differs from the master's; [`Layer::attach`] rejects such a layer, so this
    /// only happens when `update_costs` is called by hand with another grid.
    fn update_costs(&mut self, master: &mut Costmap, region: CellRegion) {
        self.touched = Bounds::empty();
        if !master.layout_matches(&self.grid) {
            return;
        }
        project_into(master, &self.grid, region, self.config.policy, |&cost| {
            (cost != COST_UNKNOWN).then_some(cost)
        });
    }

    fn attach(&mut self, master: &MapInfo, rolling_window: bool) -> Result<(), VoxelError> {
        if self.config.rolling_window != rolling_window {
            return Err(VoxelError::InvalidArgument(format!(
                "obstacle layer rolling_window is {}, but the costmap's is {rolling_window}",
                self.config.rolling_window
            )));
        }
        let own = self.grid.info();
        let same_cells = own.width == master.width
            && own.height == master.height
            && own.resolution == master.resolution;
        if !same_cells || (!rolling_window && own.origin != master.origin) {
            return Err(VoxelError::InvalidArgument(format!(
                "obstacle layer grid {own:?} does not match the costmap's {master:?}"
            )));
        }
        // Re-centring shifts each grid by whole cells from its own origin, so a
        // rolling layer that starts off the master's cell lattice never lines up
        // with it. Starting from the master's origin keeps the two in step.
        if own.origin != master.origin {
            self.match_size(master);
        }
        Ok(())
    }

    fn match_size(&mut self, info: &MapInfo) {
        let size = UVec2::new(info.width, info.height);
        self.grid.resize_map(size, info.resolution, info.origin);
        if let Some(stamps) = self.stamps.as_mut() {
            stamps.resize_map(size, info.resolution, info.origin);
        }
        self.touched = Bounds::empty();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LayeredCostmap;

    const SIZE: u32 = 20;

    fn info() -> MapInfo {
        MapInfo::square(SIZE, 1.0)
    }

    fn config() -> ObstacleConfig {
        ObstacleConfig {
            obstacle_range_m: 12.0,
            raytrace_range_m: 12.0,
            ..Default::default()
        }
    }

    fn decaying(decay_s: f32) -> ObstacleLayer {
        ObstacleLayer::new(
            info(),
            ObstacleConfig {
                decay_s: Some(decay_s),
                ..config()
            },
        )
    }

    fn cost(layer: &ObstacleLayer, x: u32, y: u32) -> u8 {
        *layer.grid().get(UVec2::new(x, y)).unwrap()
    }

    fn hit(x: f32, y: f32) -> Beam {
        Beam {
            end: Vec2::new(x, y),
            hit: true,
        }
    }

    fn miss(x: f32, y: f32) -> Beam {
        Beam {
            end: Vec2::new(x, y),
            hit: false,
        }
    }

    fn full_region() -> CellRegion {
        CellRegion::new(UVec2::ZERO, UVec2::splat(SIZE))
    }

    fn observed_cells(layer: &ObstacleLayer) -> usize {
        layer.grid().iter().filter(|&&c| c != COST_UNKNOWN).count()
    }

    #[test]
    fn marks_hit_and_clears_path() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        for x in 0..8 {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 9, 5), COST_UNKNOWN);
        assert_eq!(cost(&layer, 4, 6), COST_UNKNOWN);
    }

    #[test]
    fn later_beam_clears_earlier_mark() {
        let mut layer = ObstacleLayer::new(info(), config());
        let origin = Vec2::new(0.5, 5.5);
        layer.insert_beams(origin, [hit(8.5, 5.5)], 0.0);
        layer.insert_beams(origin, [hit(4.5, 5.5)], 1.0);
        assert_eq!(cost(&layer, 4, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);

        layer.insert_beams(origin, [miss(10.5, 5.5)], 2.0);
        assert_eq!(cost(&layer, 4, 5), COST_FREE);
        assert_eq!(cost(&layer, 8, 5), COST_FREE);
    }

    #[test]
    fn hit_cell_survives_its_own_clearing_ray() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(0.5, 0.5), [hit(7.5, 7.5)], 0.0);
        assert_eq!(cost(&layer, 7, 7), COST_LETHAL);
        assert_eq!(cost(&layer, 3, 3), COST_FREE);
    }

    #[test]
    fn hits_of_one_call_survive_beams_that_cross_them() {
        let mut layer = ObstacleLayer::new(info(), config());
        let origin = Vec2::new(0.5, 5.5);
        layer.insert_beams(origin, [hit(4.5, 5.5), hit(8.5, 5.5)], 0.0);
        assert_eq!(cost(&layer, 4, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 3, 5), COST_FREE);
    }

    #[test]
    fn second_scan_clears_a_mark_from_the_first() {
        let mut layer = ObstacleLayer::new(info(), config());
        let sensor = Pose2::new(Vec2::new(0.5, 5.5), 0.0);
        layer.insert_laser_scan(sensor, 0.0, 0.0, &[4.0], 0.0);
        assert_eq!(cost(&layer, 4, 5), COST_LETHAL);

        layer.insert_laser_scan(sensor, 0.0, 0.0, &[8.0], 1.0);
        assert_eq!(cost(&layer, 4, 5), COST_FREE);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
    }

    /// A 1081-beam scan of a straight wall seen at about 14 degrees puts many hits in
    /// cells that neighbouring beams also cross on their way to the wall.
    #[test]
    fn oblique_wall_hits_all_survive_the_same_scan() {
        let resolution = 0.05;
        let grid_info = MapInfo {
            width: 400,
            height: 400,
            resolution,
            origin: Vec2::ZERO,
        };
        let mut layer = ObstacleLayer::new(
            grid_info,
            ObstacleConfig {
                obstacle_range_m: 30.0,
                raytrace_range_m: 30.0,
                ..Default::default()
            },
        );
        let sensor = Vec2::new(10.0, 2.0);
        let (wall_start, wall_end) = (Vec2::new(2.0, 12.0), Vec2::new(18.0, 16.0));
        let (beams, angle_min) = (1081, 0.3_f32);
        let increment = 2.5 / (beams - 1) as f32;

        let wall = wall_end - wall_start;
        let mut ranges = Vec::new();
        let mut hit_cells = std::collections::HashSet::new();
        for i in 0..beams {
            let angle = angle_min + i as f32 * increment;
            let dir = Vec2::new(angle.cos(), angle.sin());
            let to_wall = wall_start - sensor;
            let det = -dir.x * wall.y + dir.y * wall.x;
            let along_beam = (-to_wall.x * wall.y + to_wall.y * wall.x) / det;
            let along_wall = (dir.x * to_wall.y - dir.y * to_wall.x) / det;
            if along_beam > 0.0 && (0.0..=1.0).contains(&along_wall) {
                ranges.push(along_beam);
                let point = sensor + dir * along_beam;
                hit_cells.insert((point / resolution).floor().as_uvec2());
            } else {
                ranges.push(f32::INFINITY);
            }
        }

        layer.insert_laser_scan(Pose2::new(sensor, 0.0), angle_min, increment, &ranges, 0.0);
        let lost: Vec<_> = hit_cells
            .iter()
            .filter(|&&cell| layer.grid()[cell] != COST_LETHAL)
            .collect();
        assert!(hit_cells.len() > 250, "{} hit cells", hit_cells.len());
        assert!(
            lost.is_empty(),
            "{} of {} hits erased",
            lost.len(),
            hit_cells.len()
        );
    }

    #[test]
    fn unknown_cells_leave_master_untouched() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(3.5, 5.5)], 0.0);

        let mut master = Costmap::new_with_value(info(), 77);
        layer.update_costs(&mut master, full_region());

        assert_eq!(master.get(UVec2::new(10, 10)), Some(&77));
        assert_eq!(master.get(UVec2::new(2, 5)), Some(&77));
        assert_eq!(master.get(UVec2::new(3, 5)), Some(&COST_LETHAL));
    }

    #[test]
    fn free_observation_does_not_lower_lethal_master_under_max() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let mut master = Costmap::new_with_value(info(), COST_FREE);
        master.set(UVec2::new(2, 5), COST_LETHAL).unwrap();
        layer.update_costs(&mut master, full_region());

        assert_eq!(master.get(UVec2::new(2, 5)), Some(&COST_LETHAL));
        assert_eq!(master.get(UVec2::new(8, 5)), Some(&COST_LETHAL));
    }

    #[test]
    fn overwrite_policy_lets_free_observation_lower_master() {
        let mut layer = ObstacleLayer::new(
            info(),
            ObstacleConfig {
                policy: MergePolicy::Overwrite,
                ..config()
            },
        );
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let mut master = Costmap::new_with_value(info(), COST_FREE);
        master.set(UVec2::new(2, 5), COST_LETHAL).unwrap();
        layer.update_costs(&mut master, full_region());

        assert_eq!(master.get(UVec2::new(2, 5)), Some(&COST_FREE));
    }

    #[test]
    fn obstacle_range_limits_marking_not_clearing() {
        let mut layer = ObstacleLayer::new(
            info(),
            ObstacleConfig {
                obstacle_range_m: 5.0,
                raytrace_range_m: 8.0,
                ..Default::default()
            },
        );
        let sensor = Pose2::new(Vec2::new(0.5, 10.5), 0.0);

        layer.insert_laser_scan(sensor, 0.0, 0.0, &[4.0], 0.0);
        assert_eq!(cost(&layer, 4, 10), COST_LETHAL);

        layer.insert_laser_scan(sensor, 0.0, 0.0, &[6.0], 0.0);
        assert_eq!(cost(&layer, 4, 10), COST_FREE);
        assert_eq!(cost(&layer, 6, 10), COST_FREE);
        assert_eq!(cost(&layer, 7, 10), COST_UNKNOWN);
        assert_eq!(observed_cells(&layer), 7);
    }

    #[test]
    fn raytrace_range_limits_clearing() {
        let mut layer = ObstacleLayer::new(
            info(),
            ObstacleConfig {
                obstacle_range_m: 12.0,
                raytrace_range_m: 8.0,
                ..Default::default()
            },
        );
        layer.insert_laser_scan(Pose2::new(Vec2::new(0.5, 10.5), 0.0), 0.0, 0.0, &[9.0], 0.0);

        assert_eq!(cost(&layer, 8, 10), COST_FREE);
        assert_eq!(cost(&layer, 9, 10), COST_UNKNOWN);
        assert!(layer.grid().iter().all(|&c| c != COST_LETHAL));
    }

    #[test]
    fn invalid_ranges_are_skipped_and_infinity_only_clears() {
        let mut layer = ObstacleLayer::new(info(), config());
        let centre = Pose2::new(Vec2::new(10.5, 10.5), 0.0);
        let ranges = [f32::NAN, 0.0, -1.0, f32::INFINITY];
        layer.insert_laser_scan(centre, 0.0, std::f32::consts::FRAC_PI_2, &ranges, 0.0);

        // Only the +inf beam, pointing in -y, changed anything: it clears the
        // sensor cell and the ten cells below it, and marks nothing.
        assert_eq!(observed_cells(&layer), 11);
        for y in 0..=10 {
            assert_eq!(cost(&layer, 10, y), COST_FREE, "y={y}");
        }
    }

    #[test]
    fn non_finite_beams_are_ignored() {
        let mut layer = ObstacleLayer::new(info(), config());
        let origin = Vec2::new(5.5, 5.5);
        layer.insert_beams(origin, [hit(f32::NAN, 1.0), hit(f32::INFINITY, 1.0)], 0.0);
        layer.insert_beams(origin, [hit(5.5, 5.5)], 0.0);
        layer.insert_beams(Vec2::NAN, [hit(5.5, 5.5)], 0.0);
        layer.insert_beams(Vec2::INFINITY, [hit(5.5, 5.5)], 0.0);
        assert_eq!(observed_cells(&layer), 0);
    }

    #[test]
    fn origin_outside_grid_still_clears_and_marks_inside() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(-5.5, 5.5), [hit(3.5, 5.5)], 0.0);

        for x in 0..3 {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(cost(&layer, 3, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 4, 5), COST_UNKNOWN);
    }

    #[test]
    fn origin_beyond_upper_edge_enters_grid() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(25.5, 5.5), [hit(15.5, 5.5)], 0.0);

        for x in 16..SIZE {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(cost(&layer, 15, 5), COST_LETHAL);
    }

    #[test]
    fn beam_crossing_grid_with_both_ends_outside_clears_it() {
        let mut layer = ObstacleLayer::new(
            info(),
            ObstacleConfig {
                obstacle_range_m: 50.0,
                raytrace_range_m: 50.0,
                ..Default::default()
            },
        );
        let across = Beam {
            end: Vec2::new(31.5, 5.5),
            hit: true,
        };
        layer.insert_beams(Vec2::new(-10.5, 5.5), [across], 0.0);

        for x in 0..SIZE {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(observed_cells(&layer), SIZE as usize);
    }

    #[test]
    fn beam_missing_grid_changes_nothing() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(-5.5, 30.5), [hit(3.5, 30.5)], 0.0);
        layer.insert_beams(Vec2::new(-5.5, -5.5), [hit(-1.5, 30.5)], 0.0);
        assert_eq!(observed_cells(&layer), 0);
    }

    #[test]
    fn marked_cells_decay_after_stream_stops() {
        let mut layer = decaying(2.0);
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        layer.expire(1.0);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
        layer.expire(2.0);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);

        layer.expire(2.5);
        assert_eq!(cost(&layer, 8, 5), COST_UNKNOWN);
        assert_eq!(cost(&layer, 3, 5), COST_FREE);
    }

    #[test]
    fn re_observation_refreshes_decay() {
        let mut layer = decaying(2.0);
        let origin = Vec2::new(0.5, 5.5);
        layer.insert_beams(origin, [hit(8.5, 5.5)], 0.0);
        layer.insert_beams(origin, [hit(8.5, 5.5)], 1.5);

        layer.expire(2.5);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
        layer.expire(3.6);
        assert_eq!(cost(&layer, 8, 5), COST_UNKNOWN);
    }

    #[test]
    fn cleared_cell_is_not_expired() {
        let mut layer = decaying(2.0);
        let origin = Vec2::new(0.5, 5.5);
        layer.insert_beams(origin, [hit(8.5, 5.5)], 0.0);
        layer.insert_beams(origin, [miss(10.5, 5.5)], 1.0);

        layer.expire(100.0);
        assert_eq!(cost(&layer, 8, 5), COST_FREE);
    }

    #[test]
    fn stamps_exist_only_with_decay() {
        assert!(ObstacleLayer::new(info(), config()).stamps.is_none());
        assert!(decaying(1.0).stamps.is_some());

        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);
        layer.expire(1e9);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
    }

    #[test]
    fn rolling_window_keeps_obstacle_at_world_position() {
        let mut layered = LayeredCostmap::new(info(), COST_FREE, true);
        let id = layered
            .add_layer(Box::new(ObstacleLayer::new(
                info(),
                ObstacleConfig {
                    rolling_window: true,
                    decay_s: Some(10.0),
                    ..config()
                },
            )))
            .unwrap();
        let obstacle = Vec2::new(15.5, 10.5);
        let lethal_at = |layered: &LayeredCostmap, world: Vec2| {
            let cell = layered.master().world_to_map(world);
            cell.and_then(|c| layered.master().get(c).copied())
        };

        layered.update_map(Pose2::new(Vec2::new(10.0, 10.0), 0.0));
        layered
            .layer_mut::<ObstacleLayer>(id)
            .unwrap()
            .insert_beams(Vec2::new(10.5, 10.5), [hit(obstacle.x, obstacle.y)], 0.0);

        for robot in [
            Vec2::new(10.0, 10.0),
            Vec2::new(13.3, 9.7),
            Vec2::new(16.1, 8.2),
        ] {
            layered.update_map(Pose2::new(robot, 0.0));
            assert_eq!(lethal_at(&layered, obstacle), Some(COST_LETHAL), "{robot}");
            let layer = layered.layer::<ObstacleLayer>(id).unwrap();
            assert!(layered.master().layout_matches(layer.grid()), "{robot}");
        }

        layered.update_map(Pose2::new(Vec2::new(60.0, 10.0), 0.0));
        assert_eq!(lethal_at(&layered, obstacle), None);
        let layer = layered.layer::<ObstacleLayer>(id).unwrap();
        assert!(layered.master().layout_matches(layer.grid()));
    }

    #[test]
    fn rolling_layer_off_the_master_lattice_still_marks() {
        let master = MapInfo {
            origin: Vec2::new(-10.5, -10.5),
            ..info()
        };
        let mut layered = LayeredCostmap::new(master, COST_FREE, true);
        let id = layered
            .add_layer(Box::new(ObstacleLayer::new(
                info(),
                ObstacleConfig {
                    rolling_window: true,
                    ..config()
                },
            )))
            .unwrap();
        let robot = Pose2::new(Vec2::new(3.2, 1.7), 0.0);
        layered.update_map(robot);
        let obstacle = Vec2::new(6.2, 1.7);
        layered
            .layer_mut::<ObstacleLayer>(id)
            .unwrap()
            .insert_beams(robot.position, [hit(obstacle.x, obstacle.y)], 0.0);
        layered.update_map(robot);
        let cell = layered.master().world_to_map(obstacle).unwrap();
        assert_eq!(layered.master().get(cell), Some(&COST_LETHAL));
    }

    #[test]
    fn rolling_window_stamps_follow_the_grid() {
        let mut layer = ObstacleLayer::new(
            info(),
            ObstacleConfig {
                rolling_window: true,
                decay_s: Some(2.0),
                ..config()
            },
        );
        layer.insert_beams(Vec2::new(10.5, 10.5), [hit(15.5, 10.5)], 0.0);

        let mut bounds = Bounds::empty();
        layer.update_bounds(Pose2::new(Vec2::new(13.0, 10.0), 0.0), &mut bounds);
        assert_eq!(layer.grid().info().origin, Vec2::new(3.0, 0.0));

        layer.expire(3.0);
        assert!(layer.grid().iter().all(|&c| c != COST_LETHAL));
    }

    #[test]
    fn fixed_bounds_cover_only_touched_area() {
        let big = MapInfo::square(100, 1.0);
        let mut layer = ObstacleLayer::new(big, config());
        let robot = Pose2::new(Vec2::new(80.0, 80.0), 0.0);

        let mut bounds = Bounds::empty();
        layer.update_bounds(robot, &mut bounds);
        assert!(bounds.is_empty());

        layer.insert_beams(Vec2::new(10.5, 10.5), [hit(14.5, 10.5)], 0.0);
        layer.update_bounds(robot, &mut bounds);
        assert!(!bounds.is_empty());
        assert!(bounds.min.x >= 8.0 && bounds.min.y >= 8.0, "{bounds:?}");
        assert!(bounds.max.x <= 17.0 && bounds.max.y <= 13.0, "{bounds:?}");
        assert!(bounds.min.x <= 10.5 && bounds.max.x >= 14.5);

        let mut master = Costmap::new_with_value(big, COST_FREE);
        layer.update_costs(&mut master, CellRegion::new(UVec2::ZERO, UVec2::splat(100)));
        let mut bounds = Bounds::empty();
        layer.update_bounds(robot, &mut bounds);
        assert!(bounds.is_empty());
    }

    #[test]
    fn expiry_is_reported_in_bounds() {
        let big = MapInfo::square(100, 1.0);
        let mut layer = ObstacleLayer::new(
            big,
            ObstacleConfig {
                decay_s: Some(1.0),
                ..config()
            },
        );
        layer.insert_beams(Vec2::new(40.5, 40.5), [hit(44.5, 40.5)], 0.0);
        let mut master = Costmap::new_with_value(big, COST_FREE);
        layer.update_costs(&mut master, CellRegion::new(UVec2::ZERO, UVec2::splat(100)));

        layer.expire(5.0);
        let mut bounds = Bounds::empty();
        layer.update_bounds(Pose2::default(), &mut bounds);
        assert!(bounds.min.x <= 44.5 && bounds.max.x >= 44.5);
        assert!(bounds.min.y <= 40.5 && bounds.max.y >= 40.5);
    }

    #[test]
    fn reset_returns_grid_to_unknown_and_reports_full_extent() {
        let mut layer = decaying(2.0);
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);
        assert!(layer.is_clearable());

        layer.reset();
        assert_eq!(observed_cells(&layer), 0);
        layer.expire(100.0);
        assert_eq!(observed_cells(&layer), 0);

        let mut bounds = Bounds::empty();
        layer.update_bounds(Pose2::default(), &mut bounds);
        assert_eq!(bounds.min, Vec2::ZERO);
        assert_eq!(bounds.max, Vec2::splat(SIZE as f32));
    }

    #[test]
    fn match_size_adopts_master_layout_and_forgets_cells() {
        let mut layer = decaying(2.0);
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let bigger = MapInfo::square(40, 0.5);
        layer.match_size(&bigger);

        assert_eq!(*layer.grid().info(), bigger);
        assert_eq!(observed_cells(&layer), 0);
        assert!(Costmap::new_with_value(bigger, 0).layout_matches(layer.grid()));
    }

    #[test]
    fn mismatched_layout_is_not_projected() {
        let mut layer = ObstacleLayer::new(info(), config());
        layer.insert_beams(Vec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let other = MapInfo::square(SIZE, 0.5);
        let mut master = Costmap::new_with_value(other, 9);
        layer.update_costs(
            &mut master,
            CellRegion::new(UVec2::ZERO, UVec2::splat(SIZE)),
        );
        assert!(master.iter().all(|&c| c == 9));
    }

    #[test]
    fn add_layer_rejects_a_rolling_window_mismatch() {
        let mut layered = LayeredCostmap::new(info(), COST_FREE, true);
        let fixed = ObstacleLayer::new(info(), config());

        assert!(matches!(
            layered.add_layer(Box::new(fixed)),
            Err(VoxelError::InvalidArgument(_))
        ));
    }

    #[test]
    fn add_layer_rejects_a_grid_that_is_not_the_masters() {
        let mut layered = LayeredCostmap::new(info(), COST_FREE, false);
        let finer = ObstacleLayer::new(MapInfo::square(SIZE, 0.5), config());
        let shifted = ObstacleLayer::new(
            MapInfo {
                origin: Vec2::new(3.0, 0.0),
                ..info()
            },
            config(),
        );

        assert!(layered.add_layer(Box::new(finer)).is_err());
        assert!(layered.add_layer(Box::new(shifted)).is_err());
        assert!(
            layered
                .add_layer(Box::new(ObstacleLayer::new(info(), config())))
                .is_ok()
        );
    }
}
