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

use glam::{DVec2, IVec2, Vec2};

use crate::{
    DenseGrid,
    costmap::{Costmap, Layer, MergePolicy, project_into},
    iterators::line::LineIterator,
    types::{
        Bounds, COST_FREE, COST_LETHAL, COST_UNKNOWN, CellRegion, CostmapError, Lattice, Pose2,
    },
};

/// Distance, in cells, that a clipped ray start is kept inside the grid edge.
/// [`LineIterator`] rejects a start on the upper edge itself, and the world round
/// trip of a point on that edge can land on either side of it.
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
    pub end: DVec2,
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
    stamps: Option<DenseGrid<Option<f64>>>,
    config: ObstacleConfig,
    /// World area changed since the last `update_costs`; used in fixed mode.
    touched: Bounds,
    /// Hit endpoints collected while clearing, marked once every beam has cleared.
    /// Kept between calls so its allocation is reused.
    pending_hits: Vec<DVec2>,
}

/// One ray to trace: `range` metres from `origin` along the unit vector `dir`.
struct Ray {
    origin: DVec2,
    dir: Vec2,
    range: f32,
    hit: bool,
}

impl ObstacleLayer {
    /// Create a layer whose grid holds `cells` of `lattice`, every cell unknown.
    pub fn new(lattice: Lattice, cells: CellRegion, config: ObstacleConfig) -> Self {
        Self {
            grid: DenseGrid::filled(lattice, cells, COST_UNKNOWN),
            stamps: config
                .decay_s
                .map(|_| DenseGrid::filled(lattice, cells, None)),
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
        origin: DVec2,
        beams: impl IntoIterator<Item = Beam>,
        stamp_s: f64,
    ) {
        let rays = beams.into_iter().map(|beam| {
            let offset = (beam.end - origin).as_vec2();
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
        let resolution = self.grid.lattice().resolution;
        let lattice = self.grid.lattice();
        for (cell, stamp) in stamps.indexed_iter_mut() {
            if stamp.is_some_and(|t| now_s - t > f64::from(decay_s)) {
                *stamp = None;
                let _ = self.grid.set(cell, COST_UNKNOWN);
                touch(&mut self.touched, lattice.cell_center(cell), resolution);
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
    fn clear_ray(&mut self, ray: &Ray) -> Option<DVec2> {
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
        let end = origin + (dir * range.min(self.config.raytrace_range_m)).as_dvec2();
        if !end.is_finite() {
            return None;
        }

        self.clear_along(origin, end);
        mark.then_some(end)
    }

    /// Mark the cell containing `end` lethal, if it lies inside the grid.
    fn mark_hit(&mut self, end: DVec2, stamp_s: f64) {
        let offset = self.grid.offset_of(self.grid.lattice().cell_at(end));
        if self.set_at(offset, COST_LETHAL, Some(stamp_s)) {
            touch(&mut self.touched, end, self.grid.lattice().resolution);
        }
    }

    /// Clear the part of the segment `origin`..`end` that lies inside the grid.
    fn clear_along(&mut self, origin: DVec2, end: DVec2) {
        let Some((start, exit)) = clip_segment(&self.grid, origin, end) else {
            return;
        };
        let resolution = self.grid.lattice().resolution;
        touch(&mut self.touched, start, resolution);
        touch(&mut self.touched, exit, resolution);

        let offset = (end - start).as_vec2();
        let Some(mut line) = LineIterator::new(&self.grid, start, offset, offset.length()) else {
            return;
        };
        while let Some((offset, _)) = line.next_offset() {
            self.set_at(offset, COST_FREE, None);
        }
    }

    /// Write `cost` and the observation stamp at array position `offset`; `None`
    /// drops the stamp. Returns whether the grid holds that position. The stamps
    /// hold the same cells as the grid, so one offset serves both.
    fn set_at(&mut self, offset: IVec2, cost: u8, stamp: Option<f64>) -> bool {
        let Some(slot) = self.grid.at_mut(offset) else {
            return false;
        };
        *slot = cost;
        if let Some(slot) = self.stamps.as_mut().and_then(|s| s.at_mut(offset)) {
            *slot = stamp;
        }
        true
    }

    /// Move the grid, and the stamps with it, so that it is centred on `center`.
    fn recenter(&mut self, center: DVec2) {
        self.grid.center_on(center);
        if let Some(stamps) = self.stamps.as_mut() {
            stamps.center_on(center);
        }
    }

    /// World extent of the whole grid.
    fn extent(&self) -> Bounds {
        self.grid.world_bounds()
    }
}

/// Grow `bounds` to include the cell-sized neighbourhood of `point`. Padding keeps
/// the bounds of a single point from being empty.
fn touch(bounds: &mut Bounds, point: DVec2, pad: f64) {
    bounds.expand_to_include(point - DVec2::splat(pad));
    bounds.expand_to_include(point + DVec2::splat(pad));
}

/// Clip the segment `origin`..`end` to `grid`, working in
/// cell units from the grid's lower-left corner.
///
/// Returns the world-frame entry and exit points, or `None` if the segment misses
/// the grid. The entry point lies at least [`CLIP_MARGIN_CELLS`] inside the grid,
/// which makes it a valid start for [`LineIterator`].
fn clip_segment<T>(grid: &DenseGrid<T>, origin: DVec2, end: DVec2) -> Option<(DVec2, DVec2)> {
    let corner = grid.world_bounds().min;
    let resolution = grid.lattice().resolution as f32;
    let a = (origin - corner).as_vec2() / resolution;
    let delta = (end - origin).as_vec2() / resolution;
    let size = grid.size().as_vec2();

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
    let to_world = |offsets: Vec2| corner + offsets.as_dvec2() * grid.lattice().resolution;
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

    fn attach(&mut self, master: &Costmap, rolling_window: bool) -> Result<(), CostmapError> {
        if self.config.rolling_window != rolling_window {
            return Err(CostmapError::InvalidArgument(format!(
                "obstacle layer rolling_window is {}, but the costmap's is {rolling_window}",
                self.config.rolling_window
            )));
        }
        let (own, theirs) = (&self.grid, master);
        let same_size =
            own.size() == theirs.size() && own.lattice().resolution == theirs.lattice().resolution;
        if !same_size || (!rolling_window && own.world_bounds() != theirs.world_bounds()) {
            return Err(CostmapError::InvalidArgument(format!(
                "obstacle layer grid {:?} on {:?} does not match the costmap's {:?} on {:?}",
                own.cells(),
                own.lattice(),
                theirs.cells(),
                theirs.lattice()
            )));
        }
        // The layer's cells must be the master's cells: a rolling grid moves by whole
        // cells of its own lattice, so a layer on another lattice never lines up.
        if !self.grid.layout_matches(master) {
            let (lattice, cells) = (master.lattice(), master.cells());
            self.grid = DenseGrid::filled(lattice, cells, COST_UNKNOWN);
            if let Some(stamps) = self.stamps.as_mut() {
                *stamps = DenseGrid::filled(lattice, cells, None);
            }
            self.touched = Bounds::empty();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LayeredCostmap;
    use crate::types::{CellRegion, Lattice};
    use glam::UVec2;

    const SIZE: u32 = 20;

    fn lattice() -> Lattice {
        Lattice::new(DVec2::ZERO, 1.0)
    }

    fn cells() -> CellRegion {
        CellRegion::from_size(UVec2::splat(SIZE))
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
            lattice(),
            cells(),
            ObstacleConfig {
                decay_s: Some(decay_s),
                ..config()
            },
        )
    }

    fn cost(layer: &ObstacleLayer, x: i32, y: i32) -> u8 {
        *layer.grid().get(IVec2::new(x, y)).unwrap()
    }

    fn hit(x: f64, y: f64) -> Beam {
        Beam {
            end: DVec2::new(x, y),
            hit: true,
        }
    }

    fn miss(x: f64, y: f64) -> Beam {
        Beam {
            end: DVec2::new(x, y),
            hit: false,
        }
    }

    fn full_region() -> CellRegion {
        CellRegion::new(IVec2::ZERO, IVec2::splat(SIZE as i32))
    }

    fn observed_cells(layer: &ObstacleLayer) -> usize {
        layer
            .grid()
            .as_slice()
            .iter()
            .filter(|&&c| c != COST_UNKNOWN)
            .count()
    }

    #[test]
    fn marks_hit_and_clears_path() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        for x in 0..8 {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 9, 5), COST_UNKNOWN);
        assert_eq!(cost(&layer, 4, 6), COST_UNKNOWN);
    }

    #[test]
    fn later_beam_clears_earlier_mark() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        let origin = DVec2::new(0.5, 5.5);
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
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(0.5, 0.5), [hit(7.5, 7.5)], 0.0);
        assert_eq!(cost(&layer, 7, 7), COST_LETHAL);
        assert_eq!(cost(&layer, 3, 3), COST_FREE);
    }

    #[test]
    fn hits_of_one_call_survive_beams_that_cross_them() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        let origin = DVec2::new(0.5, 5.5);
        layer.insert_beams(origin, [hit(4.5, 5.5), hit(8.5, 5.5)], 0.0);
        assert_eq!(cost(&layer, 4, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 3, 5), COST_FREE);
    }

    #[test]
    fn second_scan_clears_a_mark_from_the_first() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        let sensor = Pose2::new(DVec2::new(0.5, 5.5), 0.0);
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
        let grid_lattice = Lattice::new(DVec2::ZERO, resolution);
        let grid_cells = CellRegion::from_size(UVec2::splat(400));
        let mut layer = ObstacleLayer::new(
            grid_lattice,
            grid_cells,
            ObstacleConfig {
                obstacle_range_m: 30.0,
                raytrace_range_m: 30.0,
                ..Default::default()
            },
        );
        let sensor = DVec2::new(10.0, 2.0);
        let (wall_start, wall_end) = (DVec2::new(2.0, 12.0), DVec2::new(18.0, 16.0));
        let (beams, angle_min) = (1081, 0.3_f32);
        let increment = 2.5 / (beams - 1) as f32;

        let wall = wall_end - wall_start;
        let mut ranges = Vec::new();
        let mut hit_cells = std::collections::HashSet::new();
        for i in 0..beams {
            let angle = angle_min + i as f32 * increment;
            let dir = Vec2::new(angle.cos(), angle.sin()).as_dvec2();
            let to_wall = wall_start - sensor;
            let det = -dir.x * wall.y + dir.y * wall.x;
            let along_beam = (-to_wall.x * wall.y + to_wall.y * wall.x) / det;
            let along_wall = (dir.x * to_wall.y - dir.y * to_wall.x) / det;
            if along_beam > 0.0 && (0.0..=1.0).contains(&along_wall) {
                ranges.push(along_beam as f32);
                let point = sensor + dir * along_beam;
                hit_cells.insert((point / resolution).floor().as_ivec2());
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
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(3.5, 5.5)], 0.0);

        let mut master = Costmap::filled(lattice(), cells(), 77);
        layer.update_costs(&mut master, full_region());

        assert_eq!(master.get(IVec2::new(10, 10)), Some(&77));
        assert_eq!(master.get(IVec2::new(2, 5)), Some(&77));
        assert_eq!(master.get(IVec2::new(3, 5)), Some(&COST_LETHAL));
    }

    #[test]
    fn free_observation_does_not_lower_lethal_master_under_max() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let mut master = Costmap::filled(lattice(), cells(), COST_FREE);
        master.set(IVec2::new(2, 5), COST_LETHAL).unwrap();
        layer.update_costs(&mut master, full_region());

        assert_eq!(master.get(IVec2::new(2, 5)), Some(&COST_LETHAL));
        assert_eq!(master.get(IVec2::new(8, 5)), Some(&COST_LETHAL));
    }

    #[test]
    fn overwrite_policy_lets_free_observation_lower_master() {
        let mut layer = ObstacleLayer::new(
            lattice(),
            cells(),
            ObstacleConfig {
                policy: MergePolicy::Overwrite,
                ..config()
            },
        );
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let mut master = Costmap::filled(lattice(), cells(), COST_FREE);
        master.set(IVec2::new(2, 5), COST_LETHAL).unwrap();
        layer.update_costs(&mut master, full_region());

        assert_eq!(master.get(IVec2::new(2, 5)), Some(&COST_FREE));
    }

    #[test]
    fn obstacle_range_limits_marking_not_clearing() {
        let mut layer = ObstacleLayer::new(
            lattice(),
            cells(),
            ObstacleConfig {
                obstacle_range_m: 5.0,
                raytrace_range_m: 8.0,
                ..Default::default()
            },
        );
        let sensor = Pose2::new(DVec2::new(0.5, 10.5), 0.0);

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
            lattice(),
            cells(),
            ObstacleConfig {
                obstacle_range_m: 12.0,
                raytrace_range_m: 8.0,
                ..Default::default()
            },
        );
        layer.insert_laser_scan(
            Pose2::new(DVec2::new(0.5, 10.5), 0.0),
            0.0,
            0.0,
            &[9.0],
            0.0,
        );

        assert_eq!(cost(&layer, 8, 10), COST_FREE);
        assert_eq!(cost(&layer, 9, 10), COST_UNKNOWN);
        assert!(layer.grid().as_slice().iter().all(|&c| c != COST_LETHAL));
    }

    #[test]
    fn invalid_ranges_are_skipped_and_infinity_only_clears() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        let centre = Pose2::new(DVec2::new(10.5, 10.5), 0.0);
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
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        let origin = DVec2::new(5.5, 5.5);
        layer.insert_beams(origin, [hit(f64::NAN, 1.0), hit(f64::INFINITY, 1.0)], 0.0);
        layer.insert_beams(origin, [hit(5.5, 5.5)], 0.0);
        layer.insert_beams(DVec2::NAN, [hit(5.5, 5.5)], 0.0);
        layer.insert_beams(DVec2::INFINITY, [hit(5.5, 5.5)], 0.0);
        assert_eq!(observed_cells(&layer), 0);
    }

    #[test]
    fn origin_outside_grid_still_clears_and_marks_inside() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(-5.5, 5.5), [hit(3.5, 5.5)], 0.0);

        for x in 0..3 {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(cost(&layer, 3, 5), COST_LETHAL);
        assert_eq!(cost(&layer, 4, 5), COST_UNKNOWN);
    }

    #[test]
    fn origin_beyond_upper_edge_enters_grid() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(25.5, 5.5), [hit(15.5, 5.5)], 0.0);

        for x in 16..SIZE as i32 {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(cost(&layer, 15, 5), COST_LETHAL);
    }

    #[test]
    fn beam_crossing_grid_with_both_ends_outside_clears_it() {
        let mut layer = ObstacleLayer::new(
            lattice(),
            cells(),
            ObstacleConfig {
                obstacle_range_m: 50.0,
                raytrace_range_m: 50.0,
                ..Default::default()
            },
        );
        let across = Beam {
            end: DVec2::new(31.5, 5.5),
            hit: true,
        };
        layer.insert_beams(DVec2::new(-10.5, 5.5), [across], 0.0);

        for x in 0..SIZE as i32 {
            assert_eq!(cost(&layer, x, 5), COST_FREE, "x={x}");
        }
        assert_eq!(observed_cells(&layer), SIZE as usize);
    }

    #[test]
    fn beam_missing_grid_changes_nothing() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(-5.5, 30.5), [hit(3.5, 30.5)], 0.0);
        layer.insert_beams(DVec2::new(-5.5, -5.5), [hit(-1.5, 30.5)], 0.0);
        assert_eq!(observed_cells(&layer), 0);
    }

    #[test]
    fn marked_cells_decay_after_stream_stops() {
        let mut layer = decaying(2.0);
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

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
        let origin = DVec2::new(0.5, 5.5);
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
        let origin = DVec2::new(0.5, 5.5);
        layer.insert_beams(origin, [hit(8.5, 5.5)], 0.0);
        layer.insert_beams(origin, [miss(10.5, 5.5)], 1.0);

        layer.expire(100.0);
        assert_eq!(cost(&layer, 8, 5), COST_FREE);
    }

    #[test]
    fn stamps_exist_only_with_decay() {
        assert!(
            ObstacleLayer::new(lattice(), cells(), config())
                .stamps
                .is_none()
        );
        assert!(decaying(1.0).stamps.is_some());

        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);
        layer.expire(1e9);
        assert_eq!(cost(&layer, 8, 5), COST_LETHAL);
    }

    #[test]
    fn rolling_window_keeps_obstacle_at_world_position() {
        let mut layered = LayeredCostmap::new(lattice(), cells(), COST_FREE, true);
        let id = layered
            .add_layer(Box::new(ObstacleLayer::new(
                lattice(),
                cells(),
                ObstacleConfig {
                    rolling_window: true,
                    decay_s: Some(10.0),
                    ..config()
                },
            )))
            .unwrap();
        let obstacle = DVec2::new(15.5, 10.5);
        let lethal_at = |layered: &LayeredCostmap, world: DVec2| {
            let cell = layered.master().lattice().cell_at(world);
            layered.master().get(cell).copied()
        };

        layered.update_map(Pose2::new(DVec2::new(10.0, 10.0), 0.0));
        layered
            .layer_mut::<ObstacleLayer>(id)
            .unwrap()
            .insert_beams(DVec2::new(10.5, 10.5), [hit(obstacle.x, obstacle.y)], 0.0);

        for robot in [
            DVec2::new(10.0, 10.0),
            DVec2::new(13.3, 9.7),
            DVec2::new(16.1, 8.2),
        ] {
            layered.update_map(Pose2::new(robot, 0.0));
            assert_eq!(lethal_at(&layered, obstacle), Some(COST_LETHAL), "{robot}");
            let layer = layered.layer::<ObstacleLayer>(id).unwrap();
            assert!(layered.master().layout_matches(layer.grid()), "{robot}");
        }

        layered.update_map(Pose2::new(DVec2::new(60.0, 10.0), 0.0));
        assert_eq!(lethal_at(&layered, obstacle), None);
        let layer = layered.layer::<ObstacleLayer>(id).unwrap();
        assert!(layered.master().layout_matches(layer.grid()));
    }

    #[test]
    fn rolling_layer_off_the_master_lattice_still_marks() {
        let master = Lattice::new(DVec2::new(-10.5, -10.5), 1.0);
        let mut layered = LayeredCostmap::new(master, cells(), COST_FREE, true);
        let id = layered
            .add_layer(Box::new(ObstacleLayer::new(
                lattice(),
                cells(),
                ObstacleConfig {
                    rolling_window: true,
                    ..config()
                },
            )))
            .unwrap();
        let robot = Pose2::new(DVec2::new(3.2, 1.7), 0.0);
        layered.update_map(robot);
        let obstacle = DVec2::new(6.2, 1.7);
        layered
            .layer_mut::<ObstacleLayer>(id)
            .unwrap()
            .insert_beams(robot.position, [hit(obstacle.x, obstacle.y)], 0.0);
        layered.update_map(robot);
        let cell = layered.master().lattice().cell_at(obstacle);
        assert_eq!(layered.master().get(cell), Some(&COST_LETHAL));
    }

    #[test]
    fn rolling_window_stamps_follow_the_grid() {
        let mut layer = ObstacleLayer::new(
            lattice(),
            cells(),
            ObstacleConfig {
                rolling_window: true,
                decay_s: Some(2.0),
                ..config()
            },
        );
        layer.insert_beams(DVec2::new(10.5, 10.5), [hit(15.5, 10.5)], 0.0);

        let mut bounds = Bounds::empty();
        layer.update_bounds(Pose2::new(DVec2::new(13.0, 10.0), 0.0), &mut bounds);
        assert_eq!(layer.grid().world_bounds().min, DVec2::new(3.0, 0.0));

        layer.expire(3.0);
        assert!(layer.grid().as_slice().iter().all(|&c| c != COST_LETHAL));
    }

    #[test]
    fn fixed_bounds_cover_only_touched_area() {
        let big_lattice = Lattice::new(DVec2::ZERO, 1.0);
        let big_cells = CellRegion::from_size(UVec2::splat(100));
        let mut layer = ObstacleLayer::new(big_lattice, big_cells, config());
        let robot = Pose2::new(DVec2::new(80.0, 80.0), 0.0);

        let mut bounds = Bounds::empty();
        layer.update_bounds(robot, &mut bounds);
        assert!(bounds.is_empty());

        layer.insert_beams(DVec2::new(10.5, 10.5), [hit(14.5, 10.5)], 0.0);
        layer.update_bounds(robot, &mut bounds);
        assert!(!bounds.is_empty());
        assert!(bounds.min.x >= 8.0 && bounds.min.y >= 8.0, "{bounds:?}");
        assert!(bounds.max.x <= 17.0 && bounds.max.y <= 13.0, "{bounds:?}");
        assert!(bounds.min.x <= 10.5 && bounds.max.x >= 14.5);

        let mut master = Costmap::filled(big_lattice, big_cells, COST_FREE);
        layer.update_costs(&mut master, CellRegion::new(IVec2::ZERO, IVec2::splat(100)));
        let mut bounds = Bounds::empty();
        layer.update_bounds(robot, &mut bounds);
        assert!(bounds.is_empty());
    }

    #[test]
    fn expiry_is_reported_in_bounds() {
        let big_lattice = Lattice::new(DVec2::ZERO, 1.0);
        let big_cells = CellRegion::from_size(UVec2::splat(100));
        let mut layer = ObstacleLayer::new(
            big_lattice,
            big_cells,
            ObstacleConfig {
                decay_s: Some(1.0),
                ..config()
            },
        );
        layer.insert_beams(DVec2::new(40.5, 40.5), [hit(44.5, 40.5)], 0.0);
        let mut master = Costmap::filled(big_lattice, big_cells, COST_FREE);
        layer.update_costs(&mut master, CellRegion::new(IVec2::ZERO, IVec2::splat(100)));

        layer.expire(5.0);
        let mut bounds = Bounds::empty();
        layer.update_bounds(Pose2::default(), &mut bounds);
        assert!(bounds.min.x <= 44.5 && bounds.max.x >= 44.5);
        assert!(bounds.min.y <= 40.5 && bounds.max.y >= 40.5);
    }

    #[test]
    fn reset_returns_grid_to_unknown_and_reports_full_extent() {
        let mut layer = decaying(2.0);
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);
        assert!(layer.is_clearable());

        layer.reset();
        assert_eq!(observed_cells(&layer), 0);
        layer.expire(100.0);
        assert_eq!(observed_cells(&layer), 0);

        let mut bounds = Bounds::empty();
        layer.update_bounds(Pose2::default(), &mut bounds);
        assert_eq!(bounds.min, DVec2::ZERO);
        assert_eq!(bounds.max, DVec2::splat(SIZE as f64));
    }

    #[test]
    fn rolling_layer_adopts_the_master_cells_and_forgets_its_own() {
        let mut layer = ObstacleLayer::new(
            lattice(),
            cells(),
            ObstacleConfig {
                rolling_window: true,
                decay_s: Some(2.0),
                ..config()
            },
        );
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let mut master = Costmap::filled(lattice(), cells(), 0);
        master.move_to(DVec2::new(3.0, -2.0));
        layer.attach(&master, true).unwrap();

        assert_eq!(layer.grid().cells(), master.cells());
        assert!(master.layout_matches(layer.grid()));
        assert_eq!(observed_cells(&layer), 0);
    }

    #[test]
    fn mismatched_layout_is_not_projected() {
        let mut layer = ObstacleLayer::new(lattice(), cells(), config());
        layer.insert_beams(DVec2::new(0.5, 5.5), [hit(8.5, 5.5)], 0.0);

        let mut master = Costmap::filled(Lattice::new(DVec2::ZERO, 0.5), cells(), 9);
        layer.update_costs(
            &mut master,
            CellRegion::new(IVec2::ZERO, IVec2::splat(SIZE as i32)),
        );
        assert!(master.as_slice().iter().all(|&c| c == 9));
    }

    #[test]
    fn add_layer_rejects_a_rolling_window_mismatch() {
        let mut layered = LayeredCostmap::new(lattice(), cells(), COST_FREE, true);
        let fixed = ObstacleLayer::new(lattice(), cells(), config());

        assert!(matches!(
            layered.add_layer(Box::new(fixed)),
            Err(CostmapError::InvalidArgument(_))
        ));
    }

    #[test]
    fn add_layer_rejects_a_grid_that_is_not_the_masters() {
        let mut layered = LayeredCostmap::new(lattice(), cells(), COST_FREE, false);
        let finer = ObstacleLayer::new(Lattice::new(DVec2::ZERO, 0.5), cells(), config());
        let shifted =
            ObstacleLayer::new(Lattice::new(DVec2::new(3.0, 0.0), 1.0), cells(), config());

        assert!(layered.add_layer(Box::new(finer)).is_err());
        assert!(layered.add_layer(Box::new(shifted)).is_err());
        assert!(
            layered
                .add_layer(Box::new(ObstacleLayer::new(lattice(), cells(), config())))
                .is_ok()
        );
    }
}
