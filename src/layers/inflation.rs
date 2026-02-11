//! Obstacle inflation for `Grid2d<u8>` costmaps.
//!
//! Provides inflate operations that expand lethal cells outward with inscribed
//! exponential decay. Uses wavefront propagation for performance.
//!
//! Uses a **wavefront propagation** algorithm: obstacle seeds are processed in
//! distance order, each cell is visited once, and costs are looked up from a
//! precomputed cache. This is significantly faster than stamping disks for
//! dense obstacle maps.
//!
//! [`InflationLayer`] implements the layered costmap [`Layer`](crate::grid::Layer)
//! trait so it can be used inside [`LayeredGrid2d`](crate::grid::LayeredGrid2d).

use glam::{IVec2, UVec2};

use crate::Grid2d;
use crate::grid::{Bounds, CellRegion, Layer, Pose2};
use crate::types::{COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN, MapInfo};

/// Epsilon for comparing resolution (f32) when deciding whether to rebuild the cache.
const RESOLUTION_EPSILON: f32 = 1e-9;

/// Padding added to the cache so that it contains more of the exponential decay curve.
const CACHE_PADDING: u32 = 3;

/// Cell data for wavefront propagation: cell position and the obstacle source
/// it was enqueued from (for cost lookup).
#[derive(Clone, Copy)]
struct CellData {
    pos: UVec2,
    src: UVec2,
}

/// Convert an inflation radius in world units (meters) to a cell count.
///
/// Returns `ceil(radius / resolution)`. If resolution is zero or negative, or
/// if the result would be non-positive, returns 0.
#[inline]
pub fn inflation_radius_to_cells(radius_m: f32, resolution: f32) -> u32 {
    if resolution <= 0.0 || radius_m <= 0.0 {
        return 0;
    }
    (radius_m / resolution).ceil() as u32
}

/// Compute an inflation cost using exponential decay with an inscribed radius.
///
/// The cost curve is:
///   - Distance ≤ 0        → `COST_LETHAL` (254)
///   - Distance ≤ inscribed → `COST_INSCRIBED` (253)
///   - Otherwise            → `exp(-cost_scaling_factor * (distance - inscribed_radius)) * (COST_INSCRIBED - 1)`
///
/// Distance and inscribed_radius can be in any consistent units (cells or meters).
/// For world units, pass distances in meters and use `cost_scaling_factor`; for
/// cell units pass `cost_scaling_factor * resolution`.
pub fn inscribed_inflation_cost(
    distance: f32,
    inscribed_radius: f32,
    cost_scaling_factor: f32,
) -> u8 {
    if distance <= 0.0 {
        return COST_LETHAL;
    }
    if distance <= inscribed_radius {
        return COST_INSCRIBED;
    }
    let value = ((-cost_scaling_factor * (distance - inscribed_radius)).exp()
        * (COST_INSCRIBED as f32 - 1.0))
        .round() as u8;
    value.max(COST_FREE)
}

/// Configuration for the inflation layer.
///
/// Groups parameters that control inflation behaviour. Defaults match Nav2.
#[derive(Debug, Clone)]
pub struct InflationConfig {
    /// When false, `update_costs` returns immediately without inflating.
    pub enabled: bool,
    /// When true, allow overwriting unknown cells with inflation cost if cost > FREE.
    pub inflate_unknown: bool,
    /// When true, treat COST_UNKNOWN cells as obstacle seeds (same as lethal).
    pub inflate_around_unknown: bool,
    /// Inflation radius in meters.
    pub inflation_radius_m: f32,
    /// Cost scaling factor for exponential decay (world units).
    pub cost_scaling_factor: f32,
    /// Inscribed radius in meters (inside this, cost is COST_INSCRIBED).
    pub inscribed_radius_m: f32,
}

impl Default for InflationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            inflate_unknown: false,
            inflate_around_unknown: false,
            inflation_radius_m: 0.55,
            cost_scaling_factor: 10.0,
            inscribed_radius_m: 0.0,
        }
    }
}

/// Precomputed cost, distance, and integer-level cache for wavefront inflation.
///
/// Stores costs and distances for every (dx, dy) offset in the inflation disk
/// and a level matrix for wavefront ordering. Uses symmetry for negative coords.
/// Recompute when radius, resolution (for world-unit layer), or inscribed/scaling change.
pub struct InflationCache {
    /// Radius to inflate around a cell in world units
    radius: f32,
    /// Side length of the cache (width/height)
    side: u32,
    /// Cached costs for each positive position in the cache
    costs: Vec<u8>,
    /// Cached world distances for each position in the cache
    distances: Vec<f32>,
    /// Integer distance levels for wavefront
    distance_levels: Vec<u32>,
    /// Side length of the distance-level matrix (2*r + 1)
    level_side: u32,
    max_level: u32,
}

impl std::fmt::Debug for InflationCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut costs = String::new();
        for (i, cost) in self.costs.iter().enumerate() {
            if i % self.side as usize == 0 {
                costs.push('\n');
            }
            costs.push_str(&format!("{:3} ", cost));
        }
        costs.push('\n');
        let mut distances = String::new();
        for (i, distance) in self.distances.iter().enumerate() {
            if i % self.side as usize == 0 {
                distances.push('\n');
            }
            distances.push_str(&format!("{:2.6} ", distance));
        }
        distances.push('\n');
        let mut distance_levels = String::new();
        for (i, distance_level) in self.distance_levels.iter().enumerate() {
            if i % self.level_side as usize == 0 {
                distance_levels.push('\n');
            }
            distance_levels.push_str(&format!("{:2} ", distance_level));
        }
        distance_levels.push('\n');
        write!(
            f,
            "InflationCache {{ radius: {}, side: {}, level_side: {}, max_level: {}\ncosts: {}\ndistances: {}\ndistance_levels: {} }}",
            self.radius,
            self.side,
            self.level_side,
            self.max_level,
            costs,
            distances,
            distance_levels,
        )
    }
}

impl InflationCache {
    /// Build the cache for inscribed cost.
    /// Distance lookup returns world distance (for wavefront radius check); costs use world units.
    pub fn build(
        radius_m: f32,
        inscribed_radius_m: f32,
        resolution: f32,
        cost_scaling_factor: f32,
    ) -> Self {
        let radius_cells = inflation_radius_to_cells(radius_m, resolution);

        let padded_radius = radius_cells + CACHE_PADDING;
        let cost_fn =
            |dist_m: f32| inscribed_inflation_cost(dist_m, inscribed_radius_m, cost_scaling_factor);
        let capacity = (padded_radius * padded_radius) as usize;
        let mut costs = Vec::with_capacity(capacity);
        let mut distances = Vec::with_capacity(capacity);
        for dy in 0..padded_radius {
            for dx in 0..padded_radius {
                let dist_m = ((dx * dx + dy * dy) as f32).sqrt() * resolution;
                distances.push(dist_m);
                costs.push(cost_fn(dist_m));
            }
        }
        let (distance_levels, level_side, max_level) =
            Self::generate_integer_distances(padded_radius);

        Self {
            radius: radius_m,
            side: padded_radius,
            costs,
            distances,
            distance_levels,
            level_side,
            max_level,
        }
    }

    fn generate_integer_distances(radius: u32) -> (Vec<u32>, u32, u32) {
        let r = radius as i32;

        let mut points: Vec<IVec2> = Vec::new();
        for y in -r..=r {
            for x in -r..=r {
                if x * x + y * y <= r * r {
                    points.push(IVec2::new(x, y));
                }
            }
        }

        points.sort_by_key(|p| p.length_squared());

        let level_side = (2 * r + 1) as u32;
        let size = level_side as usize;
        let mut distance_matrix = vec![0u32; size * size];
        let mut last_dist_sq = -1i32;
        let mut level = 0u32;
        for p in points {
            let dist_sq = p.length_squared();
            if dist_sq != last_dist_sq {
                level += 1;
                last_dist_sq = dist_sq;
            }
            let i = (p + r).as_usizevec2();
            distance_matrix[i.y * size + i.x] = level;
        }

        (distance_matrix, level_side, level)
    }

    /// Lookup the cost for a given distance delta.
    #[inline]
    pub fn cost_lookup(&self, delta: IVec2) -> u8 {
        let a = delta.abs().as_uvec2();
        if a.x >= self.side || a.y >= self.side {
            return COST_FREE;
        }
        self.costs[(a.y * self.side + a.x) as usize]
    }

    #[inline]
    pub fn distance_lookup(&self, delta: IVec2) -> f32 {
        let a = delta.abs().as_uvec2();
        if a.x >= self.side || a.y >= self.side {
            return f32::MAX;
        }
        self.distances[(a.y * self.side + a.x) as usize]
    }

    /// Lookup the distance level for a given distance delta.
    ///
    /// If the distance delta is outside the cache, returns the max level + 1.
    #[inline]
    fn dist_level(&self, delta: IVec2) -> u32 {
        let r = self.side as i32;
        let shifted = delta + r;
        let i = shifted.as_uvec2();
        if i.x >= self.level_side || i.y >= self.level_side {
            return self.max_level + 1;
        }
        self.distance_levels[(i.y * self.level_side + i.x) as usize]
    }

    #[inline]
    pub fn cell_inflation_radius(&self) -> u32 {
        self.radius.ceil() as u32
    }

    #[inline]
    fn max_level(&self) -> u32 {
        self.max_level
    }

    #[inline]
    fn radius_f(&self) -> f32 {
        self.radius
    }
}

/// Cardinal neighbour offsets for 4-connected wavefront propagation.
const CARDINAL_OFFSETS: [IVec2; 4] = [
    IVec2::new(-1, 0),
    IVec2::new(1, 0),
    IVec2::new(0, -1),
    IVec2::new(0, 1),
];

/// Options for inflation write and seed behaviour.
#[derive(Clone, Copy, Default)]
pub struct InflateOptions {
    /// When true, allow overwriting unknown cells with inflation cost if cost > FREE.
    pub inflate_unknown: bool,
    /// When true, treat COST_UNKNOWN cells as obstacle seeds (same as lethal).
    pub inflate_around_unknown: bool,
}

/// Mutable state used when enqueueing neighbours in the wavefront. Keeps
/// `try_enqueue_neighbour` to three arguments instead of many.
struct EnqueueContext<'a> {
    cache: &'a InflationCache,
    inflation_cells: &'a mut [Vec<CellData>],
    seen: &'a mut [bool],
    width: usize,
    size: usize,
    radius_f: f32,
    max_level: u32,
}

impl EnqueueContext<'_> {
    #[inline]
    fn enqueue_neighbour(&mut self, neighbour: UVec2, src: UVec2) {
        if self.is_seen(neighbour) {
            return;
        }
        let delta = neighbour.as_ivec2() - src.as_ivec2();
        let dist = self.cache.distance_lookup(delta);
        if dist > self.radius_f {
            return;
        }
        let lvl = self.cache.dist_level(delta);
        if lvl <= self.max_level {
            self.inflation_cells[lvl as usize].push(CellData {
                pos: neighbour,
                src,
            });
        }
    }

    fn is_seen(&self, pos: UVec2) -> bool {
        let idx = pos.as_usizevec2();
        let index = idx.y * self.width + idx.x;
        index >= self.size || self.seen[index]
    }

    /// Mark a cell as seen, returning true if it was not already seen.
    fn mark_seen(&mut self, pos: UVec2) -> bool {
        let idx = pos.as_usizevec2();
        let index = idx.y * self.width + idx.x;
        if index >= self.size || self.seen[index] {
            return false;
        }
        self.seen[index] = true;
        true
    }
}

impl Grid2d<u8> {
    /// Inflate lethal obstacles in place using inscribed exponential decay.
    ///
    /// All parameters are in meters. `inscribed_radius_m` is the radius within which
    /// cost stays `COST_INSCRIBED`; `cost_scaling_factor` controls the decay beyond that.
    pub fn inflate_inscribed(
        &mut self,
        radius_m: f32,
        inscribed_radius_m: f32,
        cost_scaling_factor: f32,
    ) {
        let resolution = self.info().resolution;
        let cache = InflationCache::build(
            radius_m,
            inscribed_radius_m,
            resolution,
            cost_scaling_factor,
        );
        let region = CellRegion {
            min: UVec2::ZERO,
            max: UVec2::new(self.width(), self.height()),
        };
        let mut seen = Vec::new();
        self.inflate_wavefront(region, &cache, InflateOptions::default(), &mut seen);
    }

    /// Inflate lethal (and optionally unknown) cells in place using wavefront propagation.
    ///
    /// `seen` is a reusable buffer; it is resized to `width * height` if needed and cleared on use.
    fn inflate_wavefront(
        &mut self,
        region: CellRegion,
        cache: &InflationCache,
        options: InflateOptions,
        seen: &mut Vec<bool>,
    ) {
        let size = region.max.saturating_sub(region.min);
        if size.x == 0 || size.y == 0 {
            return;
        }

        let radius_cells = cache.cell_inflation_radius();
        let width = self.width();
        let height = self.height();
        let size = (width * height) as usize;
        let radius_f = cache.radius_f();
        let max_level = cache.max_level();

        if seen.len() != size {
            seen.resize(size, false);
        } else {
            seen.fill(false);
        }

        let search_min = region.min.saturating_sub(UVec2::splat(radius_cells));
        let search_max = (region.max + UVec2::splat(radius_cells)).min(UVec2::new(width, height));

        let mut seeds = Vec::new();
        for y in search_min.y..search_max.y {
            for x in search_min.x..search_max.x {
                let pos = UVec2::new(x, y);
                let cost = self.get(pos).copied().unwrap_or(COST_FREE);
                if cost == COST_LETHAL {
                    seeds.push(CellData { pos, src: pos });
                } else if options.inflate_around_unknown && cost == COST_UNKNOWN {
                    seeds.push(CellData { pos, src: pos });
                }
            }
        }

        if seeds.is_empty() {
            return;
        }

        let mut inflation_cells: Vec<Vec<CellData>> = vec![Vec::new(); (max_level + 1) as usize];
        inflation_cells[0] = seeds;

        let mut enqueue_ctx = EnqueueContext {
            cache,
            inflation_cells: &mut inflation_cells,
            seen: seen.as_mut_slice(),
            width: width as usize,
            size,
            radius_f,
            max_level,
        };

        for level in 0..=max_level {
            let level_usize = level as usize;
            let mut i = 0;
            while i < enqueue_ctx.inflation_cells[level_usize].len() {
                let cell = enqueue_ctx.inflation_cells[level_usize][i];
                i += 1;

                if !enqueue_ctx.mark_seen(cell.pos) {
                    continue;
                }

                if region.contains(cell.pos) {
                    let delta = cell.pos.as_ivec2() - cell.src.as_ivec2();
                    let cost = cache.cost_lookup(delta);
                    let current = self.get(cell.pos).copied().unwrap_or(COST_FREE);

                    let new_cost = if current == COST_UNKNOWN {
                        // Nav2: when inflate_unknown, overwrite if cost > FREE; else overwrite if cost >= INSCRIBED
                        if options.inflate_unknown {
                            if cost > COST_FREE { Some(cost) } else { None }
                        } else if cost >= COST_INSCRIBED {
                            Some(cost)
                        } else {
                            None
                        }
                    } else {
                        Some(cost.max(current))
                    };

                    if let Some(c) = new_cost {
                        // Cell is within region, so set succeeds.
                        let _ = self.set(cell.pos, c);
                    }
                }

                for offset in CARDINAL_OFFSETS {
                    let neighbour = cell.pos.as_ivec2() + offset;
                    let neighbour = neighbour.as_uvec2();
                    if neighbour.x < width && neighbour.y < height {
                        enqueue_ctx.enqueue_neighbour(neighbour, cell.src);
                    }
                }
            }
        }
    }
}

/// Layer that inflates lethal obstacles within the update region.
///
/// Use [`InflationLayer::new`] or [`InflationLayer::from_config`] to construct.
/// Uses wavefront propagation with a precomputed inscribed cost cache.
pub struct InflationLayer {
    config: InflationConfig,
    cache: Option<InflationCache>,
    cached_radius_cells: Option<u32>,
    cached_resolution: Option<f32>,
    /// Reusable buffer for wavefront visitation; resized when grid size changes.
    seen: Vec<bool>,
}

impl InflationLayer {
    /// Create a layer from config.
    #[must_use]
    pub fn from_config(config: InflationConfig) -> Self {
        Self {
            config,
            cache: None,
            cached_radius_cells: None,
            cached_resolution: None,
            seen: Vec::new(),
        }
    }

    /// Clear cache and seen buffer so the next update rebuilds.
    fn invalidate_cache(&mut self) {
        self.cache = None;
        self.cached_radius_cells = None;
        self.cached_resolution = None;
        self.seen.clear();
    }
}

impl Layer for InflationLayer {
    fn reset(&mut self) {
        self.invalidate_cache();
    }

    fn is_clearable(&self) -> bool {
        false
    }

    fn update_bounds(&mut self, _robot: Pose2, bounds: &mut Bounds) {
        bounds.expand_by(self.config.inflation_radius_m);
    }

    fn match_size(&mut self, _info: &MapInfo) {
        self.invalidate_cache();
    }

    fn update_costs(&mut self, master: &mut Grid2d<u8>, region: CellRegion) {
        if !self.config.enabled {
            return;
        }

        let resolution = master.info().resolution;
        let radius_cells = inflation_radius_to_cells(self.config.inflation_radius_m, resolution);

        if radius_cells == 0 {
            return;
        }

        let need_rebuild = self
            .cached_radius_cells
            .map_or(true, |cached| cached != radius_cells)
            || self
                .cached_resolution
                .map_or(false, |r| (r - resolution).abs() > RESOLUTION_EPSILON);

        if need_rebuild {
            self.cache = Some(InflationCache::build(
                self.config.inflation_radius_m,
                self.config.inscribed_radius_m,
                resolution,
                self.config.cost_scaling_factor,
            ));
            self.cached_radius_cells = Some(radius_cells);
            self.cached_resolution = Some(resolution);
        }

        if let Some(ref cache) = self.cache {
            let options = InflateOptions {
                inflate_unknown: self.config.inflate_unknown,
                inflate_around_unknown: self.config.inflate_around_unknown,
            };
            master.inflate_wavefront(region, cache, options, &mut self.seen);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{CellRegion, Layer, LayeredGrid2d, Pose2};
    use crate::{MapInfo, types::COST_UNKNOWN};
    use glam::Vec2;

    #[test]
    fn radius_to_cells_basic() {
        assert_eq!(inflation_radius_to_cells(0.5, 0.1), 5);
        assert_eq!(inflation_radius_to_cells(0.05, 0.1), 1);
        assert_eq!(inflation_radius_to_cells(0.0, 0.1), 0);
        assert_eq!(inflation_radius_to_cells(1.0, 0.0), 0);
        assert_eq!(inflation_radius_to_cells(-1.0, 0.1), 0);
    }

    #[test]
    fn radius_to_cells_rounds_up() {
        assert_eq!(inflation_radius_to_cells(0.15, 0.1), 2);
    }

    #[test]
    fn inscribed_cost_at_zero_is_lethal() {
        assert_eq!(inscribed_inflation_cost(0.0, 2.0, 3.0), COST_LETHAL);
    }

    #[test]
    fn inscribed_cost_within_inscribed_is_inscribed() {
        assert_eq!(
            inscribed_inflation_cost(1.0, 2.0, 3.0),
            crate::types::COST_INSCRIBED
        );
    }

    #[test]
    fn inscribed_cost_decays_beyond_inscribed() {
        let cost = inscribed_inflation_cost(3.0, 1.0, 1.0);
        assert!(cost < crate::types::COST_INSCRIBED);
        assert!(cost > 0);
    }

    #[test]
    fn inflation_cache_build() {
        let cache = InflationCache::build(1.0, 0.1, 0.1, 3.0);
        println!("{:?}", cache);
    }

    fn make_grid(width: u32, height: u32, data: Vec<u8>) -> Grid2d<u8> {
        Grid2d::new(
            MapInfo {
                width,
                height,
                resolution: 1.0,
                ..Default::default()
            },
            data,
        )
        .unwrap()
    }

    #[test]
    fn inflate_single_lethal_cell() {
        let mut data = vec![COST_FREE; 25];
        data[2 * 5 + 2] = COST_LETHAL;
        let source = make_grid(5, 5, data);

        let mut dest = source.clone();
        dest.inflate_inscribed(2.0, 0.0, 3.0);

        assert_eq!(dest.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        assert!(dest.get(UVec2::new(3, 2)).copied().unwrap_or(0) > 0);
        assert!(dest.get(UVec2::new(1, 2)).copied().unwrap_or(0) > 0);
        assert_eq!(dest.get(UVec2::new(0, 0)).copied(), Some(COST_FREE));
    }

    #[test]
    fn inflate_empty_grid_stays_free() {
        let source = Grid2d::<u8>::empty(MapInfo {
            width: 10,
            height: 10,
            resolution: 0.5,
            ..Default::default()
        });
        let mut dest = source.clone();
        dest.inflate_inscribed(1.0, 2.0, 3.0);

        for (_, cost) in dest.iter_cells() {
            assert_eq!(*cost, COST_FREE);
        }
    }

    #[test]
    fn inflation_layer_in_layered_grid2d() {
        let info = MapInfo {
            width: 5,
            height: 5,
            resolution: 0.2,
            ..Default::default()
        };

        struct OneLethalLayer;
        impl Layer for OneLethalLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, robot: Pose2, bounds: &mut crate::grid::Bounds) {
                bounds.expand_to_include(robot.position);
                bounds.expand_by(1.0);
            }
            fn update_costs(&mut self, master: &mut Grid2d<u8>, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_LETHAL);
            }
        }

        let mut layered = LayeredGrid2d::new(info, 0, false);
        layered.add_layer(Box::new(OneLethalLayer));
        layered.add_layer(Box::new(InflationLayer::from_config(InflationConfig {
            inflation_radius_m: 0.5,
            inscribed_radius_m: 0.2,
            cost_scaling_factor: 3.0,
            ..Default::default()
        })));
        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let m = layered.master();
        assert_eq!(m.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        let near = m.get(UVec2::new(3, 2)).copied().unwrap_or(0);
        assert!(
            near > 0 && near < COST_LETHAL,
            "inflated cost should be in (0, LETHAL)"
        );
    }

    #[test]
    fn inflate_preserves_unknown_cells() {
        let mut data = vec![COST_UNKNOWN; 25];
        for x in 0..5 {
            data[2 * 5 + x] = COST_FREE;
        }
        data[2 * 5 + 4] = COST_LETHAL;
        let source = make_grid(5, 5, data);

        let mut dest = source.clone();
        dest.inflate_inscribed(1.2, 1.0, 3.0);

        assert_eq!(dest.get(UVec2::new(0, 0)).copied(), Some(COST_UNKNOWN));
        assert_eq!(dest.get(UVec2::new(1, 4)).copied(), Some(COST_UNKNOWN));
        assert_eq!(dest.get(UVec2::new(0, 2)).copied(), Some(COST_FREE));
        assert_eq!(dest.get(UVec2::new(4, 2)).copied(), Some(COST_LETHAL));
        assert!(dest.get(UVec2::new(3, 2)).copied().unwrap_or(0) > COST_FREE);
    }

    #[test]
    fn inflation_layer_disabled_does_not_inflate() {
        let info = MapInfo {
            width: 5,
            height: 5,
            resolution: 0.2,
            ..Default::default()
        };

        struct OneLethalLayer;
        impl Layer for OneLethalLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, robot: Pose2, bounds: &mut crate::grid::Bounds) {
                bounds.expand_to_include(robot.position);
                bounds.expand_by(1.0);
            }
            fn update_costs(&mut self, master: &mut Grid2d<u8>, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_LETHAL);
            }
        }

        let mut layered = LayeredGrid2d::new(info, COST_FREE, false);
        layered.add_layer(Box::new(OneLethalLayer));
        layered.add_layer(Box::new(InflationLayer::from_config(InflationConfig {
            enabled: false,
            inflation_radius_m: 0.5,
            inscribed_radius_m: 0.4,
            cost_scaling_factor: 3.0,
            ..Default::default()
        })));
        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let m = layered.master();
        assert_eq!(m.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        assert_eq!(m.get(UVec2::new(3, 2)).copied(), Some(COST_FREE));
    }

    #[test]
    fn inflation_layer_inflate_unknown_overwrites_unknown_with_inflation() {
        let info = MapInfo {
            width: 5,
            height: 5,
            resolution: 0.2,
            ..Default::default()
        };

        struct OneLethalLayer;
        impl Layer for OneLethalLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, robot: Pose2, bounds: &mut crate::grid::Bounds) {
                bounds.expand_to_include(robot.position);
                bounds.expand_by(1.0);
            }
            fn update_costs(&mut self, master: &mut Grid2d<u8>, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_LETHAL);
            }
        }

        let mut layered = LayeredGrid2d::new(info, COST_UNKNOWN, false);
        layered.add_layer(Box::new(OneLethalLayer));
        layered.add_layer(Box::new(InflationLayer::from_config(InflationConfig {
            inflate_unknown: true,
            inflation_radius_m: 0.5,
            inscribed_radius_m: 0.4,
            cost_scaling_factor: 3.0,
            ..Default::default()
        })));
        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let m = layered.master();
        assert_eq!(m.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        let near = m.get(UVec2::new(3, 2)).copied().unwrap_or(0);
        assert!(
            near > COST_FREE && near < COST_LETHAL,
            "inflate_unknown=true should overwrite unknown (3,2) with inflation cost, got {}",
            near
        );
    }

    #[test]
    fn inflation_layer_inflate_around_unknown_treats_unknown_as_seed() {
        let info = MapInfo {
            width: 5,
            height: 5,
            resolution: 0.2,
            ..Default::default()
        };

        struct OneUnknownLayer;
        impl Layer for OneUnknownLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, robot: Pose2, bounds: &mut crate::grid::Bounds) {
                bounds.expand_to_include(robot.position);
                bounds.expand_by(1.0);
            }
            fn update_costs(&mut self, master: &mut Grid2d<u8>, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_UNKNOWN);
            }
        }

        let mut layered = LayeredGrid2d::new(info, COST_FREE, false);
        layered.add_layer(Box::new(OneUnknownLayer));
        layered.add_layer(Box::new(InflationLayer::from_config(InflationConfig {
            inflation_radius_m: 0.5,
            inscribed_radius_m: 0.4,
            cost_scaling_factor: 3.0,
            inflate_around_unknown: true,
            ..Default::default()
        })));
        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let m = layered.master();
        // Seed cell gets the inflation cost at distance 0 (LETHAL), same as Nav2
        assert_eq!(m.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        let near = m.get(UVec2::new(3, 2)).copied().unwrap_or(0);
        assert!(
            near > COST_FREE,
            "inflate_around_unknown=true should inflate from unknown seed, got {}",
            near
        );
    }

    #[test]
    fn inscribed_cost_world_matches_cell_based_with_resolution() {
        let resolution = 0.05f32;
        let inscribed_radius_m = 0.1;
        let inscribed_radius_cells = inscribed_radius_m / resolution;
        let cost_scaling_factor = 10.0;

        let distance_m = 0.2;
        let distance_cells = distance_m / resolution;

        let world_cost =
            inscribed_inflation_cost(distance_m, inscribed_radius_m, cost_scaling_factor);
        let cell_cost = inscribed_inflation_cost(
            distance_cells,
            inscribed_radius_cells,
            cost_scaling_factor * resolution,
        );
        assert_eq!(world_cost, cell_cost);
    }
}
