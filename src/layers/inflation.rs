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

use bitvec::prelude::*;
use glam::{IVec2, UVec2};

use crate::types::{COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN, MapInfo};
use crate::{
    Costmap,
    grid::{Bounds, CellRegion, Layer, Pose2},
};

/// Padding added to the cache so that it contains more of the exponential decay curve.
const CACHE_PADDING: u32 = 3;

/// Configuration for the inflation layer.
///
/// Groups parameters that control inflation behaviour. Defaults match Nav2.
#[derive(Debug, Clone)]
pub struct InflationConfig {
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
            inflate_unknown: false,
            inflate_around_unknown: false,
            inflation_radius_m: 0.55,
            cost_scaling_factor: 10.0,
            inscribed_radius_m: 0.0,
        }
    }
}

/// Layer that inflates lethal obstacles within the update region.
///
/// Use [`InflationLayer::new`] or [`InflationLayer::from_config`] to construct.
/// Uses wavefront propagation with a precomputed inscribed cost cache.
pub struct WavefrontInflationLayer {
    config: InflationConfig,
    wavefront: Option<WavefrontInflation>,
}

impl WavefrontInflationLayer {
    /// Create a new inflation layer with the given configuration.
    #[must_use]
    pub fn new(config: InflationConfig) -> Self {
        Self {
            config,
            wavefront: None,
        }
    }

    /// Clear cache and seen buffer so the next update rebuilds.
    fn invalidate_cache(&mut self) {
        self.wavefront = None;
    }
}

impl Layer for WavefrontInflationLayer {
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
        // TODO: Can we be more clever about this?
        self.invalidate_cache();
    }

    fn update_costs(&mut self, master: &mut Costmap, region: CellRegion) {
        let resolution = master.info().resolution;
        let radius_cells = cell_distance(self.config.inflation_radius_m, resolution);

        if radius_cells == 0 {
            return;
        }

        let need_rebuild = self
            .wavefront
            .as_ref()
            .map_or(true, |cache| cache.resolution != resolution);

        if need_rebuild {
            self.wavefront = Some(WavefrontInflation::build(
                self.config.inflation_radius_m,
                self.config.inscribed_radius_m,
                resolution,
                self.config.cost_scaling_factor,
            ));
        }

        if let Some(ref mut wavefront) = self.wavefront {
            let options = InflateOptions {
                inflate_unknown: self.config.inflate_unknown,
                inflate_around_unknown: self.config.inflate_around_unknown,
            };
            wavefront.inflate(master, region, options);
        }
    }
}

impl Costmap {
    /// Inflate the grid with the given radius and inscribed radius.
    ///
    /// Note this is a convenience method and is not as efficient as using the `InflationLayer` directly.
    pub fn inflate(&mut self, radius_m: f32, inscribed_radius_m: f32, cost_scaling_factor: f32) {
        let mut inflation = WavefrontInflationLayer::new(InflationConfig {
            inflation_radius_m: radius_m,
            inscribed_radius_m,
            cost_scaling_factor,
            ..Default::default()
        });
        inflation.update_costs(
            self,
            CellRegion::new(UVec2::ZERO, UVec2::new(self.width(), self.height())),
        );
    }
}

/// Cell data for wavefront propagation: cell position and the obstacle source
/// it was enqueued from (for cost lookup).
#[derive(Debug, Clone, Copy)]
struct CellData {
    pos: UVec2,
    src: UVec2,
}

impl CellData {
    fn new(pos: UVec2, src: UVec2) -> Self {
        Self { pos, src }
    }
}

/// Convert a distance in world units (meters) to a cell count.
///
/// Returns `ceil(distance / resolution)`. If resolution is zero or negative, or
/// if the result would be non-positive, returns 0.
#[inline]
fn cell_distance(distance: f32, resolution: f32) -> u32 {
    if resolution <= 0.0 || distance <= 0.0 {
        return 0;
    }
    (distance / resolution).ceil() as u32
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
fn inscribed_inflation_cost(distance: f32, inscribed_radius: f32, cost_scaling_factor: f32) -> u8 {
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

/// Precomputed cost, distance, and integer-level cache for wavefront inflation.
///
/// Stores costs and distances for every (dx, dy) offset in the inflation disk
/// and a level matrix for wavefront ordering. Uses symmetry for negative coords.
/// Recompute when radius, resolution (for world-unit layer), or inscribed/scaling change.
///
/// Sometimes referred to as "brushfire" inflation.
struct WavefrontInflation {
    /// Resolution of the grid. If this changes, the cache needs to be rebuilt.
    resolution: f32,
    /// Radius to inflate around a cell in world units
    radius: f32,
    /// Side length of the cost and distance caches (side * side)
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
    /// Reusable buffer for wavefront visitation; resized when grid size changes.
    inflation_cells: Vec<Vec<CellData>>,
    /// Reusable buffer for wavefront visitation; resized when grid size changes.
    seen: BitVec<usize, Lsb0>,
    /// Width of the grid
    width: usize,
    /// Height of the grid
    height: usize,
    /// Size of the grid (width * height)
    size: usize,
}

impl WavefrontInflation {
    /// Build the cache for inscribed cost.
    /// Distance lookup returns world distance (for wavefront radius check); costs use world units.
    pub fn build(
        radius_m: f32,
        inscribed_radius_m: f32,
        resolution: f32,
        cost_scaling_factor: f32,
    ) -> Self {
        let radius_cells = cell_distance(radius_m, resolution);

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

        let inflation_cells: Vec<Vec<CellData>> = vec![Vec::new(); (max_level + 1) as usize];

        Self {
            resolution,
            radius: radius_m,
            side: padded_radius,
            costs,
            distances,
            distance_levels,
            level_side,
            max_level,
            inflation_cells,
            seen: BitVec::new(),
            width: 0,
            height: 0,
            size: 0,
        }
    }

    /// Inflate lethal (and optionally unknown) cells in place using wavefront propagation.
    ///
    /// `seen` is a reusable buffer; it is resized to `width * height` if needed and cleared on use.
    pub fn inflate(&mut self, grid: &mut Costmap, region: CellRegion, options: InflateOptions) {
        let size = region.max.saturating_sub(region.min);
        if size.x == 0 || size.y == 0 {
            return;
        }

        self.width = grid.info().width as usize;
        self.height = grid.info().height as usize;
        self.size = (self.width * self.height) as usize;

        if self.seen.len() != self.size {
            self.seen.resize(self.size, false);
        } else {
            self.seen.fill(false);
        }

        let width = grid.width();
        let height = grid.height();

        if !self.generate_seeds(grid, region, options) {
            return;
        }

        for level in 0..=self.max_level {
            let level_usize = level as usize;
            let mut i = 0;
            while i < self.inflation_cells[level_usize].len() {
                let cell = self.inflation_cells[level_usize][i];
                i += 1;

                if !self.mark_seen(cell.pos) {
                    continue;
                }

                if region.contains(cell.pos) {
                    let delta = cell.pos.as_ivec2() - cell.src.as_ivec2();
                    let cost = self.cost_lookup(delta);
                    let current = grid.get(cell.pos).copied().unwrap_or(COST_FREE);

                    let new_cost = if current == COST_UNKNOWN {
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
                        let _ = grid.set(cell.pos, c);
                    }
                }

                for offset in CARDINAL_OFFSETS {
                    let neighbour = cell.pos.as_ivec2() + offset;
                    let neighbour = neighbour.as_uvec2();
                    if neighbour.x < width && neighbour.y < height {
                        if self.is_seen(neighbour) {
                            continue;
                        }
                        let delta = neighbour.as_ivec2() - cell.src.as_ivec2();
                        let dist = self.distance_lookup(delta);
                        if dist > self.radius {
                            continue;
                        }
                        let lvl = self.dist_level(delta);
                        if lvl <= self.max_level {
                            self.inflation_cells[lvl as usize]
                                .push(CellData::new(neighbour, cell.src));
                        }
                    }
                }
            }
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

    /// Generate the seeds for the wavefront propagation.
    ///
    /// Returns true if any seeds were generated.
    fn generate_seeds(
        &mut self,
        grid: &Costmap,
        region: CellRegion,
        options: InflateOptions,
    ) -> bool {
        // Clear the inflation from old seeds
        for level in 0..=self.max_level {
            self.inflation_cells[level as usize].clear();
        }

        let radius_cells = self.cell_inflation_radius();
        let search_min = region.min.saturating_sub(UVec2::splat(radius_cells));
        let search_max = (region.max + radius_cells).min(UVec2::new(grid.width(), grid.height()));

        let seeds = &mut self.inflation_cells[0];
        for y in search_min.y..search_max.y {
            for x in search_min.x..search_max.x {
                let pos = UVec2::new(x, y);
                let cost = grid.get(pos).copied().unwrap_or(COST_FREE);
                if cost == COST_LETHAL {
                    seeds.push(CellData::new(pos, pos));
                } else if options.inflate_around_unknown && cost == COST_UNKNOWN {
                    seeds.push(CellData::new(pos, pos));
                }
            }
        }

        !seeds.is_empty()
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
    fn cell_inflation_radius(&self) -> u32 {
        (self.radius / self.resolution).ceil() as u32
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
        self.seen.set(index, true);
        true
    }
}

impl std::fmt::Debug for WavefrontInflation {
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

/// Cardinal neighbour offsets for 4-connected wavefront propagation.
const CARDINAL_OFFSETS: [IVec2; 4] = [
    IVec2::new(-1, 0),
    IVec2::new(1, 0),
    IVec2::new(0, -1),
    IVec2::new(0, 1),
];

/// Options for inflation write and seed behaviour.
#[derive(Clone, Copy, Default)]
struct InflateOptions {
    /// When true, allow overwriting unknown cells with inflation cost if cost > FREE.
    pub inflate_unknown: bool,
    /// When true, treat COST_UNKNOWN cells as obstacle seeds (same as lethal).
    pub inflate_around_unknown: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{CellRegion, Layer, LayeredCostmap, Pose2};
    use crate::{MapInfo, types::COST_UNKNOWN};
    use glam::Vec2;

    #[test]
    fn radius_to_cells_basic() {
        assert_eq!(cell_distance(0.5, 0.1), 5);
        assert_eq!(cell_distance(0.05, 0.1), 1);
        assert_eq!(cell_distance(0.0, 0.1), 0);
        assert_eq!(cell_distance(1.0, 0.0), 0);
        assert_eq!(cell_distance(-1.0, 0.1), 0);
    }

    #[test]
    fn radius_to_cells_rounds_up() {
        assert_eq!(cell_distance(0.15, 0.1), 2);
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
        let cache = WavefrontInflation::build(1.0, 0.1, 0.1, 3.0);
        println!("{:?}", cache);
    }

    fn make_grid(width: u32, height: u32, data: Vec<u8>) -> Costmap {
        Costmap::init(
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
        let mut wavefront = WavefrontInflation::build(2.0, 0.0, dest.info().resolution, 3.0);
        wavefront.inflate(
            &mut dest,
            CellRegion::new(UVec2::ZERO, UVec2::new(5, 5)),
            InflateOptions::default(),
        );

        assert_eq!(dest.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        assert!(dest.get(UVec2::new(3, 2)).copied().unwrap_or(0) > 0);
        assert!(dest.get(UVec2::new(1, 2)).copied().unwrap_or(0) > 0);
        assert_eq!(dest.get(UVec2::new(0, 0)).copied(), Some(COST_FREE));
    }

    #[test]
    fn inflate_empty_grid_stays_free() {
        let source = Costmap::new(MapInfo {
            width: 10,
            height: 10,
            resolution: 0.5,
            ..Default::default()
        });
        let mut dest = source.clone();
        let mut wavefront = WavefrontInflation::build(1.0, 2.0, dest.info().resolution, 3.0);
        wavefront.inflate(
            &mut dest,
            CellRegion::new(UVec2::ZERO, UVec2::new(10, 10)),
            InflateOptions::default(),
        );

        for (_, cost) in dest.indexed_iter() {
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
            fn update_costs(&mut self, master: &mut Costmap, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_LETHAL);
            }
        }

        let mut layered = LayeredCostmap::new(info, 0, false);
        layered.add_layer(Box::new(OneLethalLayer));
        layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
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
        let mut wavefront = WavefrontInflation::build(1.2, 1.0, dest.info().resolution, 3.0);
        wavefront.inflate(
            &mut dest,
            CellRegion::new(UVec2::ZERO, UVec2::new(5, 5)),
            InflateOptions::default(),
        );

        assert_eq!(dest.get(UVec2::new(0, 0)).copied(), Some(COST_UNKNOWN));
        assert_eq!(dest.get(UVec2::new(1, 4)).copied(), Some(COST_UNKNOWN));
        assert_eq!(dest.get(UVec2::new(0, 2)).copied(), Some(COST_FREE));
        assert_eq!(dest.get(UVec2::new(4, 2)).copied(), Some(COST_LETHAL));
        assert!(dest.get(UVec2::new(3, 2)).copied().unwrap_or(0) > COST_FREE);
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
            fn update_costs(&mut self, master: &mut Costmap, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_LETHAL);
            }
        }

        let mut layered = LayeredCostmap::new(info, COST_UNKNOWN, false);
        layered.add_layer(Box::new(OneLethalLayer));
        layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
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
            fn update_costs(&mut self, master: &mut Costmap, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_UNKNOWN);
            }
        }

        let mut layered = LayeredCostmap::new(info, COST_FREE, false);
        layered.add_layer(Box::new(OneUnknownLayer));
        layered.add_layer(Box::new(WavefrontInflationLayer::new(InflationConfig {
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
