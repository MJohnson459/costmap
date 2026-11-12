//! Global planning on a coarse overview of the whole map.

use std::{collections::HashSet, fmt};

use costmap::{
    CostGeoTiff, Costmap, MapInfo, PlanOptions, VoxelError, plan,
    planning::Regions,
    types::{COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN},
};
use glam::{DVec2, I64Vec2, IVec2, UVec2, Vec2};

use crate::ROBOT_RADIUS_M;

/// The overview cell edge is the largest power-of-two multiple of the map resolution
/// at or below this.
///
/// A stored level and the streamed overview of the same factor have the same cell size
/// but not the same cells: a stored level is anchored at the map's top-left corner and
/// the streamed overview at its bottom-left, so when the map height is not a multiple
/// of the factor their rows are offset. [`Overview::load`] sets the overview's origin to
/// its own lower-left corner in the map's frame, and every use of the overview converts
/// through that origin.
pub const DEFAULT_OVERVIEW_RESOLUTION_M: f64 = 0.5;
/// Chunk edge of the tiled grids the overview is read into.
const OVERVIEW_CHUNK_CELLS: u32 = 256;
/// Inflation of the overview for global planning.
const GLOBAL_INFLATION_RADIUS_M: f32 = 2.0;
const GLOBAL_COST_SCALING: f32 = 2.0;
const GLOBAL_PLAN: PlanOptions = PlanOptions {
    lethal_threshold: COST_INSCRIBED,
    allow_unknown: false,
    cost_weight: 3.0,
};
/// Default endpoints have no unknown overview cell within this distance.
const ENDPOINT_KNOWN_MARGIN_M: f32 = 2.0;
/// Largest gap between consecutive route points; the local planner's carrot is one of them.
pub const ROUTE_POINT_SPACING_M: f32 = 0.5;

/// Where the overview came from.
#[derive(Debug, Clone, PartialEq)]
pub enum OverviewSource {
    /// A reduced-resolution image stored in the file and recorded as max-pooled.
    FileLevel { factor: u32 },
    /// Max-pooled from the full-resolution raster while streaming it.
    Streamed { factor: u32, reason: StreamReason },
}

/// Why the overview was streamed instead of read from a stored level.
#[derive(Debug, Clone, PartialEq)]
pub enum StreamReason {
    /// The file has no level with the overview's factor.
    NoStoredLevel,
    /// The file's levels with the overview's factor are not recorded as max-pooled, so
    /// they may have lost thin obstacles. Holds the recorded method of the first one.
    NotMaxPooled { resampling: Option<String> },
}

impl fmt::Display for OverviewSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FileLevel { factor } => write!(f, "stored 1/{factor} level, max-pooled"),
            Self::Streamed { factor, reason } => {
                write!(f, "max-pooled 1/{factor} while streaming: ")?;
                match reason {
                    StreamReason::NoStoredLevel => write!(f, "the file has no 1/{factor} level"),
                    StreamReason::NotMaxPooled {
                        resampling: Some(method),
                    } => write!(
                        f,
                        "the stored 1/{factor} level was resampled with {method}, \
                         which can drop thin obstacles"
                    ),
                    StreamReason::NotMaxPooled { resampling: None } => write!(
                        f,
                        "the stored 1/{factor} level records no resampling method"
                    ),
                }
            }
        }
    }
}

/// The whole map at overview resolution, in the full-resolution map's local frame.
pub struct Overview {
    /// Raw costs, max-pooled, plus the cells [`mark_lethal`](Self::mark_lethal) marked.
    pub costs: Costmap,
    /// `costs` inflated for the robot; the global planner runs on this.
    pub inflated: Costmap,
    pub source: OverviewSource,
}

impl Overview {
    /// Read the overview of `map` from a stored level recorded as max-pooled, or stream
    /// it from the full-resolution raster if there is none. Overview cells are the
    /// largest power-of-two multiple of the map's cells no larger than `target_m`:
    /// the route can only pass gaps a few overview cells wide, so maps with narrow
    /// lanes need a small target.
    pub fn load(map: &mut CostGeoTiff, target_m: f64) -> Result<Self, VoxelError> {
        let factor = overview_factor(map.resolution(), target_m);
        let (tiles, size, offset, source) = match stored_level(map, factor) {
            Ok(index) => {
                let mut level = map.overview(index)?;
                // The stored level shares the map's top-left corner, not its lower-left
                // one, so its local frame is shifted; projected coordinates relate the two.
                let offset = map.georeference().to_local(level.georeference().anchor);
                let tiles = level.read_all(OVERVIEW_CHUNK_CELLS)?;
                (
                    tiles,
                    level.size(),
                    offset,
                    OverviewSource::FileLevel { factor },
                )
            }
            Err(reason) => {
                let tiles = map.read_overview_max(factor, OVERVIEW_CHUNK_CELLS)?;
                let size = map.size().map(|s| s.div_ceil(factor));
                let source = OverviewSource::Streamed { factor, reason };
                (tiles, size, DVec2::ZERO, source)
            }
        };
        let window = tiles.window(IVec2::ZERO, size);
        let info = MapInfo {
            origin: offset,
            ..*window.info()
        };
        let costs = Costmap::from_vec(info, window.as_slice().to_vec())?.with_fill(COST_UNKNOWN);
        Ok(Self {
            inflated: inflate(&costs),
            costs,
            source,
        })
    }

    /// Size in cells.
    pub fn size(&self) -> UVec2 {
        UVec2::new(self.costs.size().x, self.costs.size().y)
    }

    /// Mark the cells under `points` lethal in the raw costs, re-inflate if any changed,
    /// and return how many changed. Points outside the overview are ignored.
    pub fn mark_lethal(&mut self, points: impl Iterator<Item = DVec2>) -> usize {
        let mut changed = 0;
        for point in points {
            if let Some(cell) = self.costs.world_to_map(point)
                && self.costs[cell] != COST_LETHAL
            {
                self.costs[cell] = COST_LETHAL;
                changed += 1;
            }
        }
        if changed > 0 {
            self.inflated = inflate(&self.costs);
        }
        changed
    }

    /// Plan from `start` to `goal`, both in the map's local frame.
    ///
    /// The planned cells are shortcut into straight segments between overview cell
    /// centres, with the exact start and goal as end points, and resampled every
    /// [`ROUTE_POINT_SPACING_M`].
    pub fn plan_route(&self, start: DVec2, goal: DVec2) -> Result<Route, String> {
        let start_cell = self.passable_cell(start, "start")?;
        let goal_cell = self.passable_cell(goal, "goal")?;
        let path = plan(&self.inflated, start_cell, goal_cell, &GLOBAL_PLAN)
            .ok_or_else(|| format!("no route from {start} to {goal} on the overview"))?;
        let mut corners: Vec<DVec2> = path
            .shortcut(&self.inflated, &GLOBAL_PLAN)
            .into_iter()
            .map(|cell| self.inflated.map_to_world(cell))
            .collect();
        if let Some(first) = corners.first_mut() {
            *first = start;
        }
        if let Some(last) = corners.last_mut() {
            *last = goal;
        }
        Ok(Route::new(resample(&corners, ROUTE_POINT_SPACING_M)))
    }

    /// Two candidate cells far apart in the largest region the global planner can join.
    ///
    /// Candidates are the cells of the region that are free in the inflated overview and
    /// have no unknown overview cell within [`ENDPOINT_KNOWN_MARGIN_M`]. An overview cell
    /// is known if any full-resolution cell under it is, so a cell at the edge of the
    /// data can hold unknown ground, which the local planner will not enter.
    /// "Far apart" is straight-line distance, found by a double sweep: the candidate
    /// furthest from the region's first cell in row-major order, then the candidate
    /// furthest from that one. Each sweep is one pass over the cells, and the pair is at
    /// least half as far apart as the furthest pair of candidates. Ties go to the first
    /// cell in row-major order, so the choice is deterministic for a given map.
    pub fn default_endpoints(&self) -> Result<(DVec2, DVec2), String> {
        let none = "the map has no two free cells to drive between";
        let regions = Regions::new(&self.inflated, &GLOBAL_PLAN);
        let region = regions.largest().ok_or(none)?;
        let margin =
            (f64::from(ENDPOINT_KNOWN_MARGIN_M) / self.costs.info().resolution).ceil() as usize;
        let near_unknown = near_unknown(&self.costs, margin);
        let width = self.costs.size().x as usize;
        let candidates = || {
            regions.cells(region).filter(|&cell| {
                self.inflated[cell] == COST_FREE
                    && !near_unknown[cell.y as usize * width + cell.x as usize]
            })
        };
        let seed = candidates().next().ok_or(none)?;
        let a = furthest(candidates(), seed);
        let b = furthest(candidates(), a);
        if a == b {
            return Err(none.into());
        }
        Ok((self.inflated.map_to_world(a), self.inflated.map_to_world(b)))
    }

    /// Number of square chunks of `chunk_cells` cells of the full-resolution map, which
    /// has `map_size` cells of `resolution` metres, that lie under at least one known
    /// overview cell.
    ///
    /// A chunk holds data exactly when it lies under a known overview cell if every
    /// overview cell falls within one chunk: the overview's origin is on a chunk
    /// boundary and `chunk_cells` is a multiple of the overview factor. Otherwise an
    /// overview cell that straddles a chunk boundary counts every chunk it touches, and
    /// the count is an upper bound.
    pub fn chunks_under_known_cells(
        &self,
        chunk_cells: u32,
        resolution: f64,
        map_size: UVec2,
    ) -> usize {
        let info = self.costs.info();
        let factor = (info.resolution / resolution).round() as i64;
        let offset = (info.origin / resolution).round().as_i64vec2();
        let chunk = I64Vec2::splat(i64::from(chunk_cells));
        let mut chunks = HashSet::new();
        for (cell, _) in self
            .costs
            .indexed_iter()
            .filter(|&(_, &c)| c != COST_UNKNOWN)
        {
            let first = (offset + cell.as_i64vec2() * factor).max(I64Vec2::ZERO);
            let last = (offset + (cell.as_i64vec2() + 1) * factor).min(map_size.as_i64vec2()) - 1;
            if first.cmpgt(last).any() {
                continue;
            }
            let (lo, hi) = (first.div_euclid(chunk), last.div_euclid(chunk));
            for y in lo.y..=hi.y {
                chunks.extend((lo.x..=hi.x).map(|x| (x, y)));
            }
        }
        chunks.len()
    }

    fn passable_cell(&self, point: DVec2, name: &str) -> Result<UVec2, String> {
        let cell = self
            .inflated
            .world_to_map(point)
            .ok_or_else(|| format!("{name} {point} is outside the map"))?;
        match self.inflated[cell] {
            COST_UNKNOWN => Err(format!("{name} {point} is in unknown space")),
            cost if cost >= COST_INSCRIBED => {
                Err(format!("{name} {point} is in or too close to an obstacle"))
            }
            _ => Ok(cell),
        }
    }
}

/// The global route: a polyline in the map's local frame.
pub struct Route {
    points: Vec<DVec2>,
    /// Distance along the route to each point.
    cumulative_m: Vec<f32>,
}

impl Route {
    /// `points` must hold at least one point.
    pub fn new(points: Vec<DVec2>) -> Self {
        let mut total = 0.0;
        let mut cumulative_m = Vec::with_capacity(points.len());
        for (i, point) in points.iter().enumerate() {
            if i > 0 {
                total += points[i - 1].distance(*point) as f32;
            }
            cumulative_m.push(total);
        }
        Self {
            points,
            cumulative_m,
        }
    }

    pub fn points(&self) -> &[DVec2] {
        &self.points
    }

    pub fn length_m(&self) -> f32 {
        self.cumulative_m.last().copied().unwrap_or(0.0)
    }

    pub fn goal(&self) -> DVec2 {
        self.points[self.points.len() - 1]
    }

    /// Distance along the route to point `index`.
    pub fn distance_to_point(&self, index: usize) -> f32 {
        self.cumulative_m[index]
    }

    /// Index of the first point at or beyond `distance` along the route.
    pub fn index_at(&self, distance: f32) -> usize {
        self.cumulative_m
            .partition_point(|&d| d < distance)
            .min(self.points.len() - 1)
    }

    /// Unit direction of the route around point `index`.
    pub fn direction_at(&self, index: usize) -> Vec2 {
        let before = self.points[index.saturating_sub(1)];
        let after = self.points[(index + 1).min(self.points.len() - 1)];
        (after - before)
            .try_normalize()
            .map_or(Vec2::X, |direction| direction.as_vec2())
    }
}

/// Points along the polyline `corners`, keeping every corner and adding points so that
/// none are further apart than `spacing`.
pub fn resample(corners: &[DVec2], spacing: f32) -> Vec<DVec2> {
    let mut points: Vec<DVec2> = corners.first().copied().into_iter().collect();
    for pair in corners.windows(2) {
        let pieces = (pair[0].distance(pair[1]) / f64::from(spacing))
            .ceil()
            .max(1.0) as u32;
        points
            .extend((1..=pieces).map(|i| pair[0].lerp(pair[1], f64::from(i) / f64::from(pieces))));
    }
    points
}

/// Index of the first stored level of `map` with `factor` that is recorded as max-pooled,
/// or why there is none.
fn stored_level(map: &CostGeoTiff, factor: u32) -> Result<usize, StreamReason> {
    let candidates: Vec<usize> = (map.overview_factors().iter().enumerate())
        .filter(|&(_, &f)| f == factor)
        .map(|(index, _)| index)
        .collect();
    let first = *candidates.first().ok_or(StreamReason::NoStoredLevel)?;
    candidates
        .into_iter()
        .find(|&index| map.overview_is_max_pooled(index))
        .ok_or_else(|| StreamReason::NotMaxPooled {
            resampling: map.overview_resampling(first).map(str::to_owned),
        })
}

/// Largest power of two `f` with `resolution * f` at or below `target_m`.
/// It is 1 for cells coarser than half the target, and the overview is then the
/// full-resolution map.
fn overview_factor(resolution: f64, target_m: f64) -> u32 {
    let mut factor = 1;
    while resolution * f64::from(2 * factor) <= target_m {
        factor *= 2;
    }
    factor
}

/// For each cell of `grid`, row-major, whether an unknown cell lies within `margin`
/// cells along both axes (a square neighbourhood). Two sliding-window passes, so linear
/// in the number of cells whatever the margin.
fn near_unknown(grid: &Costmap, margin: usize) -> Vec<bool> {
    let (width, height) = (grid.size().x as usize, grid.size().y as usize);
    let unknown: Vec<bool> = grid.as_slice().iter().map(|&c| c == COST_UNKNOWN).collect();
    let rows = dilate(&unknown, width, height, margin, 1, width);
    dilate(&rows, height, width, margin, width, 1)
}

/// `mask` dilated by `margin` along one axis: `len` cells per line, `lines` lines, `step`
/// between cells of a line and `stride` between lines.
fn dilate(
    mask: &[bool],
    len: usize,
    lines: usize,
    margin: usize,
    step: usize,
    stride: usize,
) -> Vec<bool> {
    let mut out = vec![false; mask.len()];
    for line in 0..lines {
        let at = |i: usize| line * stride + i * step;
        // Set cells within the window [i - margin, i + margin] of cell i.
        let mut count = (0..margin.min(len)).filter(|&i| mask[at(i)]).count();
        for i in 0..len {
            if i + margin < len && mask[at(i + margin)] {
                count += 1;
            }
            if i > margin && mask[at(i - margin - 1)] {
                count -= 1;
            }
            out[at(i)] = count > 0;
        }
    }
    out
}

/// `costs` inflated for global planning.
fn inflate(costs: &Costmap) -> Costmap {
    let mut inflated = costs.clone();
    inflated.inflate(
        GLOBAL_INFLATION_RADIUS_M,
        ROBOT_RADIUS_M,
        GLOBAL_COST_SCALING,
    );
    inflated
}

/// The first of `cells` furthest from `from`, or `from` if there are none.
fn furthest(cells: impl Iterator<Item = UVec2>, from: UVec2) -> UVec2 {
    let distance_sq = |cell: UVec2| (cell.as_i64vec2() - from.as_i64vec2()).length_squared();
    cells.fold(from, |best, cell| {
        if distance_sq(cell) > distance_sq(best) {
            cell
        } else {
            best
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        gdal::{gdal_available, run_gdal},
        testing::{TempDir, write_map},
    };

    #[test]
    fn stored_and_streamed_overviews_place_features_at_the_same_position() {
        if !gdal_available("stored_and_streamed_overviews_place_features_at_the_same_position") {
            return;
        }
        // 201 rows at 0.1 m: the 1/4 levels' rows are offset by 0.3 m.
        let dir = TempDir::new("anchor");
        let feature = DVec2::new(12.05, 15.05);
        let lethal_at_feature = |p: DVec2| {
            if p.distance(feature) < 0.01 {
                COST_LETHAL
            } else {
                COST_FREE
            }
        };
        let stored = write_map(
            &dir,
            "stored.tif",
            DVec2::new(120.0, 20.1),
            0.1,
            lethal_at_feature,
        );
        let plain = dir.path().join("plain.tif");
        run_gdal("gdal_translate", &[&stored, &plain]);

        for (path, stored_level) in [(stored, true), (plain, false)] {
            let overview = Overview::load(
                &mut CostGeoTiff::open(&path).unwrap(),
                DEFAULT_OVERVIEW_RESOLUTION_M,
            )
            .unwrap();
            let from_file = matches!(overview.source, OverviewSource::FileLevel { factor: 4 });
            assert_eq!(from_file, stored_level, "{:?}", overview.source);
            let costs = &overview.costs;
            let lethal: Vec<DVec2> = (costs.indexed_iter())
                .filter(|&(_, &c)| c == COST_LETHAL)
                .map(|(cell, _)| costs.map_to_world(cell))
                .collect();
            assert_eq!(lethal.len(), 1, "{:?}", overview.source);
            let half_cell = costs.info().resolution / 2.0;
            let offset = (lethal[0] - feature).abs();
            assert!(
                offset.max_element() <= half_cell,
                "{:?}: {lethal:?}",
                overview.source
            );
        }
    }

    #[test]
    fn overview_factor_is_a_power_of_two_at_or_below_target() {
        let target = DEFAULT_OVERVIEW_RESOLUTION_M;
        assert_eq!(overview_factor(0.1, target), 4);
        assert_eq!(overview_factor(0.05, target), 8);
        assert_eq!(overview_factor(0.25, target), 2);
        assert_eq!(overview_factor(0.5, target), 1);
        assert_eq!(overview_factor(2.0, target), 1);
        assert_eq!(overview_factor(0.03, 0.12), 4);
        assert_eq!(overview_factor(0.03, 0.1), 2);
    }

    #[test]
    fn resample_keeps_corners_and_bounds_spacing() {
        let corners = [DVec2::ZERO, DVec2::new(1.2, 0.0), DVec2::new(1.2, 0.2)];
        let points = resample(&corners, 0.5);
        assert_eq!(points.len(), 5);
        assert_eq!(points[3], corners[1]);
        assert_eq!(points[4], corners[2]);
        assert!(points.windows(2).all(|p| p[0].distance(p[1]) <= 0.5));
    }

    /// An overview whose raw and inflated costs are both `costs`.
    fn uninflated(costs: Costmap) -> Overview {
        Overview {
            inflated: costs.clone(),
            costs,
            source: OverviewSource::Streamed {
                factor: 1,
                reason: StreamReason::NoStoredLevel,
            },
        }
    }

    #[test]
    fn default_endpoints_are_joined_by_the_planner() {
        // Two free 3 x 3 blocks that touch only at a corner: an 8-connected flood joins
        // them, the planner's no-corner-cutting rule does not.
        let info = MapInfo::square(6, 1.0);
        let costs = Costmap::from_fn(info, |x, y| {
            if (x < 3 && y < 3) || (x >= 3 && y >= 3) {
                COST_FREE
            } else {
                COST_INSCRIBED
            }
        });
        let overview = uninflated(costs);
        let (a, b) = overview.default_endpoints().unwrap();
        let cell = |p: DVec2| overview.inflated.world_to_map(p).unwrap();
        assert!(
            plan(&overview.inflated, cell(a), cell(b), &GLOBAL_PLAN).is_some(),
            "{a} to {b}"
        );
    }

    /// Bound for the test below. Its test build (opt-level 1) took 1.5 s with the double
    /// sweep and 26 s comparing all pairs of a 5 m candidate lattice.
    const ENDPOINTS_TIME_LIMIT_S: f32 = 10.0;

    #[test]
    fn default_endpoints_stay_clear_of_unknown_ground() {
        // A free 40 x 20 overview with an unknown margin of 2 cells: its edge cells may
        // be max-pooled from blocks that are partly unknown at full resolution.
        let info = MapInfo {
            width: 40,
            height: 20,
            resolution: 0.4,
            origin: DVec2::ZERO,
        };
        let costs = Costmap::from_fn(info, |x, y| {
            let inside = (2..38).contains(&x) && (2..18).contains(&y);
            if inside { COST_FREE } else { COST_UNKNOWN }
        });
        let overview = uninflated(costs);
        let (a, b) = overview.default_endpoints().unwrap();
        for p in [a, b] {
            let cell = overview.costs.world_to_map(p).unwrap().as_ivec2();
            // 2 m at 0.4 m per cell.
            let margin = 5;
            assert!(cell.cmpge(IVec2::splat(2 + margin)).all(), "{p}");
            assert!(cell.cmplt(IVec2::new(38, 18) - margin).all(), "{p}");
        }
    }

    #[test]
    fn default_endpoints_take_linear_time_on_a_large_overview() {
        let side = 5000;
        let costs = Costmap::filled(MapInfo::square(side, 0.5), COST_FREE);
        let overview = uninflated(costs);
        let started = std::time::Instant::now();
        let (a, b) = overview.default_endpoints().unwrap();
        let elapsed = started.elapsed();
        assert!(
            elapsed.as_secs_f32() < ENDPOINTS_TIME_LIMIT_S,
            "{elapsed:?}"
        );
        let corners = [
            DVec2::splat(0.25),
            DVec2::splat(f64::from(side) * 0.5 - 0.25),
        ];
        assert!(corners.contains(&a) && corners.contains(&b), "{a} {b}");
    }

    #[test]
    fn chunks_under_known_cells_skip_unknown_chunks() {
        // A 4 x 4 overview at 1 m of a 16 x 16 map at 0.25 m, in four 8-cell chunks.
        let costs = Costmap::from_fn(MapInfo::square(4, 1.0), |x, y| {
            if (x, y) == (0, 0) || (x, y) == (3, 1) {
                COST_FREE
            } else {
                COST_UNKNOWN
            }
        });
        let overview = uninflated(costs);
        assert_eq!(
            overview.chunks_under_known_cells(8, 0.25, UVec2::splat(16)),
            2
        );
        // With 6-cell chunks the second known cell, cells 12..16 by 4..8, straddles
        // chunk rows 0 and 1.
        assert_eq!(
            overview.chunks_under_known_cells(6, 0.25, UVec2::splat(16)),
            3
        );
    }
}
