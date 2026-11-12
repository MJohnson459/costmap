//! The simulated world: the aerial map plus obstacles it does not contain, and a lidar.

use std::f32::consts::TAU;

use costmap::{
    Beam, Costmap, TiledGrid,
    iterators::line::LineIterator,
    types::{COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN},
};
use glam::{DVec2, IVec2, UVec2, Vec2};

use crate::route::Route;

pub const LIDAR_BEAMS: usize = 360;
pub const LIDAR_RANGE_M: f32 = 15.0;
/// A beam that reaches unknown ground ends this many cells, along the beam, before
/// entering it. The obstacle layer clears every cell up to and including the one
/// holding the beam's end; ending inside the last known cell rather than on its edge
/// keeps that end out of the unknown cell when the local costmap's cells are slightly
/// offset from the map's.
const UNKNOWN_STOP_BACKOFF_CELLS: f32 = 0.5;

/// An obstacle placed on the route that the aerial map does not show.
struct SurpriseSpec {
    /// Nominal position as a fraction of the route length.
    route_fraction: f32,
    /// Length along the route and width across it.
    size_m: Vec2,
}

/// A parked trailer 40 % of the way along, then a bale stack lying across the route.
const SURPRISES: [SurpriseSpec; 2] = [
    SurpriseSpec {
        route_fraction: 0.4,
        size_m: Vec2::new(3.0, 2.0),
    },
    SurpriseSpec {
        route_fraction: 0.8,
        size_m: Vec2::new(1.5, 4.0),
    },
];
/// A surprise obstacle needs open, known ground within this radius of its centre, so
/// the robot has room to go round it.
const SURPRISE_CLEARANCE_M: f32 = 8.0;
/// Step along the route when searching for open ground.
const SURPRISE_SEARCH_STEP_M: f32 = 1.0;
/// How far, as a fraction of the route length, a surprise obstacle may move from its
/// nominal position in either direction.
const SURPRISE_SEARCH_LIMIT_FRACTION: f32 = 0.15;

/// An oriented rectangle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    pub centre: DVec2,
    /// Unit vector along the obstacle's length.
    pub axis: Vec2,
    pub half_size: Vec2,
}

impl Obstacle {
    pub fn contains(&self, point: DVec2) -> bool {
        self.local(point).abs().cmple(self.half_size).all()
    }

    /// Distance from `point` to the rectangle; zero inside it.
    pub fn distance_to(&self, point: DVec2) -> f32 {
        (self.local(point).abs() - self.half_size)
            .max(Vec2::ZERO)
            .length()
    }

    /// `point` in the obstacle's frame: x along its axis, y across.
    fn local(&self, point: DVec2) -> Vec2 {
        let offset = (point - self.centre).as_vec2();
        Vec2::new(self.axis.dot(offset), self.axis.perp_dot(offset))
    }

    pub fn corners(&self) -> [DVec2; 4] {
        let along = (self.axis * self.half_size.x).as_dvec2();
        let across = (self.axis.perp() * self.half_size.y).as_dvec2();
        [
            self.centre - along - across,
            self.centre + along - across,
            self.centre + along + across,
            self.centre - along + across,
        ]
    }
}

/// Place the surprise obstacles on `route`, each centred on the route and aligned with
/// it, at the position nearest its nominal one (searching after, then before, in
/// [`SURPRISE_SEARCH_STEP_M`] steps) where it stands on open ground of the raw overview
/// `costs`. Obstacles that find no open ground are left out.
pub fn place_surprises(route: &Route, costs: &Costmap) -> Vec<Obstacle> {
    let length = route.length_m();
    let steps = (SURPRISE_SEARCH_LIMIT_FRACTION * length / SURPRISE_SEARCH_STEP_M) as i32;
    SURPRISES
        .iter()
        .filter_map(|spec| {
            let nominal = spec.route_fraction * length;
            (0..=steps)
                .flat_map(|k| [k, -k])
                .map(|k| route.index_at(nominal + k as f32 * SURPRISE_SEARCH_STEP_M))
                .find(|&index| is_open(costs, route.points()[index], SURPRISE_CLEARANCE_M))
                .map(|index| Obstacle {
                    centre: route.points()[index],
                    axis: route.direction_at(index),
                    half_size: spec.size_m / 2.0,
                })
        })
        .collect()
}

/// Whether every cell of `costs` within `radius` of `centre` is known and below inscribed.
fn is_open(costs: &Costmap, centre: DVec2, radius: f32) -> bool {
    let resolution = costs.info().resolution as f32;
    let reach = (radius / resolution).ceil() as i32;
    let Some(middle) = costs.world_to_map(centre) else {
        return false;
    };
    (-reach..=reach).all(|dy| {
        (-reach..=reach).all(|dx| {
            if (dx * dx + dy * dy) as f32 * resolution * resolution > radius * radius {
                return true;
            }
            let cell = middle.as_ivec2() + IVec2::new(dx, dy);
            let cost = (cell.min_element() >= 0)
                .then(|| costs.get(cell.as_uvec2()))
                .flatten();
            matches!(cost, Some(&c) if c != COST_UNKNOWN && c < COST_INSCRIBED)
        })
    })
}

/// Lethal cells around `centre` as a lidar would see them: the aerial map's lethal
/// cells plus the surprise obstacles. The window reaches [`LIDAR_RANGE_M`] each way.
pub fn ground_truth(tiles: &TiledGrid<u8>, centre: DVec2, obstacles: &[Obstacle]) -> Costmap {
    let side = (f64::from(2.0 * LIDAR_RANGE_M) / tiles.resolution()).ceil() as u32 + 2;
    let mut truth = tiles.window_centered(centre, UVec2::splat(side));
    let info = *truth.info();
    for obstacle in obstacles {
        let corners = obstacle.corners();
        let (min, max) = corners
            .iter()
            .fold((corners[0], corners[0]), |(lo, hi), &c| {
                (lo.min(c), hi.max(c))
            });
        // Float-to-unsigned casts saturate, so boxes off either side of the window
        // give empty ranges.
        let first = ((min - info.origin) / info.resolution).floor().as_uvec2();
        let end = ((max - info.origin) / info.resolution)
            .ceil()
            .min(DVec2::splat(f64::from(side)))
            .as_uvec2();
        for y in first.y..end.y {
            for x in first.x..end.x {
                let cell = UVec2::new(x, y);
                if obstacle.contains(truth.map_to_world(cell)) {
                    truth[cell] = COST_LETHAL;
                }
            }
        }
    }
    truth
}

/// A 360-degree scan from `origin` against `truth`.
///
/// A beam that reaches a lethal cell ends at that cell's centre and marks it. A beam
/// that reaches an unknown cell, where the map has no data and the simulation no ground
/// truth, ends [`UNKNOWN_STOP_BACKOFF_CELLS`] before entering it, marks nothing and
/// clears only the known cells before it. Any other beam ends at [`LIDAR_RANGE_M`] and
/// only clears.
pub fn scan(truth: &Costmap, origin: DVec2) -> Vec<Beam> {
    let resolution = truth.info().resolution as f32;
    (0..LIDAR_BEAMS)
        .map(|i| {
            let direction = Vec2::from_angle(TAU * i as f32 / LIDAR_BEAMS as f32);
            let stop =
                LineIterator::new(truth, origin, direction, LIDAR_RANGE_M).and_then(|mut cells| {
                    cells.find(|step| matches!(truth[step.cell], COST_LETHAL | COST_UNKNOWN))
                });
            match stop {
                Some(step) if truth[step.cell] == COST_LETHAL => Beam {
                    end: truth.map_to_world(step.cell),
                    hit: true,
                },
                Some(step) => {
                    // `step.t` is the distance in cells at which the beam enters the cell.
                    let known_cells = (step.t - UNKNOWN_STOP_BACKOFF_CELLS).max(0.0);
                    Beam {
                        end: origin + (direction * known_cells * resolution).as_dvec2(),
                        hit: false,
                    }
                }
                None => Beam {
                    end: origin + (direction * LIDAR_RANGE_M).as_dvec2(),
                    hit: false,
                },
            }
        })
        .collect()
}
