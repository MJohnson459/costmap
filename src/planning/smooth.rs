//! Straight-line shortcuts through planned paths.
//!
//! An 8-connected path runs in eight directions only, and many such paths of equal
//! cost join two cells, so the one [`plan`](super::plan) returns can bend away from
//! the straight line between its ends. [`Path::shortcut`] replaces runs of cells with
//! straight segments wherever that is as safe as the run it replaces.

use glam::{I64Vec2, UVec2};

use crate::Costmap;

use super::astar::{Path, PlanOptions, passable_cost};

impl Path {
    /// Waypoints along the path, joined by straight segments.
    ///
    /// From each waypoint the next is the furthest later path cell that a straight
    /// line from cell centre to cell centre reaches through cells that are passable
    /// under `options` and no costlier than the costliest path cell it bypasses. The
    /// segments therefore never come closer to obstacles than the path did. The first
    /// and last cells are always waypoints.
    ///
    /// Lines are traced as in [`line_passable`].
    pub fn shortcut(&self, grid: &Costmap, options: &PlanOptions) -> Vec<UVec2> {
        shortcut_cells(&self.cells, |cell| passable_cost(grid, cell, options))
    }
}

/// [`Path::shortcut`] over `cells`, with `cost` giving the cost of entering a cell or
/// `None` if it is impassable.
pub(super) fn shortcut_cells(
    cells: &[UVec2],
    mut cost: impl FnMut(UVec2) -> Option<u8>,
) -> Vec<UVec2> {
    let Some(&first) = cells.first() else {
        return Vec::new();
    };
    let mut waypoints = vec![first];
    let mut anchor = 0;
    let mut bound = cost(first).unwrap_or(0);
    let mut next = 1;
    while next < cells.len() {
        let bound_with_next = bound.max(cost(cells[next]).unwrap_or(0));
        let adjacent = next == anchor + 1;
        if adjacent || line_within(cells[anchor], cells[next], bound_with_next, &mut cost) {
            bound = bound_with_next;
            next += 1;
        } else {
            anchor = next - 1;
            waypoints.push(cells[anchor]);
            bound = cost(cells[anchor]).unwrap_or(0);
        }
    }
    if let Some(&last) = cells.last()
        && waypoints.last() != Some(&last)
    {
        waypoints.push(last);
    }
    waypoints
}

/// Whether the straight line between the centres of `from` and `to` touches only cells
/// that are passable under `options`.
///
/// Every cell whose closed square the line meets is tested. Where the line passes
/// exactly through a cell corner, that includes both cells beside the corner, which
/// the planner's diagonal moves also require to be passable.
pub fn line_passable(grid: &Costmap, from: UVec2, to: UVec2, options: &PlanOptions) -> bool {
    line_within(from, to, u8::MAX, &mut |cell| {
        passable_cost(grid, cell, options)
    })
}

/// Whether every cell the line touches is passable with a cost of at most `bound`.
fn line_within(
    from: UVec2,
    to: UVec2,
    bound: u8,
    cost: &mut impl FnMut(UVec2) -> Option<u8>,
) -> bool {
    let mut ok = |cell: UVec2| cost(cell).is_some_and(|c| c <= bound);
    ok(from) && ok(to) && all_touched(from, to, ok)
}

/// Whether `ok` holds for every cell the segment between the centres of `from` and `to`
/// touches, visited in order along the segment and stopping at the first failure.
///
/// The walk is exact integer arithmetic: the segment crosses the next vertical cell edge
/// at parameter `(2i + 1) / (2 dx)` and the next horizontal one at `(2j + 1) / (2 dy)`,
/// after `i` and `j` crossings of each, so comparing `(2i + 1) dy` with `(2j + 1) dx`
/// orders the crossings and equality is a corner. Every touched cell lies between
/// `from` and `to`, so inside any grid holding both.
fn all_touched(from: UVec2, to: UVec2, mut ok: impl FnMut(UVec2) -> bool) -> bool {
    let delta = to.as_i64vec2() - from.as_i64vec2();
    let (span, step) = (delta.abs(), delta.signum());
    let mut cell = from.as_i64vec2();
    let mut crossed = I64Vec2::ZERO;
    let at = |c: I64Vec2| c.as_uvec2();
    while crossed.cmplt(span).any() {
        let x_next = (2 * crossed.x + 1) * span.y;
        let y_next = (2 * crossed.y + 1) * span.x;
        if x_next == y_next {
            let beside = [
                cell + I64Vec2::new(step.x, 0),
                cell + I64Vec2::new(0, step.y),
            ];
            if !beside.into_iter().all(|c| ok(at(c))) {
                return false;
            }
            cell += step;
            crossed += 1;
        } else if x_next < y_next {
            cell.x += step.x;
            crossed.x += 1;
        } else {
            cell.y += step.y;
            crossed.y += 1;
        }
        if !ok(at(cell)) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        plan,
        types::{COST_FREE, COST_LETHAL, MapInfo},
    };

    fn grid(width: u32, height: u32) -> Costmap {
        Costmap::filled(MapInfo::square(width.max(height), 1.0), COST_FREE)
    }

    fn cell(x: u32, y: u32) -> UVec2 {
        UVec2::new(x, y)
    }

    fn options() -> PlanOptions {
        PlanOptions::default()
    }

    #[test]
    fn open_ground_becomes_one_segment() {
        let g = grid(20, 20);
        let path = plan(&g, cell(1, 2), cell(15, 7), &options()).unwrap();
        assert!(path.cells.len() > 2);
        assert_eq!(path.shortcut(&g, &options()), [cell(1, 2), cell(15, 7)]);
    }

    #[test]
    fn segments_avoid_an_obstacle() {
        let mut g = grid(20, 20);
        for y in 0..15 {
            g.set(cell(10, y), COST_LETHAL).unwrap();
        }
        let path = plan(&g, cell(2, 2), cell(18, 2), &options()).unwrap();
        let waypoints = path.shortcut(&g, &options());

        assert_eq!(waypoints.first(), Some(&cell(2, 2)));
        assert_eq!(waypoints.last(), Some(&cell(18, 2)));
        assert!(waypoints.len() >= 3, "{waypoints:?}");
        for pair in waypoints.windows(2) {
            assert!(line_passable(&g, pair[0], pair[1], &options()), "{pair:?}");
        }
        assert!(!line_passable(&g, cell(2, 2), cell(18, 2), &options()));
    }

    #[test]
    fn shortcut_does_not_cut_through_costlier_cells() {
        let mut g = grid(20, 20);
        // A costly but passable band the planner routes around.
        for y in 0..12 {
            g.set(cell(10, y), 200).unwrap();
        }
        let path = plan(
            &g,
            cell(2, 2),
            cell(18, 2),
            &PlanOptions {
                cost_weight: 50.0,
                ..options()
            },
        )
        .unwrap();
        assert!(path.cells.iter().all(|&c| g[c] == COST_FREE));
        let waypoints = path.shortcut(&g, &options());
        assert!(waypoints.len() < path.cells.len());
        // Neighbouring waypoints are a planner step, which may pass a costly cell's corner.
        let segments = waypoints.windows(2).filter(|pair| {
            (pair[0].as_ivec2() - pair[1].as_ivec2())
                .abs()
                .max_element()
                > 1
        });
        for pair in segments {
            assert!(
                line_within(pair[0], pair[1], COST_FREE, &mut |c| passable_cost(
                    &g,
                    c,
                    &options()
                )),
                "{pair:?}"
            );
        }
    }

    /// Whether the segment between the centres of `from` and `to` meets the square of
    /// `cell`: the closed square, or only its interior. Exact: in doubled coordinates
    /// every point involved is an integer.
    fn segment_meets(from: UVec2, to: UVec2, cell: UVec2, closed: bool) -> bool {
        let (a, b) = (from.as_i64vec2() * 2 + 1, to.as_i64vec2() * 2 + 1);
        let (lo, hi) = (cell.as_i64vec2() * 2, cell.as_i64vec2() * 2 + 2);
        let apart = if closed {
            a.min(b).cmpgt(hi).any() || a.max(b).cmplt(lo).any()
        } else {
            a.min(b).cmpge(hi).any() || a.max(b).cmple(lo).any()
        };
        let d = b - a;
        if apart || d == I64Vec2::ZERO {
            return !apart;
        }
        let sides = [lo, hi, I64Vec2::new(lo.x, hi.y), I64Vec2::new(hi.x, lo.y)]
            .map(|corner| (corner - a).perp_dot(d).signum());
        let one_side = |s: i64| {
            sides
                .iter()
                .all(|&side| side == s || (!closed && side == 0))
        };
        !(one_side(1) || one_side(-1))
    }

    /// Small deterministic generator for the fuzz test.
    struct Lcg(u64);

    impl Lcg {
        fn below(&mut self, n: u32) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 33) % u64::from(n)) as u32
        }
    }

    #[test]
    fn line_passable_matches_every_cell_the_segment_touches() {
        const SIDE: u32 = 12;
        let mut rng = Lcg(0x5EED);
        let (mut checked, mut blocked, mut blocked_at_corner) = (0, 0, 0);
        while checked < 5000 {
            let mut g = grid(SIDE, SIDE);
            for _ in 0..12 {
                let c = cell(rng.below(SIDE), rng.below(SIDE));
                g.set(c, COST_LETHAL).unwrap();
            }
            for _ in 0..50 {
                let from = cell(rng.below(SIDE), rng.below(SIDE));
                let to = cell(rng.below(SIDE), rng.below(SIDE));
                let free_where = |closed: bool| {
                    g.indexed_iter()
                        .filter(|&(c, _)| segment_meets(from, to, c, closed))
                        .all(|(_, &cost)| cost == COST_FREE)
                };
                let expected = free_where(true);
                assert_eq!(
                    line_passable(&g, from, to, &options()),
                    expected,
                    "{from} -> {to}"
                );
                checked += 1;
                blocked += usize::from(!expected);
                blocked_at_corner += usize::from(!expected && free_where(false));
            }
        }
        assert!(
            blocked > 1000 && checked - blocked > 1000 && blocked_at_corner > 100,
            "{blocked} blocked, {blocked_at_corner} only at a corner, of {checked}"
        );
    }

    #[test]
    fn corner_crossings_need_both_cells_beside_the_corner() {
        let mut g = grid(4, 4);
        g.set(cell(1, 0), COST_LETHAL).unwrap();
        // The diagonal from (0, 0) to (2, 2) passes the corner shared by (0, 0), (1, 0),
        // (0, 1) and (1, 1); like the planner's diagonal move, it needs (1, 0) passable.
        assert!(!line_passable(&g, cell(0, 0), cell(2, 2), &options()));
        assert!(
            plan(&g, cell(0, 0), cell(1, 1), &options())
                .unwrap()
                .cells
                .len()
                == 3
        );
        assert!(line_passable(&g, cell(0, 1), cell(2, 3), &options()));
    }

    #[test]
    fn short_paths_are_kept() {
        let g = grid(5, 5);
        let single = Path {
            cells: vec![cell(2, 2)],
            cost: 0.0,
        };
        assert_eq!(single.shortcut(&g, &options()), [cell(2, 2)]);
        let empty = Path {
            cells: Vec::new(),
            cost: 0.0,
        };
        assert!(empty.shortcut(&g, &options()).is_empty());
    }
}
