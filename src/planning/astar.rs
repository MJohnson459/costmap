//! 8-connected A* over a costmap.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f32::consts::SQRT_2;

use glam::{DVec2, IVec2, UVec2};

use crate::{
    Costmap, DenseGrid,
    types::{COST_FREE, COST_INSCRIBED, COST_UNKNOWN},
};

/// Highest cost below lethal; scales the per-cell penalty to `[0, cost_weight]`.
const COST_SCALE: f32 = 252.0;

/// Largest effective [`PlanOptions::cost_weight`]. Larger finite weights and `+inf` are
/// clamped to it, so step costs stay finite and every reachable cell stays reachable.
pub const MAX_COST_WEIGHT: f32 = 1.0e6;

/// Moves to the eight neighbours, as (offset, step length in cells).
const MOVES: [(IVec2, f32); 8] = [
    (IVec2::new(1, 0), 1.0),
    (IVec2::new(-1, 0), 1.0),
    (IVec2::new(0, 1), 1.0),
    (IVec2::new(0, -1), 1.0),
    (IVec2::new(1, 1), SQRT_2),
    (IVec2::new(1, -1), SQRT_2),
    (IVec2::new(-1, 1), SQRT_2),
    (IVec2::new(-1, -1), SQRT_2),
];

const NO_PARENT: usize = usize::MAX;

/// Options for [`plan`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanOptions {
    /// Cells with a cost at or above this are impassable.
    pub lethal_threshold: u8,
    /// Whether [`COST_UNKNOWN`] cells can be traversed, at the cost of free space.
    pub allow_unknown: bool,
    /// Extra cost per metre at the highest non-lethal cost. A step of `length_m`
    /// into a cell of cost `c` costs `length_m * (1 + cost_weight * c / 252)`.
    /// NaN and negative values are treated as zero; values above [`MAX_COST_WEIGHT`],
    /// including `+inf`, are clamped to it.
    pub cost_weight: f32,
}

impl Default for PlanOptions {
    fn default() -> Self {
        Self {
            lethal_threshold: COST_INSCRIBED,
            allow_unknown: false,
            cost_weight: 3.0,
        }
    }
}

/// A planned path through a costmap.
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    /// Cells from start to goal inclusive; consecutive cells are 8-neighbours.
    pub cells: Vec<UVec2>,
    /// Sum of the step costs described at [`PlanOptions::cost_weight`].
    pub cost: f32,
}

impl Path {
    /// World coordinates of the cell centres along the path.
    pub fn to_world<T>(&self, grid: &DenseGrid<T>) -> Vec<DVec2> {
        self.cells.iter().map(|&c| grid.map_to_world(c)).collect()
    }

    /// Geometric length of the path in metres, ignoring cell costs.
    pub fn length_m(&self, resolution: f64) -> f64 {
        self.cells
            .windows(2)
            .map(|pair| pair[0].as_dvec2().distance(pair[1].as_dvec2()) * resolution)
            .sum()
    }
}

/// Plan the cheapest 8-connected path from `start` to `goal`.
///
/// Returns `None` if either cell is outside the grid or impassable, or if no path
/// exists. A diagonal move is allowed only when both cells it passes between are
/// passable. `start == goal` yields a one-cell path of cost 0.
pub fn plan(grid: &Costmap, start: UVec2, goal: UVec2, options: &PlanOptions) -> Option<Path> {
    passable_cost(grid, start, options)?;
    passable_cost(grid, goal, options)?;
    if start == goal {
        return Some(Path {
            cells: vec![start],
            cost: 0.0,
        });
    }

    let search = Search::new(grid, options);
    let solution = search.run(start, goal)?;
    Some(Path {
        cells: solution.trace(start, goal),
        cost: solution.cost,
    })
}

/// One move of [`plan`] to a neighbouring cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Move {
    pub cell: UVec2,
    /// Step length in cells: 1 or √2.
    pub length_cells: f32,
    /// Cost of entering `cell`, as in [`passable_cost`].
    pub cost: u8,
}

/// The moves [`plan`] may make from `cell`: one to each 8-neighbour that is passable
/// under `options`. A diagonal move is included only when both cells it passes between
/// are passable, so the cells these moves connect are exactly the cells `plan` can join.
pub fn moves<'a>(
    grid: &'a Costmap,
    cell: UVec2,
    options: &'a PlanOptions,
) -> impl Iterator<Item = Move> + 'a {
    let cost_at = move |offset: IVec2| {
        let next = offset_cell(grid, cell, offset)?;
        Some((next, passable_cost(grid, next, options)?))
    };
    MOVES.iter().filter_map(move |&(offset, length_cells)| {
        let (next, cost) = cost_at(offset)?;
        let diagonal = offset.x != 0 && offset.y != 0;
        let sides_open = !diagonal
            || (cost_at(IVec2::new(offset.x, 0)).is_some()
                && cost_at(IVec2::new(0, offset.y)).is_some());
        sides_open.then_some(Move {
            cell: next,
            length_cells,
            cost,
        })
    })
}

/// `cell` moved by `offset`, or `None` if that leaves the grid.
fn offset_cell(grid: &Costmap, cell: UVec2, offset: IVec2) -> Option<UVec2> {
    let moved = UVec2::try_from(cell.as_ivec2() + offset).ok()?;
    (moved.x < grid.size().x && moved.y < grid.size().y).then_some(moved)
}

/// Cost of entering `cell`, or `None` if it is outside the grid or impassable.
/// Unknown cells are tested before the threshold because [`COST_UNKNOWN`] is
/// numerically above every lethal threshold.
pub fn passable_cost(grid: &Costmap, cell: UVec2, options: &PlanOptions) -> Option<u8> {
    match *grid.get(cell)? {
        COST_UNKNOWN => options.allow_unknown.then_some(COST_FREE),
        cost => (cost < options.lethal_threshold).then_some(cost),
    }
}

/// `weight` limited to `0..=MAX_COST_WEIGHT`, with NaN as zero.
fn effective_cost_weight(weight: f32) -> f32 {
    if weight.is_nan() {
        0.0
    } else {
        weight.clamp(0.0, MAX_COST_WEIGHT)
    }
}

/// Cost of one step of `length_m` into a cell of cost `cell_cost`.
fn step_cost(length_m: f32, cell_cost: u8, cost_weight: f32) -> f32 {
    length_m * (1.0 + cost_weight * f32::from(cell_cost) / COST_SCALE)
}

/// Open-list entry. The ordering is reversed so the lowest `f` pops first from
/// the max-heap; ties prefer the larger `g`, which is nearer the goal.
#[derive(Clone, Copy)]
struct Node {
    f: f32,
    g: f32,
    index: usize,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| self.g.total_cmp(&other.g))
            .then_with(|| other.index.cmp(&self.index))
    }
}

struct Search<'a> {
    grid: &'a Costmap,
    options: &'a PlanOptions,
    cost_weight: f32,
}

/// A found path: parent link per cell index, and the cost of reaching the goal.
struct Solution {
    parent: Vec<usize>,
    width: usize,
    cost: f32,
}

impl<'a> Search<'a> {
    fn new(grid: &'a Costmap, options: &'a PlanOptions) -> Self {
        Self {
            grid,
            options,
            cost_weight: effective_cost_weight(options.cost_weight),
        }
    }

    fn index(&self, cell: UVec2) -> usize {
        cell.y as usize * self.grid.size().x as usize + cell.x as usize
    }

    fn cell(&self, index: usize) -> UVec2 {
        let width = self.grid.size().x as usize;
        UVec2::new((index % width) as u32, (index / width) as u32)
    }

    /// Octile distance to `goal` in metres. Admissible and consistent because
    /// every step costs at least its length.
    fn heuristic(&self, cell: UVec2, goal: UVec2) -> f32 {
        let d = (cell.as_ivec2() - goal.as_ivec2()).abs();
        let (long, short) = (d.x.max(d.y) as f32, d.x.min(d.y) as f32);
        (long - short + SQRT_2 * short) * self.grid.info().resolution as f32
    }

    /// Neighbours of `cell` reachable in one move, with the cost of the move.
    fn neighbours(&self, cell: UVec2) -> impl Iterator<Item = (UVec2, f32)> + '_ {
        let resolution = self.grid.info().resolution as f32;
        moves(self.grid, cell, self.options).map(move |step| {
            let length_m = step.length_cells * resolution;
            (step.cell, step_cost(length_m, step.cost, self.cost_weight))
        })
    }

    fn run(&self, start: UVec2, goal: UVec2) -> Option<Solution> {
        let cells = self.grid.size().x as usize * self.grid.size().y as usize;
        let mut best_g = vec![f32::INFINITY; cells];
        let mut parent = vec![NO_PARENT; cells];
        let mut open = BinaryHeap::new();

        let start_index = self.index(start);
        let goal_index = self.index(goal);
        best_g[start_index] = 0.0;
        open.push(Node {
            f: self.heuristic(start, goal),
            g: 0.0,
            index: start_index,
        });

        while let Some(node) = open.pop() {
            if node.index == goal_index {
                return Some(Solution {
                    parent,
                    width: self.grid.size().x as usize,
                    cost: node.g,
                });
            }
            if node.g > best_g[node.index] {
                continue;
            }
            for (next, step) in self.neighbours(self.cell(node.index)) {
                let next_index = self.index(next);
                let g = node.g + step;
                if g < best_g[next_index] {
                    best_g[next_index] = g;
                    parent[next_index] = node.index;
                    open.push(Node {
                        f: g + self.heuristic(next, goal),
                        g,
                        index: next_index,
                    });
                }
            }
        }
        None
    }
}

impl Solution {
    /// Cells from `start` to `goal`, found by following parent links back from `goal`.
    fn trace(&self, start: UVec2, goal: UVec2) -> Vec<UVec2> {
        let index_of = |c: UVec2| c.y as usize * self.width + c.x as usize;
        let cell = |i: usize| UVec2::new((i % self.width) as u32, (i / self.width) as u32);
        let start_index = index_of(start);
        let mut index = index_of(goal);
        let mut cells = vec![goal];
        while index != start_index {
            index = self.parent[index];
            cells.push(cell(index));
        }
        cells.reverse();
        cells
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{COST_LETHAL, MapInfo};
    use approx::assert_relative_eq;

    fn grid(width: u32, height: u32, value: u8) -> Costmap {
        let info = MapInfo {
            width,
            height,
            resolution: 0.5,
            ..Default::default()
        };
        Costmap::filled(info, value)
    }

    fn set(grid: &mut Costmap, x: u32, y: u32, value: u8) {
        grid.set(UVec2::new(x, y), value).unwrap();
    }

    fn cell(x: u32, y: u32) -> UVec2 {
        UVec2::new(x, y)
    }

    fn path_cost(grid: &Costmap, path: &Path, options: &PlanOptions) -> f32 {
        let resolution = grid.info().resolution;
        path.cells
            .windows(2)
            .map(|pair| {
                let length = pair[0].as_vec2().distance(pair[1].as_vec2()) * resolution as f32;
                let cell_cost = f32::from(*grid.get(pair[1]).unwrap());
                length * (1.0 + options.cost_weight * cell_cost / 252.0)
            })
            .sum()
    }

    #[test]
    fn straight_line_has_octile_cost() {
        let g = grid(20, 20, COST_FREE);
        let path = plan(&g, cell(1, 2), cell(11, 7), &PlanOptions::default()).unwrap();

        let expected = (5.0 + 5.0 * SQRT_2) * 0.5;
        assert_relative_eq!(path.cost, expected, epsilon = 1e-4);
        assert_relative_eq!(path.length_m(0.5), f64::from(expected), epsilon = 1e-4);
        assert_eq!(path.cells.first(), Some(&cell(1, 2)));
        assert_eq!(path.cells.last(), Some(&cell(11, 7)));
    }

    #[test]
    fn start_equals_goal_is_a_single_cell() {
        let g = grid(5, 5, COST_FREE);
        let path = plan(&g, cell(2, 2), cell(2, 2), &PlanOptions::default()).unwrap();
        assert_eq!(path.cells, vec![cell(2, 2)]);
        assert_eq!(path.cost, 0.0);
    }

    #[test]
    fn detours_around_a_wall() {
        let mut g = grid(11, 11, COST_FREE);
        for y in 0..10 {
            set(&mut g, 5, y, COST_LETHAL);
        }
        let path = plan(&g, cell(0, 0), cell(10, 0), &PlanOptions::default()).unwrap();

        assert!(path.cells.contains(&cell(5, 10)));
        assert!(path.cells.iter().all(|&c| g.get(c) == Some(&COST_FREE)));
        assert!(path.length_m(0.5) > 10.0 * 0.5);
    }

    #[test]
    fn no_path_when_enclosed() {
        let mut g = grid(9, 9, COST_FREE);
        for i in 2..=6 {
            for (x, y) in [(i, 2), (i, 6), (2, i), (6, i)] {
                set(&mut g, x, y, COST_LETHAL);
            }
        }
        assert!(plan(&g, cell(4, 4), cell(0, 0), &PlanOptions::default()).is_none());
        assert!(plan(&g, cell(0, 0), cell(4, 4), &PlanOptions::default()).is_none());
    }

    #[test]
    fn high_cost_weight_prefers_longer_cheap_corridor() {
        // A cheap corridor along y=0 and y=6 with a costly shortcut through y=3.
        let mut g = grid(15, 7, 200);
        for x in 0..15 {
            set(&mut g, x, 0, COST_FREE);
            set(&mut g, x, 6, COST_FREE);
        }
        for y in 0..7 {
            set(&mut g, 0, y, COST_FREE);
            set(&mut g, 14, y, COST_FREE);
        }

        let (start, goal) = (cell(0, 3), cell(14, 3));
        let eager = PlanOptions {
            cost_weight: 0.0,
            ..Default::default()
        };
        let careful = PlanOptions {
            cost_weight: 10.0,
            ..Default::default()
        };
        let direct = plan(&g, start, goal, &eager).unwrap();
        let around = plan(&g, start, goal, &careful).unwrap();

        assert!(direct.cells.iter().any(|c| c.y == 3 && c.x == 7));
        assert!(
            around
                .cells
                .iter()
                .all(|c| c.y != 3 || c.x == 0 || c.x == 14)
        );
        assert!(around.length_m(0.5) > direct.length_m(0.5));
    }

    #[test]
    fn non_finite_and_negative_cost_weights_follow_one_rule() {
        let mut g = grid(15, 7, 200);
        for x in 0..15 {
            set(&mut g, x, 0, COST_FREE);
            set(&mut g, x, 6, COST_FREE);
        }
        for y in 0..7 {
            set(&mut g, 0, y, COST_FREE);
            set(&mut g, 14, y, COST_FREE);
        }
        let (start, goal) = (cell(0, 3), cell(14, 3));
        let with_weight = |cost_weight: f32| {
            let options = PlanOptions {
                cost_weight,
                ..Default::default()
            };
            plan(&g, start, goal, &options)
        };

        let zero = with_weight(0.0).unwrap();
        for weight in [f32::NAN, -1.0, f32::NEG_INFINITY] {
            assert_eq!(with_weight(weight).as_ref(), Some(&zero), "{weight}");
        }

        let clamped = with_weight(MAX_COST_WEIGHT).unwrap();
        for weight in [f32::INFINITY, f32::MAX] {
            let path = with_weight(weight).expect("an infinite weight must not block cells");
            assert!(path.cost.is_finite(), "{weight}");
            assert_eq!(path, clamped, "{weight}");
        }
        assert!(clamped.cells.iter().all(|c| g.get(*c) == Some(&COST_FREE)));
    }

    #[test]
    fn unknown_cells_are_blocked_unless_allowed() {
        let mut g = grid(7, 3, COST_FREE);
        for y in 0..3 {
            set(&mut g, 3, y, COST_UNKNOWN);
        }
        let (start, goal) = (cell(0, 1), cell(6, 1));

        assert!(plan(&g, start, goal, &PlanOptions::default()).is_none());

        let allowing = PlanOptions {
            allow_unknown: true,
            ..Default::default()
        };
        let path = plan(&g, start, goal, &allowing).unwrap();
        assert_relative_eq!(path.cost, 6.0 * 0.5, epsilon = 1e-5);
    }

    #[test]
    fn unknown_start_or_goal_follows_allow_unknown() {
        let mut g = grid(4, 4, COST_FREE);
        set(&mut g, 3, 3, COST_UNKNOWN);
        assert!(plan(&g, cell(0, 0), cell(3, 3), &PlanOptions::default()).is_none());
        assert!(plan(&g, cell(3, 3), cell(0, 0), &PlanOptions::default()).is_none());
        let allowing = PlanOptions {
            allow_unknown: true,
            ..Default::default()
        };
        assert!(plan(&g, cell(0, 0), cell(3, 3), &allowing).is_some());
    }

    #[test]
    fn lethal_threshold_is_respected() {
        let mut g = grid(5, 1, COST_FREE);
        set(&mut g, 2, 0, 100);
        let strict = PlanOptions {
            lethal_threshold: 100,
            ..Default::default()
        };
        assert!(plan(&g, cell(0, 0), cell(4, 0), &strict).is_none());
        assert!(plan(&g, cell(0, 0), cell(4, 0), &PlanOptions::default()).is_some());
    }

    #[test]
    fn diagonal_does_not_cut_corners() {
        // Two blocked cells touching at a corner: the diagonal between (0,1) and
        // (1,0) is closed.
        let mut g = grid(2, 2, COST_FREE);
        set(&mut g, 1, 0, COST_LETHAL);
        assert!(plan(&g, cell(0, 0), cell(1, 1), &PlanOptions::default()).is_some());
        let path = plan(&g, cell(0, 0), cell(1, 1), &PlanOptions::default()).unwrap();
        assert_eq!(path.cells, vec![cell(0, 0), cell(0, 1), cell(1, 1)]);

        set(&mut g, 0, 1, COST_LETHAL);
        assert!(plan(&g, cell(0, 0), cell(1, 1), &PlanOptions::default()).is_none());
    }

    #[test]
    fn out_of_bounds_or_impassable_endpoints_fail() {
        let mut g = grid(5, 5, COST_FREE);
        set(&mut g, 4, 4, COST_LETHAL);
        let options = PlanOptions::default();
        assert!(plan(&g, cell(5, 0), cell(1, 1), &options).is_none());
        assert!(plan(&g, cell(0, 0), cell(1, 5), &options).is_none());
        assert!(plan(&g, cell(0, 0), cell(4, 4), &options).is_none());
        assert!(plan(&g, cell(4, 4), cell(0, 0), &options).is_none());
        assert!(plan(&g, cell(9, 9), cell(9, 9), &options).is_none());
    }

    #[test]
    fn path_is_contiguous_and_cost_matches_steps() {
        let mut g = grid(30, 30, COST_FREE);
        for y in 0..30 {
            for x in 0..30 {
                if (x * 7 + y * 13) % 11 == 0 && (x, y) != (0, 0) && (x, y) != (29, 29) {
                    set(&mut g, x, y, 60 + (x + y) as u8);
                }
            }
        }
        for y in 5..25 {
            set(&mut g, 15, y, COST_LETHAL);
        }
        let options = PlanOptions::default();
        let path = plan(&g, cell(0, 0), cell(29, 29), &options).unwrap();

        for pair in path.cells.windows(2) {
            let step = pair[0].as_ivec2() - pair[1].as_ivec2();
            assert!(step.abs().max_element() == 1, "{pair:?}");
        }
        assert_relative_eq!(path.cost, path_cost(&g, &path, &options), epsilon = 1e-3);
    }

    #[test]
    fn to_world_returns_cell_centres() {
        let g = grid(10, 10, COST_FREE);
        let path = plan(&g, cell(0, 0), cell(2, 0), &PlanOptions::default()).unwrap();
        assert_eq!(
            path.to_world(&g),
            vec![
                DVec2::new(0.25, 0.25),
                DVec2::new(0.75, 0.25),
                DVec2::new(1.25, 0.25)
            ]
        );
    }
}
