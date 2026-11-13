//! Connected regions of a costmap under the planner's moves.

use std::collections::VecDeque;

use glam::IVec2;

use crate::Costmap;

use super::astar::{PlanOptions, moves, passable_cost};

/// Identifier of one region of a [`Regions`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RegionId(u32);

/// The passable cells of a costmap grouped into regions: two cells share a region
/// exactly when [`plan`](super::plan) with the same options finds a path between them,
/// because the regions are flooded with the planner's own [`moves`].
pub struct Regions {
    width: u32,
    /// Region of each cell, row-major; [`NONE`] for impassable cells.
    labels: Vec<u32>,
    /// Cell count of each region, indexed by its id.
    sizes: Vec<usize>,
}

/// Label of a cell that belongs to no region.
const NONE: u32 = u32::MAX;

impl Regions {
    /// Label every passable cell of `grid` under `options`. Takes time linear in the
    /// number of cells.
    pub fn new(grid: &Costmap, options: &PlanOptions) -> Self {
        let width = grid.size().x;
        let cell = |index: usize| {
            IVec2::new(
                (index % width as usize) as i32,
                (index / width as usize) as i32,
            )
        };
        let index = |cell: IVec2| cell.y as usize * width as usize + cell.x as usize;
        let mut labels = vec![NONE; grid.as_slice().len()];
        let mut sizes = Vec::new();
        let mut queue = VecDeque::new();
        for seed in 0..labels.len() {
            if labels[seed] != NONE || passable_cost(grid, cell(seed), options).is_none() {
                continue;
            }
            let label = sizes.len() as u32;
            labels[seed] = label;
            queue.push_back(seed);
            let mut size = 0;
            while let Some(current) = queue.pop_front() {
                size += 1;
                for step in moves(grid, cell(current), options) {
                    let next = index(step.cell);
                    if labels[next] == NONE {
                        labels[next] = label;
                        queue.push_back(next);
                    }
                }
            }
            sizes.push(size);
        }
        Self {
            width,
            labels,
            sizes,
        }
    }

    /// Region of `cell`, or `None` if it is outside the grid or impassable.
    pub fn region(&self, cell: IVec2) -> Option<RegionId> {
        if cell.x < 0 || cell.y < 0 || cell.x as u32 >= self.width {
            return None;
        }
        let index = cell.y as usize * self.width as usize + cell.x as usize;
        match *self.labels.get(index)? {
            NONE => None,
            label => Some(RegionId(label)),
        }
    }

    /// Number of cells in `region`.
    pub fn size(&self, region: RegionId) -> usize {
        self.sizes[region.0 as usize]
    }

    /// The region with the most cells. Of equal regions, the one whose first cell comes
    /// first in row-major order. `None` if no cell is passable.
    pub fn largest(&self) -> Option<RegionId> {
        let mut best: Option<(usize, usize)> = None;
        for (label, &size) in self.sizes.iter().enumerate() {
            if best.is_none_or(|(_, most)| size > most) {
                best = Some((label, size));
            }
        }
        best.map(|(label, _)| RegionId(label as u32))
    }

    /// The cells of `region` in row-major order.
    pub fn cells(&self, region: RegionId) -> impl Iterator<Item = IVec2> + '_ {
        let width = self.width as usize;
        (self.labels.iter().enumerate())
            .filter(move |&(_, &label)| label == region.0)
            .map(move |(index, _)| IVec2::new((index % width) as i32, (index / width) as i32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        plan,
        types::{COST_FREE, COST_LETHAL, MapInfo},
    };

    #[test]
    fn cells_touching_only_at_a_blocked_corner_are_separate_regions() {
        // Two free 3 x 3 blocks that touch only at a corner: the planner's diagonal move
        // between them needs both blocked cells beside it to be passable.
        let grid = Costmap::from_fn(MapInfo::square(6, 1.0), |cell| {
            if (cell.x < 3 && cell.y < 3) || (cell.x >= 3 && cell.y >= 3) {
                COST_FREE
            } else {
                COST_LETHAL
            }
        });
        let options = PlanOptions::default();
        let regions = Regions::new(&grid, &options);
        let (a, b) = (IVec2::new(2, 2), IVec2::new(3, 3));
        assert_ne!(regions.region(a), regions.region(b));
        assert!(plan(&grid, a, b, &options).is_none());
        let lower = regions.region(a).unwrap();
        assert_eq!(regions.size(lower), 9);
        assert_eq!(regions.largest(), Some(lower));
        assert_eq!(regions.region(IVec2::new(3, 0)), None);
        assert_eq!(regions.region(IVec2::new(6, 0)), None);
    }

    #[test]
    fn largest_region_ignores_smaller_islands() {
        // Column 2 is a wall: a 2-wide island on the left, a 3-wide region on the right.
        let grid = Costmap::from_fn(MapInfo::square(6, 1.0), |cell| {
            if cell.x == 2 { COST_LETHAL } else { COST_FREE }
        });
        let regions = Regions::new(&grid, &PlanOptions::default());
        let largest = regions.largest().unwrap();
        assert_eq!(regions.size(largest), 18);
        assert!(regions.cells(largest).all(|cell| cell.x >= 3));
        assert_eq!(regions.cells(largest).next(), Some(IVec2::new(3, 0)));
    }
}
