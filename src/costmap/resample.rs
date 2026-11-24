//! Copying a cost map onto another lattice, in the same frame or another one.

use std::ops::Range;

use glam::{DVec2, IVec2};

use crate::{
    CostWindows, Costmap,
    types::{
        COST_LETHAL, COST_UNKNOWN, CellRegion, CostmapError, Lattice, Pose2, lattice::floor_i32,
    },
};

/// How [`resample`] chooses a destination cell's cost from the source cells under it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resample {
    /// The highest known cost of the source cells the destination cell overlaps.
    /// No obstacle is lost; free space narrows by up to one destination cell per
    /// edge.
    WorstCost,
    /// The cost of the source cell under the destination cell's centre. Widths are
    /// kept to within a cell; an obstacle smaller than a destination cell can be lost.
    Nearest,
}

impl Resample {
    /// [`WorstCost`](Self::WorstCost) where destination cells are coarser than
    /// source cells by more than a relative 1e-4, [`Nearest`](Self::Nearest)
    /// otherwise.
    pub fn for_resolutions(source: f64, destination: f64) -> Self {
        if destination > source * (1.0 + 1e-4) {
            Self::WorstCost
        } else {
            Self::Nearest
        }
    }
}

/// Source cells a destination square may miss by and still not count as
/// overlapping, so that floating-point error on a shared edge adds no cell.
const EDGE_TOLERANCE: f64 = 1e-6;

/// The cells `cells` of `lattice`, filled from `map` by `policy`.
///
/// `transform` is the pose of `map`'s frame in the frame of `lattice`: a point `p`
/// of the map's frame lies at `transform.position + R(transform.yaw) p`. Pass
/// [`Pose2::default`] when both lattices are in one frame. The library keeps no
/// transform tree; the caller supplies the transform at each call.
///
/// A destination cell is [`COST_UNKNOWN`] only if every source cell the policy
/// reads is unknown or outside the map: unknown loses to any known cost. `map`
/// is asked for one window, the cells under `cells` and a margin of one cell.
///
/// ```
/// use costmap::{Costmap, Lattice, Resample, resample, types::{CellRegion, Pose2}};
/// use glam::{DVec2, IVec2, UVec2};
///
/// // A 0.05 m map with one lethal cell, read onto 0.1 m cells.
/// let fine = Lattice::new(DVec2::ZERO, 0.05);
/// let mut map = Costmap::filled(fine, CellRegion::from_size(UVec2::splat(20)), 0);
/// map.set(IVec2::new(7, 7), 254).unwrap();
///
/// let coarse = Lattice::new(DVec2::ZERO, 0.1);
/// let cells = CellRegion::from_size(UVec2::splat(10));
/// let window = resample(&mut &map, Pose2::default(), coarse, cells, Resample::WorstCost)?;
/// assert_eq!(window.get(IVec2::new(3, 3)), Some(&254));
/// # Ok::<(), costmap::CostmapError>(())
/// ```
pub fn resample(
    map: &mut (impl CostWindows + ?Sized),
    transform: Pose2,
    lattice: Lattice,
    cells: CellRegion,
    policy: Resample,
) -> Result<Costmap, CostmapError> {
    let needed = source_cells(map.lattice(), transform, lattice, cells);
    let source = map.window(needed)?;
    Ok(resample_window(&source, transform, lattice, cells, policy))
}

/// The cells of `source` under `cells` of `lattice`, with a margin of one cell.
fn source_cells(
    source: Lattice,
    transform: Pose2,
    lattice: Lattice,
    cells: CellRegion,
) -> CellRegion {
    if cells.size().cmpeq(glam::UVec2::ZERO).any() {
        return CellRegion::new(IVec2::ZERO, IVec2::ZERO);
    }
    let to_source = Placement::new(source, transform);
    let corners = [
        cells.min,
        IVec2::new(cells.max.x, cells.min.y),
        IVec2::new(cells.min.x, cells.max.y),
        cells.max,
    ]
    .map(|corner| to_source.cells_of(lattice.cell_corner(corner)));
    let low = corners.into_iter().reduce(DVec2::min).unwrap_or_default();
    let high = corners.into_iter().reduce(DVec2::max).unwrap_or_default();
    let floor = |v: DVec2| IVec2::new(floor_i32(v.x), floor_i32(v.y));
    CellRegion::new(floor(low), floor(high).saturating_add(IVec2::ONE)).grown(1)
}

/// Converts positions of the destination frame to cell units of the source lattice.
struct Placement {
    /// The destination frame's x and y axes, in the source frame.
    x_axis: DVec2,
    y_axis: DVec2,
    /// The destination frame's origin, in the source frame.
    origin: DVec2,
    lattice: Lattice,
}

impl Placement {
    fn new(source: Lattice, transform: Pose2) -> Self {
        let (sin, cos) = f64::from(transform.yaw).sin_cos();
        let x_axis = DVec2::new(cos, -sin);
        let y_axis = DVec2::new(sin, cos);
        let origin = -(x_axis * transform.position.x + y_axis * transform.position.y);
        Self {
            x_axis,
            y_axis,
            origin,
            lattice: source,
        }
    }

    /// A direction of the destination frame, in the source frame.
    fn rotate(&self, v: DVec2) -> DVec2 {
        self.x_axis * v.x + self.y_axis * v.y
    }

    /// `position` of the destination frame in cell units of the source lattice.
    fn cells_of(&self, position: DVec2) -> DVec2 {
        self.lattice.to_cells(self.origin + self.rotate(position))
    }
}

/// [`resample`] from a window already read. Source cells outside `source` are
/// unknown.
pub(crate) fn resample_window(
    source: &Costmap,
    transform: Pose2,
    lattice: Lattice,
    cells: CellRegion,
    policy: Resample,
) -> Costmap {
    let mut out = Costmap::filled(lattice, cells, COST_UNKNOWN);
    if out.as_slice().is_empty() || source.as_slice().is_empty() {
        return out;
    }
    let placement = Placement::new(source.lattice(), transform);
    let scale = lattice.resolution / source.lattice().resolution;
    let first_centre = lattice.cell_center(cells.min);
    let walk = Walk {
        first: placement.cells_of(first_centre) - source.cells().min.as_dvec2(),
        step_x: placement.rotate(DVec2::X) * scale,
        step_y: placement.rotate(DVec2::Y) * scale,
    };
    match policy {
        Resample::Nearest => nearest(source, &walk, &mut out),
        Resample::WorstCost => worst_cost(source, &walk, &mut out),
    }
    out
}

/// Where destination cell centres fall in the source window, in array offsets:
/// the centre of the destination cell at offset `(i, j)` is
/// `first + i * step_x + j * step_y`.
struct Walk {
    first: DVec2,
    step_x: DVec2,
    step_y: DVec2,
}

impl Walk {
    fn axis_aligned(&self) -> bool {
        self.step_x.y == 0.0 && self.step_y.x == 0.0
    }

    fn centre(&self, i: usize, j: usize) -> DVec2 {
        self.first + self.step_x * i as f64 + self.step_y * j as f64
    }
}

/// The offset of `v` in `0..len`, or `None` outside it.
#[inline]
fn offset(v: f64, len: usize) -> Option<usize> {
    let cell = floor_i32(v);
    (cell >= 0 && (cell as usize) < len).then_some(cell as usize)
}

fn nearest(source: &Costmap, walk: &Walk, out: &mut Costmap) {
    let (width, height) = (source.size().x as usize, source.size().y as usize);
    let out_width = out.size().x as usize;
    let data = source.as_slice();
    let rows = out.as_mut_slice().chunks_exact_mut(out_width);
    if walk.axis_aligned() {
        let columns: Vec<Option<usize>> = (0..out_width)
            .map(|i| offset(walk.centre(i, 0).x, width))
            .collect();
        for (j, row) in rows.enumerate() {
            let Some(y) = offset(walk.centre(0, j).y, height) else {
                continue;
            };
            let source_row = &data[y * width..][..width];
            for (cell, column) in row.iter_mut().zip(&columns) {
                if let Some(x) = *column {
                    *cell = source_row[x];
                }
            }
        }
        return;
    }
    for (j, row) in rows.enumerate() {
        for (i, cell) in row.iter_mut().enumerate() {
            let centre = walk.centre(i, j);
            if let (Some(x), Some(y)) = (offset(centre.x, width), offset(centre.y, height)) {
                *cell = data[y * width + x];
            }
        }
    }
}

/// The source offsets on one axis a destination cell centred at `centre` spans,
/// `half` either side of it, clipped to `0..len`.
#[inline]
fn span(centre: f64, half: f64, len: usize) -> Range<usize> {
    let low = floor_i32(centre - half + EDGE_TOLERANCE).max(0);
    let high = floor_i32(centre + half - EDGE_TOLERANCE)
        .saturating_add(1)
        .min(len as i32);
    low as usize..high.max(low) as usize
}

/// `cost` if it is known and above `worst`, else `worst`; [`COST_UNKNOWN`] is below
/// every known cost.
#[inline]
fn worse(worst: u8, cost: u8) -> u8 {
    if cost != COST_UNKNOWN && (worst == COST_UNKNOWN || cost > worst) {
        cost
    } else {
        worst
    }
}

fn worst_cost(source: &Costmap, walk: &Walk, out: &mut Costmap) {
    let (width, height) = (source.size().x as usize, source.size().y as usize);
    let out_width = out.size().x as usize;
    let data = source.as_slice();
    // Half the extent, in source cells, of a destination cell's bounding box.
    let half = (walk.step_x.abs() + walk.step_y.abs()) * 0.5;
    let rows = out.as_mut_slice().chunks_exact_mut(out_width);
    if walk.axis_aligned() {
        let columns: Vec<Range<usize>> = (0..out_width)
            .map(|i| span(walk.centre(i, 0).x, half.x, width))
            .collect();
        for (j, row) in rows.enumerate() {
            let ys = span(walk.centre(0, j).y, half.y, height);
            if ys.is_empty() {
                continue;
            }
            for (cell, xs) in row.iter_mut().zip(&columns) {
                if xs.is_empty() {
                    continue;
                }
                let mut cost = COST_UNKNOWN;
                for y in ys.clone() {
                    let source_row = &data[y * width..][xs.clone()];
                    cost = source_row.iter().fold(cost, |worst, &c| worse(worst, c));
                    if cost == COST_LETHAL {
                        break;
                    }
                }
                *cell = cost;
            }
        }
        return;
    }
    // A source cell overlaps the destination square if their projections overlap
    // on the destination square's two axes as well as on the source axes.
    let reach = |axis: DVec2| {
        (axis.length_squared() + axis.x.abs() + axis.y.abs()) * 0.5 - EDGE_TOLERANCE * axis.length()
    };
    let (reach_x, reach_y) = (reach(walk.step_x), reach(walk.step_y));
    for (j, row) in rows.enumerate() {
        for (i, cell) in row.iter_mut().enumerate() {
            let centre = walk.centre(i, j);
            let (xs, ys) = (
                span(centre.x, half.x, width),
                span(centre.y, half.y, height),
            );
            if xs.is_empty() || ys.is_empty() {
                continue;
            }
            // The source cell under the centre overlaps the square: only costs above
            // its own need the overlap test.
            let mut cost = match (offset(centre.x, width), offset(centre.y, height)) {
                (Some(x), Some(y)) => data[y * width + x],
                _ => COST_UNKNOWN,
            };
            'rows: for y in ys {
                let source_row = &data[y * width..][xs.clone()];
                for (x, &source_cost) in (xs.start..).zip(source_row) {
                    if worse(cost, source_cost) == cost {
                        continue;
                    }
                    let to_cell = DVec2::new(x as f64 + 0.5, y as f64 + 0.5) - centre;
                    if to_cell.dot(walk.step_x).abs() < reach_x
                        && to_cell.dot(walk.step_y).abs() < reach_y
                    {
                        cost = source_cost;
                        if cost == COST_LETHAL {
                            break 'rows;
                        }
                    }
                }
            }
            *cell = cost;
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::UVec2;

    use super::*;
    use crate::types::COST_FREE;

    fn map(lattice: Lattice, size: u32, lethal: &[IVec2]) -> Costmap {
        let mut map = Costmap::filled(lattice, CellRegion::from_size(UVec2::splat(size)), 0);
        for &cell in lethal {
            map.set(cell, COST_LETHAL).unwrap();
        }
        map
    }

    fn lethal(map: &Costmap) -> Vec<IVec2> {
        map.indexed_iter()
            .filter(|&(_, &c)| c == COST_LETHAL)
            .map(|(cell, _)| cell)
            .collect()
    }

    fn run(
        source: &Costmap,
        transform: Pose2,
        lattice: Lattice,
        cells: CellRegion,
        policy: Resample,
    ) -> Costmap {
        resample(&mut &*source, transform, lattice, cells, policy).unwrap()
    }

    #[test]
    fn the_same_lattice_copies_under_either_policy() {
        let lattice = Lattice::new(DVec2::new(3.0, -1.0), 0.25);
        let source = Costmap::from_fn(lattice, CellRegion::from_size(UVec2::splat(12)), |c| {
            (c.x * 12 + c.y) as u8
        });
        let cells = CellRegion::new(IVec2::new(-2, 3), IVec2::new(14, 9));
        for policy in [Resample::Nearest, Resample::WorstCost] {
            let out = run(&source, Pose2::default(), lattice, cells, policy);
            let expected = (&source).window(cells).unwrap();
            assert_eq!(out, expected, "{policy:?}");
        }
    }

    #[test]
    fn a_whole_cell_shift_of_frame_names_other_cells() {
        let lattice = Lattice::new(DVec2::ZERO, 0.5);
        let source = map(lattice, 10, &[IVec2::new(4, 6)]);
        let transform = Pose2::new(DVec2::new(1.0, -1.5), 0.0);
        let cells = CellRegion::from_size(UVec2::splat(10));
        for policy in [Resample::Nearest, Resample::WorstCost] {
            let out = run(&source, transform, lattice, cells, policy);
            assert_eq!(lethal(&out), [IVec2::new(6, 3)], "{policy:?}");
        }
    }

    #[test]
    fn worst_cost_keeps_a_lethal_cell_on_a_coarser_lattice() {
        let fine = Lattice::new(DVec2::ZERO, 0.1);
        let source = map(fine, 30, &[IVec2::new(13, 17)]);
        let coarse = Lattice::new(DVec2::new(0.05, 0.05), 0.3);
        let cells = CellRegion::from_size(UVec2::splat(10));
        let out = run(
            &source,
            Pose2::default(),
            coarse,
            cells,
            Resample::WorstCost,
        );
        // Cell (13, 17) spans 1.3..1.4 by 1.7..1.8 m: coarse cell (4, 5).
        assert_eq!(lethal(&out), [IVec2::new(4, 5)]);
        assert_eq!(out[IVec2::ZERO], COST_FREE);
    }

    #[test]
    fn nearest_can_lose_a_cell_smaller_than_its_cells() {
        let fine = Lattice::new(DVec2::ZERO, 0.1);
        let source = map(fine, 30, &[IVec2::new(13, 17)]);
        let coarse = Lattice::new(DVec2::ZERO, 0.3);
        let cells = CellRegion::from_size(UVec2::splat(10));
        let out = run(&source, Pose2::default(), coarse, cells, Resample::Nearest);
        assert!(lethal(&out).is_empty());
    }

    #[test]
    fn a_sub_cell_offset_marks_every_cell_a_lethal_cell_touches() {
        let lattice = Lattice::new(DVec2::ZERO, 1.0);
        let source = map(lattice, 8, &[IVec2::new(3, 3)]);
        let shifted = Lattice::new(DVec2::new(0.5, 0.25), 1.0);
        let cells = CellRegion::from_size(UVec2::splat(7));
        let out = run(
            &source,
            Pose2::default(),
            shifted,
            cells,
            Resample::WorstCost,
        );
        assert_eq!(
            lethal(&out),
            [
                IVec2::new(2, 2),
                IVec2::new(3, 2),
                IVec2::new(2, 3),
                IVec2::new(3, 3)
            ]
        );
        let out = run(&source, Pose2::default(), shifted, cells, Resample::Nearest);
        assert_eq!(lethal(&out), [IVec2::new(2, 3)]);
    }

    #[test]
    fn a_rotated_frame_moves_cells_round_its_origin() {
        let lattice = Lattice::new(DVec2::ZERO, 1.0);
        let source = map(lattice, 10, &[IVec2::new(5, 1)]);
        // The map's frame turned a quarter turn: its (5.5, 1.5) is at (-1.5, 5.5).
        let transform = Pose2::new(DVec2::ZERO, std::f32::consts::FRAC_PI_2);
        let dest = Lattice::new(DVec2::new(-10.0, 0.0), 1.0);
        let cells = CellRegion::from_size(UVec2::new(10, 10));
        let out = run(&source, transform, dest, cells, Resample::Nearest);
        assert_eq!(lethal(&out), [IVec2::new(8, 5)]);
        let out = run(&source, transform, dest, cells, Resample::WorstCost);
        assert_eq!(lethal(&out), [IVec2::new(8, 5)]);
    }

    #[test]
    fn worst_cost_marks_only_cells_a_rotated_lethal_cell_overlaps() {
        let lattice = Lattice::new(DVec2::ZERO, 1.0);
        let source = map(lattice, 10, &[IVec2::new(5, 5)]);
        // An eighth turn about the cell's centre: the cell becomes a diamond of
        // half-diagonal 0.71 m centred on (5.5, 5.5), which overlaps the four
        // edge neighbours of cell (5, 5) and not its corner neighbours.
        let (sin, cos) = std::f64::consts::FRAC_PI_4.sin_cos();
        let pivot = DVec2::splat(5.5);
        let rotated = DVec2::new(cos * pivot.x - sin * pivot.y, sin * pivot.x + cos * pivot.y);
        let transform = Pose2::new(pivot - rotated, std::f32::consts::FRAC_PI_4);
        let cells = CellRegion::from_size(UVec2::splat(10));
        let out = run(&source, transform, lattice, cells, Resample::WorstCost);
        assert_eq!(
            lethal(&out),
            [
                IVec2::new(5, 4),
                IVec2::new(4, 5),
                IVec2::new(5, 5),
                IVec2::new(6, 5),
                IVec2::new(5, 6),
            ]
        );
    }

    #[test]
    fn unknown_loses_to_any_known_cost() {
        let fine = Lattice::new(DVec2::ZERO, 0.5);
        let mut source =
            Costmap::filled(fine, CellRegion::from_size(UVec2::splat(4)), COST_UNKNOWN);
        source.set(IVec2::new(1, 0), 30).unwrap();
        let coarse = Lattice::new(DVec2::ZERO, 1.0);
        let cells = CellRegion::from_size(UVec2::splat(2));
        let out = run(
            &source,
            Pose2::default(),
            coarse,
            cells,
            Resample::WorstCost,
        );
        assert_eq!(
            out.as_slice(),
            [30, COST_UNKNOWN, COST_UNKNOWN, COST_UNKNOWN]
        );
    }

    #[test]
    fn cells_beyond_the_map_are_unknown() {
        let lattice = Lattice::new(DVec2::ZERO, 1.0);
        let source = map(lattice, 4, &[]);
        let cells = CellRegion::new(IVec2::new(2, 2), IVec2::new(6, 3));
        let turned = Pose2::new(DVec2::ZERO, 0.1);
        for (transform, policy) in [
            (Pose2::default(), Resample::Nearest),
            (Pose2::default(), Resample::WorstCost),
            (turned, Resample::Nearest),
            (turned, Resample::WorstCost),
        ] {
            let out = run(&source, transform, lattice, cells, policy);
            assert_eq!(out.cells(), cells);
            assert_eq!(out.as_slice()[3], COST_UNKNOWN, "{policy:?} {transform:?}");
            assert_eq!(out.as_slice()[0], COST_FREE, "{policy:?} {transform:?}");
        }
    }

    #[test]
    fn the_default_is_worst_cost_only_onto_coarser_cells() {
        assert_eq!(Resample::for_resolutions(0.05, 0.1), Resample::WorstCost);
        assert_eq!(Resample::for_resolutions(0.05, 0.05), Resample::Nearest);
        assert_eq!(
            Resample::for_resolutions(0.05, 0.050000001),
            Resample::Nearest
        );
        assert_eq!(Resample::for_resolutions(0.1, 0.05), Resample::Nearest);
    }
}
