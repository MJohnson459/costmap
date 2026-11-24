//! Resampling real maps onto other lattices and frames loses no lethal cell and
//! closes no passable gap.
//!
//! - Worst cost: the destination cell under each lethal source cell's centre is
//!   lethal.
//! - Both policies: each lethal destination cell overlaps a lethal source cell.
//!   Free space therefore narrows by at most one destination cell per edge, and a
//!   gap wider than two destination cells stays open.
//!
//! The warehouse map is a fixture. The vineyard is a cost GeoTIFF of a
//! 166 x 196 m survey at 0.03 m, too large to check in: set `COSTMAP_VINEYARD` to
//! its path to include it.

use std::path::Path;

use costmap::types::{COST_LETHAL, CellRegion, Lattice, Pose2};
use costmap::{CostWindows, Costmap, Resample, RosMapLoader, costmap_from_occupancy, resample};
use glam::{DVec2, IVec2, UVec2};

/// A destination for one case: its resolution, the pose of the map's frame in
/// its frame, and the sub-cell placement of its lattice.
struct Case {
    name: &'static str,
    resolution: f64,
    transform: Pose2,
    anchor_offset: DVec2,
}

/// The cases for a map at `resolution` whose crop is centred at `centre`. Rotations
/// turn the map about that centre, and every destination frame has its origin
/// near the crop, as an `odom` frame would.
fn cases(resolution: f64, centre: DVec2) -> Vec<Case> {
    let about_centre = |shift: DVec2, yaw: f32| {
        let (sin, cos) = f64::from(yaw).sin_cos();
        let turned = DVec2::new(
            cos * centre.x - sin * centre.y,
            sin * centre.x + cos * centre.y,
        );
        Pose2::new(shift - turned, yaw)
    };
    let case = |name, scale: f64, shift: DVec2, yaw, offset: f64| Case {
        name,
        resolution: resolution * scale,
        transform: about_centre(shift, yaw),
        anchor_offset: DVec2::splat(offset * resolution * scale),
    };
    vec![
        case("twice as coarse", 2.0, DVec2::ZERO, 0.0, 0.0),
        case(
            "1.7 x coarser, sub-cell offset",
            1.7,
            DVec2::new(0.013, -0.021),
            0.0,
            0.37,
        ),
        case(
            "same resolution, sub-cell offset",
            1.0,
            DVec2::ZERO,
            0.0,
            0.5,
        ),
        case(
            "same resolution, odom drift",
            1.0,
            DVec2::new(0.4, -0.3),
            0.012,
            0.21,
        ),
        case(
            "same resolution, turned 0.3 rad",
            1.0,
            DVec2::ZERO,
            0.3,
            0.0,
        ),
        case(
            "twice as coarse, turned 1 rad",
            2.0,
            DVec2::new(1.1, 2.3),
            1.0,
            0.43,
        ),
        case("half the size, turned 0.7 rad", 0.5, DVec2::ZERO, 0.7, 0.1),
    ]
}

/// The destination cells covering `crop` of `source` once moved by `transform`.
fn destination(source: Lattice, crop: CellRegion, case: &Case) -> (Lattice, CellRegion) {
    let lattice = Lattice::new(case.anchor_offset, case.resolution);
    let (sin, cos) = f64::from(case.transform.yaw).sin_cos();
    let to_destination = |p: DVec2| {
        case.transform.position + DVec2::new(cos * p.x - sin * p.y, sin * p.x + cos * p.y)
    };
    let corners = [
        crop.min,
        IVec2::new(crop.max.x, crop.min.y),
        IVec2::new(crop.min.x, crop.max.y),
        crop.max,
    ]
    .map(|c| lattice.cell_at(to_destination(source.cell_corner(c))));
    let min = corners.into_iter().reduce(IVec2::min).unwrap();
    let max = corners.into_iter().reduce(IVec2::max).unwrap() + IVec2::ONE;
    (lattice, CellRegion::new(min, max))
}

/// `position` of the destination frame, in cell units of the source lattice.
fn in_source(source: Lattice, transform: Pose2, position: DVec2) -> DVec2 {
    let (sin, cos) = f64::from(transform.yaw).sin_cos();
    let p = position - transform.position;
    source.to_cells(DVec2::new(cos * p.x + sin * p.y, -sin * p.x + cos * p.y))
}

/// Whether a lethal cell of `map` lies within `reach` source cells of `point`, in
/// source cell units.
fn lethal_within(map: &Costmap, point: DVec2, reach: f64) -> bool {
    let low = (point - DVec2::splat(reach)).floor().as_ivec2();
    let high = (point + DVec2::splat(reach)).floor().as_ivec2();
    (low.y..=high.y).any(|y| {
        (low.x..=high.x).any(|x| {
            let cell = IVec2::new(x, y);
            let nearest = point.clamp(cell.as_dvec2(), (cell + IVec2::ONE).as_dvec2());
            map.get(cell) == Some(&COST_LETHAL) && nearest.distance(point) <= reach
        })
    })
}

fn check(map: &mut dyn CostWindows, crop: CellRegion) {
    let source_lattice = map.lattice();
    let source = map
        .window(crop.grown(crop.size().max_element() / 2 + 8))
        .unwrap();
    let lethal: Vec<IVec2> = source
        .indexed_iter()
        .filter(|&(cell, &cost)| cost == COST_LETHAL && crop.contains(cell))
        .map(|(cell, _)| cell)
        .collect();
    let cells = crop.size().x as usize * crop.size().y as usize;
    assert!(
        lethal.len() > cells / 100 && lethal.len() < cells / 2,
        "{} of {cells} cells lethal",
        lethal.len()
    );

    let centre = source_lattice.cell_center(crop.min + (crop.size() / 2).as_ivec2());
    for case in cases(source_lattice.resolution, centre) {
        let (lattice, cells) = destination(source_lattice, crop, &case);
        let scale = case.resolution / source_lattice.resolution;
        // A destination cell overlaps every source cell within half its diagonal of
        // its centre, and no other.
        let reach = scale * std::f64::consts::FRAC_1_SQRT_2 + 1e-6;
        for policy in [Resample::WorstCost, Resample::Nearest] {
            let name = format!("{} ({policy:?})", case.name);
            let out = resample(&mut *map, case.transform, lattice, cells, policy).unwrap();

            for (cell, &cost) in out.indexed_iter() {
                if cost != COST_LETHAL {
                    continue;
                }
                let centre = in_source(source_lattice, case.transform, lattice.cell_center(cell));
                assert!(
                    lethal_within(&source, centre, reach),
                    "{name}: lethal cell {cell} overlaps no lethal source cell"
                );
            }

            if policy == Resample::WorstCost {
                let (sin, cos) = f64::from(case.transform.yaw).sin_cos();
                for &cell in &lethal {
                    let p = source_lattice.cell_center(cell);
                    let moved = case.transform.position
                        + DVec2::new(cos * p.x - sin * p.y, sin * p.x + cos * p.y);
                    let under = lattice.cell_at(moved);
                    assert_eq!(
                        out.get(under),
                        Some(&COST_LETHAL),
                        "{name}: lethal source cell {cell} lost at {under}"
                    );
                }
            }
        }
    }
}

#[test]
fn resampling_the_warehouse_loses_no_obstacle_and_closes_no_gap() {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    let grid = RosMapLoader::load_from_yaml(manifest.join("tests/fixtures/warehouse.yaml"))
        .expect("warehouse map should load");
    let mut map = costmap_from_occupancy(&grid).expect("the map converts to costs");
    // Shelving and walls near the middle of the map.
    let crop = CellRegion::sized(IVec2::new(4000, 1500), UVec2::splat(300));
    check(&mut map, crop);
}

#[cfg(feature = "geotiff")]
#[test]
fn resampling_the_vineyard_loses_no_obstacle_and_closes_no_gap() {
    let Some(path) = std::env::var_os("COSTMAP_VINEYARD") else {
        eprintln!("skipping: COSTMAP_VINEYARD is not set");
        return;
    };
    let mut map = costmap::GeoTiffWindows::open(path, 256).expect("the vineyard opens");
    // Vine rows near the middle of the survey.
    let crop = CellRegion::sized(IVec2::new(2600, 3100), UVec2::splat(300));
    check(&mut map, crop);
}
