//! Rasters in, cost GeoTIFF out. The output is built in square blocks so the inputs are
//! read through their chunk caches and never held whole.

use std::{
    fmt,
    path::{Path, PathBuf},
};

use costmap::{
    DenseGrid, GeoReference, TiledGrid,
    types::{COST_INSCRIBED, COST_UNKNOWN},
    write_cost_geotiff_region,
};
use glam::{DVec2, IVec2, UVec2};

use crate::{
    error::GeoError,
    frame::{CellFrame, cells},
    overrides::{Zone, apply_zones, load_zones},
    raster::{DemRaster, OrthoRaster},
    rules::{CellInput, Colour, CostRules},
    slope::{baseline_metres, box_metres, box_radius, halo_cells, slope_degrees},
    vegetation::{self, vegetation_mask},
};

/// Output cells per block edge, excluding the halos, used by [`run`].
pub const DEFAULT_BLOCK_CELLS: u32 = 1024;
/// Cells per chunk edge of the output grid.
const CHUNK_CELLS: u32 = 256;
/// Largest halo in cells, for slope or vegetation; larger windows would make the halo
/// dominate each block.
const MAX_HALO_CELLS: u32 = 2048;
/// Slack when dividing an extent by the resolution, so 7528 cells of 0.05 m do not become 7527.
const EXTENT_EPSILON: f64 = 1e-6;

/// Inputs, output and rules of one run.
#[derive(Debug, Clone)]
pub struct Job {
    pub dsm: PathBuf,
    pub dtm: PathBuf,
    pub ortho: Option<PathBuf>,
    pub overrides: Option<PathBuf>,
    pub out: PathBuf,
    pub rules: CostRules,
}

/// Cells per cost class.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ClassCounts {
    /// Cost 0.
    pub free: u64,
    /// Costs 1 to 252.
    pub graded: u64,
    /// Costs 253 (inscribed) and 254 (lethal).
    pub lethal: u64,
    /// Cost 255.
    pub unknown: u64,
}

impl ClassCounts {
    fn add(&mut self, cost: u8) {
        match cost {
            0 => self.free += 1,
            1..COST_INSCRIBED => self.graded += 1,
            COST_UNKNOWN => self.unknown += 1,
            _ => self.lethal += 1,
        }
    }

    fn total(&self) -> u64 {
        self.free + self.graded + self.lethal + self.unknown
    }
}

/// What a run produced.
#[derive(Debug, Clone, PartialEq)]
pub struct Summary {
    /// Output size in cells.
    pub size: UVec2,
    /// Cell edge length in metres.
    pub resolution: f64,
    pub epsg: u32,
    /// Edge in metres of the box the DTM is averaged over for slope.
    pub slope_box_m: f64,
    /// Distance in metres between the two box means a slope difference is taken over.
    pub slope_baseline_m: f64,
    /// The box the vegetation rule counts green samples in; `None` if the rule is off.
    pub vegetation_box: Option<VegetationBox>,
    pub counts: ClassCounts,
}

/// The square box of orthophoto samples around a cell that the vegetation rule counts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VegetationBox {
    /// Edge in cells; odd.
    pub cells: u32,
    /// Edge in metres.
    pub metres: f64,
}

impl VegetationBox {
    fn new(resolution: f64, radius: u32) -> Self {
        Self {
            cells: 2 * radius + 1,
            metres: box_metres(resolution, radius),
        }
    }
}

/// `value` rounded to 6 decimals, so floating-point noise does not reach the output.
fn round_for_display(value: f64) -> f64 {
    (value * 1e6).round() / 1e6
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let extent = self.size.as_dvec2() * self.resolution;
        let total = self.counts.total().max(1) as f64;
        let percent = |n: u64| 100.0 * n as f64 / total;
        let resolution = round_for_display(self.resolution);
        writeln!(
            f,
            "output: {} x {} cells, {:.1} x {:.1} m, resolution {} m, EPSG:{}",
            self.size.x, self.size.y, extent.x, extent.y, resolution, self.epsg
        )?;
        writeln!(
            f,
            "slope: {} m box, {} m baseline",
            round_for_display(self.slope_box_m),
            round_for_display(self.slope_baseline_m)
        )?;
        if let Some(vegetation) = &self.vegetation_box {
            writeln!(
                f,
                "vegetation: {0} x {0} cells, {1} m box",
                vegetation.cells,
                round_for_display(vegetation.metres)
            )?;
        }
        write!(
            f,
            "cells: free {:.1}%, graded {:.1}%, lethal or inscribed (253-254) {:.1}%, unknown {:.1}%",
            percent(self.counts.free),
            percent(self.counts.graded),
            percent(self.counts.lethal),
            percent(self.counts.unknown)
        )
    }
}

/// Where the output grid sits and how large it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutputSpec {
    pub frame: CellFrame,
    pub size: UVec2,
}

impl OutputSpec {
    /// The grid of `resolution`-metre cells that starts at `lower_left` and fits inside
    /// `[lower_left, upper_right]`.
    pub fn fitting(
        lower_left: DVec2,
        upper_right: DVec2,
        resolution: f64,
    ) -> Result<Self, GeoError> {
        let cells = ((upper_right - lower_left) / resolution + EXTENT_EPSILON).floor();
        let fits = |n: f64| (1.0..=f64::from(i32::MAX)).contains(&n);
        if !(fits(cells.x) && fits(cells.y)) {
            return Err(GeoError::Raster(format!(
                "a {resolution} m grid over the DTM extent would have {} x {} cells",
                cells.x, cells.y
            )));
        }
        Ok(Self {
            frame: CellFrame {
                origin: lower_left,
                resolution,
            },
            size: UVec2::new(cells.x as u32, cells.y as u32),
        })
    }
}

/// The EPSG code the inputs share, or an error naming the first mismatch.
fn shared_epsg(frames: &[(&Path, Option<u32>)]) -> Result<u32, GeoError> {
    let mut shared = None;
    for &(path, epsg) in frames {
        let epsg = epsg.ok_or_else(|| {
            GeoError::Crs("the raster names no EPSG code, so its CRS cannot be checked".into())
                .at(path)
        })?;
        match shared {
            None => shared = Some((path, epsg)),
            Some((first, expected)) if expected != epsg => {
                return Err(GeoError::Crs(format!(
                    "inputs are in different CRSs: {} is EPSG:{expected} but {} is EPSG:{epsg}",
                    first.display(),
                    path.display()
                )));
            }
            Some(_) => {}
        }
    }
    shared
        .map(|(_, epsg)| epsg)
        .ok_or_else(|| GeoError::Crs("no input rasters".into()))
}

/// A block of output cells.
#[derive(Debug, Clone, Copy)]
struct Block {
    min: UVec2,
    size: UVec2,
}

fn blocks(size: UVec2, block_cells: u32) -> impl Iterator<Item = Block> {
    (0..size.y)
        .step_by(block_cells as usize)
        .flat_map(move |y| {
            (0..size.x).step_by(block_cells as usize).map(move |x| {
                let min = UVec2::new(x, y);
                Block {
                    min,
                    size: (size - min).min(UVec2::splat(block_cells)),
                }
            })
        })
}

/// Heights of the `size` cells starting at global cell `min` (which may be negative),
/// with NaN where the raster has no value.
fn sample_heights(
    raster: &mut DemRaster,
    frame: &CellFrame,
    min: IVec2,
    size: UVec2,
) -> Result<DenseGrid<f32>, GeoError> {
    let mut grid = DenseGrid::filled(cells(size), f32::NAN);
    for y in 0..size.y {
        for x in 0..size.x {
            let centre = frame.centre(
                i64::from(min.x) + i64::from(x),
                i64::from(min.y) + i64::from(y),
            );
            if let Some(height) = raster.sample(centre)? {
                grid[UVec2::new(x, y)] = height;
            }
        }
    }
    Ok(grid)
}

fn sample_colours(
    raster: Option<&mut OrthoRaster>,
    frame: &CellFrame,
    min: IVec2,
    size: UVec2,
) -> Result<DenseGrid<Colour>, GeoError> {
    let Some(raster) = raster else {
        return Ok(DenseGrid::filled(cells(size), Colour::Absent));
    };
    let mut grid = DenseGrid::filled(cells(size), Colour::NoData);
    for y in 0..size.y {
        for x in 0..size.x {
            let centre = frame.centre(
                i64::from(min.x) + i64::from(x),
                i64::from(min.y) + i64::from(y),
            );
            if let Some(rgb) = raster.sample(centre)? {
                grid[UVec2::new(x, y)] = Colour::Rgb(rgb);
            }
        }
    }
    Ok(grid)
}

fn optional(height: f32) -> Option<f32> {
    (!height.is_nan()).then_some(height)
}

/// Cells of surrounding data a block reads beyond its own cells, per input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Halos {
    /// Around the DTM, for the slope window.
    slope: u32,
    /// Around the orthophoto, for the vegetation box; 0 when the rule is off.
    vegetation: u32,
}

impl Halos {
    fn for_rules(rules: &CostRules, resolution: f64) -> Self {
        Self {
            slope: halo_cells(resolution, rules.slope_window_m),
            vegetation: if rules.vegetation_cost > 0 {
                vegetation::box_radius(resolution, rules.vegetation_window_m)
            } else {
                0
            },
        }
    }
}

/// Costs of a block from the rules, before overrides.
///
/// `dtm` covers the block plus `halos.slope` cells on every side, `colours` the block plus
/// `halos.vegetation`, and `dsm` the block.
fn block_costs(
    rules: &CostRules,
    resolution: f64,
    halos: Halos,
    dtm: &DenseGrid<f32>,
    dsm: &DenseGrid<f32>,
    colours: &DenseGrid<Colour>,
) -> DenseGrid<u8> {
    let slopes = slope_degrees(dtm, resolution, rules.slope_window_m);
    let vegetation =
        (rules.vegetation_cost > 0).then(|| vegetation_mask(colours, rules, halos.vegetation));
    DenseGrid::from_fn(cells(dsm.size()), |x, y| {
        let (sx, sy) = (x + halos.slope, y + halos.slope);
        let (cx, cy) = (x + halos.vegetation, y + halos.vegetation);
        rules.cell_cost(&CellInput {
            dsm: optional(dsm[UVec2::new(x, y)]),
            dtm: optional(dtm[UVec2::new(sx, sy)]),
            slope_deg: slopes[UVec2::new(sx, sy)],
            ortho_nodata: colours[UVec2::new(cx, cy)] == Colour::NoData,
            vegetation: vegetation
                .as_ref()
                .is_some_and(|mask| mask[UVec2::new(cx, cy)]),
        })
    })
}

struct Processor {
    rules: CostRules,
    spec: OutputSpec,
    halos: Halos,
    dsm: DemRaster,
    dtm: DemRaster,
    ortho: Option<OrthoRaster>,
    zones: Vec<Zone>,
}

impl Processor {
    fn block(&mut self, block: Block) -> Result<DenseGrid<u8>, GeoError> {
        let frame = &self.spec.frame;
        let resolution = frame.resolution;
        let min = block.min.as_ivec2();
        let slope_halo = self.halos.slope;
        let colour_halo = self.halos.vegetation;
        let dtm = sample_heights(
            &mut self.dtm,
            frame,
            min - IVec2::splat(slope_halo as i32),
            block.size + UVec2::splat(2 * slope_halo),
        )?;
        let dsm = sample_heights(&mut self.dsm, frame, min, block.size)?;
        let colours = sample_colours(
            self.ortho.as_mut(),
            frame,
            min - IVec2::splat(colour_halo as i32),
            block.size + UVec2::splat(2 * colour_halo),
        )?;
        let mut costs = block_costs(&self.rules, resolution, self.halos, &dtm, &dsm, &colours);
        let block_frame = CellFrame {
            origin: frame.origin + block.min.as_dvec2() * resolution,
            resolution,
        };
        apply_zones(&mut costs, &block_frame, &self.zones);
        Ok(costs)
    }
}

/// Run the whole pipeline in blocks of [`DEFAULT_BLOCK_CELLS`] cells and write the cost
/// GeoTIFF.
pub fn run(job: &Job) -> Result<Summary, GeoError> {
    run_in_blocks(job, DEFAULT_BLOCK_CELLS)
}

/// Run the whole pipeline in blocks of `block_cells` x `block_cells` output cells and
/// write the cost GeoTIFF. The output does not depend on the block size; only the
/// working set does.
///
/// Memory use is bounded by the raster chunk caches and one block, except for the output
/// raster, which `write_cost_geotiff_region` assembles in memory (one byte per cell).
pub fn run_in_blocks(job: &Job, block_cells: u32) -> Result<Summary, GeoError> {
    if block_cells == 0 {
        return Err(GeoError::Rules(
            "the block size must be at least 1 cell".into(),
        ));
    }
    job.rules.validate()?;
    if job.rules.vegetation_cost > 0 && job.ortho.is_none() {
        return Err(GeoError::Rules(
            "a vegetation cost needs an orthophoto".into(),
        ));
    }
    let mut dtm = DemRaster::open(&job.dtm)?;
    let mut dsm = DemRaster::open(&job.dsm)?;
    let mut ortho = job.ortho.as_deref().map(OrthoRaster::open).transpose()?;
    let mut frames = vec![
        (job.dtm.as_path(), dtm.frame().epsg),
        (job.dsm.as_path(), dsm.frame().epsg),
    ];
    if let (Some(path), Some(ortho)) = (job.ortho.as_deref(), &ortho) {
        frames.push((path, ortho.frame().epsg));
    }
    let epsg = shared_epsg(&frames)?;

    let resolution = job.rules.resolution.unwrap_or(dtm.frame().pixel_size.x);
    let radius = box_radius(resolution, job.rules.slope_window_m);
    let halos = Halos::for_rules(&job.rules, resolution);
    if halos.slope > MAX_HALO_CELLS {
        return Err(GeoError::Rules(format!(
            "a {} m slope window is {} cells of halo at {resolution} m; raise the resolution or shrink the window",
            job.rules.slope_window_m, halos.slope
        )));
    }
    if halos.vegetation > MAX_HALO_CELLS {
        return Err(GeoError::Rules(format!(
            "a {} m vegetation window is {} cells of halo at {resolution} m; raise the resolution or shrink the window",
            job.rules.vegetation_window_m, halos.vegetation
        )));
    }
    let row_span_m = f64::from(block_cells + 2 * halos.slope.max(halos.vegetation)) * resolution;
    dtm.cache_for_row_span(row_span_m);
    dsm.cache_for_row_span(row_span_m);
    if let Some(ortho) = ortho.as_mut() {
        ortho.cache_for_row_span(row_span_m);
    }
    let (lower_left, upper_right) = dtm.extent();
    let spec = OutputSpec::fitting(lower_left, upper_right, resolution)?;
    let zones = job
        .overrides
        .as_deref()
        .map(load_zones)
        .transpose()?
        .unwrap_or_default();

    let mut processor = Processor {
        rules: job.rules.clone(),
        spec,
        halos,
        dsm,
        dtm,
        ortho,
        zones,
    };
    let mut grid = TiledGrid::new(resolution as f32, CHUNK_CELLS, COST_UNKNOWN)?;
    let mut counts = ClassCounts::default();
    for block in blocks(spec.size, block_cells) {
        let costs = processor.block(block)?;
        store_block(&mut grid, block, &costs, &mut counts);
    }

    let georef = GeoReference {
        epsg: Some(epsg),
        anchor: spec.frame.origin,
    };
    write_cost_geotiff_region(&job.out, &grid, &georef, IVec2::ZERO, spec.size.as_ivec2())
        .map_err(|e| GeoError::from(e).at(&job.out))?;
    Ok(Summary {
        size: spec.size,
        resolution,
        epsg,
        slope_box_m: box_metres(resolution, radius),
        slope_baseline_m: baseline_metres(resolution, radius),
        vegetation_box: (job.rules.vegetation_cost > 0)
            .then(|| VegetationBox::new(resolution, halos.vegetation)),
        counts,
    })
}

/// Copy the known cells of `costs` into `grid` and count every cell. Unknown cells are
/// the grid's fill value, so all-unknown areas allocate no chunks.
fn store_block(
    grid: &mut TiledGrid<u8>,
    block: Block,
    costs: &DenseGrid<u8>,
    counts: &mut ClassCounts,
) {
    for y in 0..block.size.y {
        for x in 0..block.size.x {
            let cost = costs[UVec2::new(x, y)];
            counts.add(cost);
            if cost != COST_UNKNOWN {
                grid.set((block.min + UVec2::new(x, y)).as_ivec2(), cost);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::types::COST_LETHAL;

    #[test]
    fn output_spec_keeps_whole_cells_and_absorbs_rounding() {
        let spec = OutputSpec::fitting(
            DVec2::new(436_000.0, 4_572_000.0),
            DVec2::new(436_376.4, 4_572_296.95),
            0.05,
        )
        .unwrap();
        assert_eq!(spec.size, UVec2::new(7528, 5939));
        let coarse = OutputSpec::fitting(DVec2::ZERO, DVec2::new(10.9, 5.0), 2.0).unwrap();
        assert_eq!(coarse.size, UVec2::new(5, 2));
    }

    #[test]
    fn output_spec_rejects_degenerate_extents() {
        assert!(OutputSpec::fitting(DVec2::ZERO, DVec2::new(0.5, 5.0), 1.0).is_err());
        assert!(OutputSpec::fitting(DVec2::ZERO, DVec2::new(1e12, 1e12), 0.001).is_err());
    }

    #[test]
    fn blocks_tile_the_output_without_overlap() {
        let size = UVec2::new(2500, 1100);
        let all: Vec<Block> = blocks(size, DEFAULT_BLOCK_CELLS).collect();
        assert_eq!(all.len(), 3 * 2);
        let area: u64 = all
            .iter()
            .map(|b| u64::from(b.size.x) * u64::from(b.size.y))
            .sum();
        assert_eq!(area, 2500 * 1100);
        assert_eq!(all[2].size, UVec2::new(452, 1024));
        assert_eq!(all[5].min, UVec2::new(2048, 1024));
    }

    #[test]
    fn epsg_must_match_and_exist() {
        let a = Path::new("a.tif");
        let b = Path::new("b.tif");
        assert_eq!(
            shared_epsg(&[(a, Some(32617)), (b, Some(32617))]).unwrap(),
            32617
        );
        let mismatch = shared_epsg(&[(a, Some(32617)), (b, Some(32618))])
            .unwrap_err()
            .to_string();
        assert!(
            mismatch.contains("a.tif") && mismatch.contains("b.tif"),
            "{mismatch}"
        );
        let missing = shared_epsg(&[(a, Some(32617)), (b, None)])
            .unwrap_err()
            .to_string();
        assert!(
            missing.contains("b.tif") && missing.contains("EPSG"),
            "{missing}"
        );
    }

    #[test]
    fn class_counts_split_at_the_reserved_costs() {
        let mut counts = ClassCounts::default();
        for cost in [0, 1, 252, COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN] {
            counts.add(cost);
        }
        assert_eq!(
            counts,
            ClassCounts {
                free: 1,
                graded: 2,
                lethal: 2,
                unknown: 1
            }
        );
    }

    #[test]
    fn block_costs_read_the_dtm_through_the_halo() {
        let rules = CostRules::default();
        let halos = Halos {
            slope: 1,
            vegetation: 0,
        };
        let dtm = DenseGrid::filled(cells(UVec2::new(5, 5)), 100.0_f32);
        let mut dsm = DenseGrid::filled(cells(UVec2::new(3, 3)), 100.0_f32);
        dsm[UVec2::new(1, 1)] = 105.0;
        dsm[UVec2::new(0, 0)] = f32::NAN;
        let colours = DenseGrid::filled(cells(UVec2::new(3, 3)), Colour::Absent);
        let costs = block_costs(&rules, 0.5, halos, &dtm, &dsm, &colours);
        assert_eq!(costs[UVec2::new(1, 1)], COST_LETHAL);
        assert_eq!(costs[UVec2::new(0, 0)], COST_UNKNOWN);
        assert_eq!(costs[UVec2::new(2, 2)], 0);
    }

    #[test]
    fn halos_follow_the_enabled_rules() {
        let off = CostRules::default();
        assert_eq!(Halos::for_rules(&off, 0.05).vegetation, 0);
        let on = CostRules {
            vegetation_cost: 254,
            vegetation_window_m: 0.35,
            ..off
        };
        assert_eq!(
            Halos::for_rules(&on, 0.05),
            Halos {
                slope: 18,
                vegetation: 3
            }
        );
    }

    #[test]
    fn block_costs_read_vegetation_through_the_colour_halo() {
        let rules = CostRules {
            vegetation_cost: 254,
            vegetation_window_m: 0.15,
            ..CostRules::default()
        };
        let halos = Halos {
            slope: 0,
            vegetation: 1,
        };
        let dtm = DenseGrid::filled(cells(UVec2::new(2, 1)), 100.0_f32);
        let dsm = dtm.clone();
        let green = Colour::Rgb([40, 160, 40]);
        let bare = Colour::Rgb([150, 140, 120]);
        let colours =
            DenseGrid::from_fn(
                cells(UVec2::new(4, 3)),
                |x, _| if x == 0 { green } else { bare },
            );
        let costs = block_costs(&rules, 0.05, halos, &dtm, &dsm, &colours);
        assert_eq!((costs[UVec2::new(0, 0)], costs[UVec2::new(1, 0)]), (0, 0));
        let colours = DenseGrid::from_fn(
            cells(UVec2::new(4, 3)),
            |x, _| if x < 2 { green } else { bare },
        );
        let costs = block_costs(&rules, 0.05, halos, &dtm, &dsm, &colours);
        assert_eq!(
            (costs[UVec2::new(0, 0)], costs[UVec2::new(1, 0)]),
            (COST_LETHAL, 0)
        );
    }
}
