//! End-to-end tests on a synthetic scene written to temporary GeoTIFFs.
//!
//! Scene, in metres east and north of `BASE`:
//! - flat ground at 100 m,
//! - a ramp rising at 40 degrees for x in 8..12, then a plateau,
//! - a 2 m tall box at x in 3..5, y in 10..12,
//! - a nodata hole in the DSM and DTM at x in 15..17, y in 2..4,
//! - green ground at x in 16..19, y in 10..14,
//! - a green strip 0.4 m wide at x in 14.6..15.0, y in 5..9, and a single green pixel at
//!   x in 14.8..14.9, y in 12.0..12.1; both sit on a block seam of the 37-cell block test,
//! - a transparent orthophoto patch at x in 1..2, y in 6..8.
//!
//! The three rasters deliberately differ in size and origin, as ODM's outputs do.

use std::{
    fs::File,
    io::BufWriter,
    path::{Path, PathBuf},
    process::Command,
};

use costmap::{CostGeoTiff, GeoReference, TiledGrid};
use costmap_geo::{
    CostRules, GeoError, Job,
    raster::{DemRaster, OrthoRaster},
    run, run_in_blocks,
};
use glam::DVec2;
use tempfile::TempDir;
use tiff::{
    encoder::{
        Compression, DeflateLevel, DirectoryEncoder, TiffEncoder, TiffKindStandard,
        colortype::{Gray32Float, RGB8, RGBA8},
    },
    tags::Tag,
};

const BASE: DVec2 = DVec2::new(500_000.0, 4_500_000.0);
const RAMP_RISE_PER_M: f64 = 0.8390996311772799; // tan(40 degrees)
const NODATA: f32 = -9999.0;
const FREE: u8 = 0;
const LETHAL: u8 = 254;
const UNKNOWN: u8 = 255;

fn ground(x: f64, _y: f64) -> f64 {
    100.0 + (x.clamp(8.0, 12.0) - 8.0) * RAMP_RISE_PER_M
}

fn in_box(x: f64, y: f64) -> bool {
    (3.0..5.0).contains(&x) && (10.0..12.0).contains(&y)
}

fn in_hole(x: f64, y: f64) -> bool {
    (15.0..17.0).contains(&x) && (2.0..4.0).contains(&y)
}

fn in_green_patch(x: f64, y: f64) -> bool {
    (16.0..19.0).contains(&x) && (10.0..14.0).contains(&y)
}

fn in_green_strip(x: f64, y: f64) -> bool {
    (14.6..15.0).contains(&x) && (5.0..9.0).contains(&y)
}

fn in_green_speck(x: f64, y: f64) -> bool {
    (14.8..14.9).contains(&x) && (12.0..12.1).contains(&y)
}

fn in_green(x: f64, y: f64) -> bool {
    in_green_patch(x, y) || in_green_strip(x, y) || in_green_speck(x, y)
}

/// Vegetation at a lethal cost over a 3 x 3 cell box at the scene's 0.1 m resolution.
fn lethal_vegetation() -> CostRules {
    CostRules {
        vegetation_cost: LETHAL,
        vegetation_window_m: 0.3,
        ..CostRules::default()
    }
}

fn in_transparent(x: f64, y: f64) -> bool {
    (1.0..2.0).contains(&x) && (6.0..8.0).contains(&y)
}

fn write_tags(
    encoder: &mut DirectoryEncoder<'_, BufWriter<File>, TiffKindStandard>,
    top_left: DVec2,
    pixel: f64,
    epsg: u16,
    nodata: Option<&str>,
) {
    encoder
        .write_tag(Tag::ModelPixelScaleTag, &[pixel, pixel, 0.0][..])
        .unwrap();
    encoder
        .write_tag(
            Tag::ModelTiepointTag,
            &[0.0, 0.0, 0.0, top_left.x, top_left.y, 0.0][..],
        )
        .unwrap();
    let keys: [u16; 16] = [1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, epsg];
    encoder
        .write_tag(Tag::GeoKeyDirectoryTag, &keys[..])
        .unwrap();
    if let Some(nodata) = nodata {
        encoder.write_tag(Tag::GdalNodata, nodata).unwrap();
    }
}

/// Projected position relative to `BASE` of the centre of pixel `(column, row)`.
fn centre(top_left: DVec2, pixel: f64, column: u32, row: u32) -> (f64, f64) {
    let at = top_left + DVec2::new(f64::from(column) + 0.5, -(f64::from(row) + 0.5)) * pixel;
    (at.x - BASE.x, at.y - BASE.y)
}

struct Dem<'a> {
    size: (u32, u32),
    top_left: DVec2,
    pixel: f64,
    epsg: u16,
    height: &'a dyn Fn(f64, f64) -> f32,
}

fn write_dem(path: &Path, dem: &Dem) {
    let mut data = Vec::new();
    for row in 0..dem.size.1 {
        for column in 0..dem.size.0 {
            let (x, y) = centre(dem.top_left, dem.pixel, column, row);
            data.push((dem.height)(x, y));
        }
    }
    let file = BufWriter::new(File::create(path).unwrap());
    let mut encoder = TiffEncoder::new(file)
        .unwrap()
        .with_compression(Compression::Deflate(DeflateLevel::Fast));
    let mut image = encoder
        .new_image::<Gray32Float>(dem.size.0, dem.size.1)
        .unwrap();
    image.rows_per_strip(16).unwrap();
    write_tags(
        image.encoder(),
        dem.top_left,
        dem.pixel,
        dem.epsg,
        Some("-9999"),
    );
    image.write_data(&data).unwrap();
}

fn write_ortho(path: &Path, size: (u32, u32), top_left: DVec2, pixel: f64, with_alpha: bool) {
    let mut data = Vec::new();
    for row in 0..size.1 {
        for column in 0..size.0 {
            let (x, y) = centre(top_left, pixel, column, row);
            let rgb = if in_green(x, y) {
                [40, 160, 40]
            } else {
                [150, 140, 120]
            };
            data.extend(rgb);
            if with_alpha {
                data.push(if in_transparent(x, y) { 0 } else { 255 });
            }
        }
    }
    let file = BufWriter::new(File::create(path).unwrap());
    let mut encoder = TiffEncoder::new(file)
        .unwrap()
        .with_compression(Compression::Deflate(DeflateLevel::Fast));
    if with_alpha {
        let mut image = encoder.new_image::<RGBA8>(size.0, size.1).unwrap();
        image.rows_per_strip(16).unwrap();
        write_tags(image.encoder(), top_left, pixel, 32617, None);
        image.write_data(&data).unwrap();
    } else {
        let mut image = encoder.new_image::<RGB8>(size.0, size.1).unwrap();
        image.rows_per_strip(16).unwrap();
        write_tags(image.encoder(), top_left, pixel, 32617, None);
        image.write_data(&data).unwrap();
    }
}

struct Scene {
    dir: TempDir,
    dsm: PathBuf,
    dtm: PathBuf,
    ortho: PathBuf,
}

impl Scene {
    fn new() -> Self {
        let dir = TempDir::new().unwrap();
        let (dsm, dtm, ortho) = (
            dir.path().join("dsm.tif"),
            dir.path().join("dtm.tif"),
            dir.path().join("ortho.tif"),
        );
        let dtm_height = |x: f64, y: f64| {
            if in_hole(x, y) {
                NODATA
            } else {
                ground(x, y) as f32
            }
        };
        let dsm_height = |x: f64, y: f64| {
            if in_hole(x, y) {
                NODATA
            } else if in_box(x, y) {
                (ground(x, y) + 2.0) as f32
            } else {
                ground(x, y) as f32
            }
        };
        let top_left = BASE + DVec2::new(0.0, 16.0);
        write_dem(
            &dtm,
            &Dem {
                size: (200, 160),
                top_left,
                pixel: 0.1,
                epsg: 32617,
                height: &dtm_height,
            },
        );
        write_dem(
            &dsm,
            &Dem {
                size: (200, 160),
                top_left: top_left + DVec2::new(0.01, 0.01),
                pixel: 0.1,
                epsg: 32617,
                height: &dsm_height,
            },
        );
        write_ortho(
            &ortho,
            (201, 161),
            top_left + DVec2::new(-0.5, 0.5),
            0.1,
            true,
        );
        Self {
            dir,
            dsm,
            dtm,
            ortho,
        }
    }

    fn job(&self, name: &str, rules: CostRules) -> Job {
        Job {
            dsm: self.dsm.clone(),
            dtm: self.dtm.clone(),
            ortho: Some(self.ortho.clone()),
            overrides: None,
            out: self.dir.path().join(name),
            rules,
        }
    }

    fn write_overrides(&self) -> PathBuf {
        let path = self.dir.path().join("zones.geojson");
        let square = |x0: f64, y0: f64, x1: f64, y1: f64| {
            let (x0, x1, y0, y1) = (BASE.x + x0, BASE.x + x1, BASE.y + y0, BASE.y + y1);
            format!("[[[{x0},{y0}],[{x1},{y0}],[{x1},{y1}],[{x0},{y1}],[{x0},{y0}]]]")
        };
        let feature = |cost: u8, ring: String| {
            format!(
                r#"{{"type":"Feature","properties":{{"cost":{cost}}},"geometry":{{"type":"Polygon","coordinates":{ring}}}}}"#
            )
        };
        let json = format!(
            r#"{{"type":"FeatureCollection","features":[{},{}]}}"#,
            feature(254, square(6.0, 12.0, 7.0, 14.0)),
            feature(0, square(4.0, 10.0, 5.0, 11.0)),
        );
        std::fs::write(&path, json).unwrap();
        path
    }
}

struct Output {
    grid: TiledGrid<u8>,
    georef: GeoReference,
    size: glam::UVec2,
    resolution: f64,
}

impl Output {
    fn open(path: &Path) -> Self {
        let mut file = CostGeoTiff::open(path).unwrap();
        Self {
            grid: file.read_all(256).unwrap(),
            georef: file.georeference(),
            size: file.size(),
            resolution: file.resolution(),
        }
    }

    /// Cost at the projected position `(x, y)` metres east and north of `BASE`.
    fn at(&self, x: f64, y: f64) -> u8 {
        let local = self.georef.to_local(BASE + DVec2::new(x, y));
        *self.grid.get(self.grid.world_to_cell(local))
    }
}

#[test]
fn scene_features_get_their_costs() {
    let scene = Scene::new();
    let job = scene.job("cost.tif", CostRules::default());
    let summary = run(&job).unwrap();
    assert_eq!(summary.size, glam::UVec2::new(200, 160));
    assert_eq!(summary.epsg, 32617);
    assert!((summary.slope_box_m - 0.9).abs() < 1e-9, "{summary}");
    assert!((summary.slope_baseline_m - 0.8).abs() < 1e-9, "{summary}");
    assert_eq!(
        summary.counts.free
            + summary.counts.graded
            + summary.counts.lethal
            + summary.counts.unknown,
        200 * 160
    );

    let out = Output::open(&job.out);
    assert_eq!(out.size, glam::UVec2::new(200, 160));
    assert_eq!(out.georef.epsg, Some(32617));
    assert_eq!(out.resolution, 0.1);

    assert_eq!(out.at(5.55, 7.05), FREE, "flat ground");
    assert_eq!(out.at(14.05, 8.05), FREE, "plateau");
    assert_eq!(out.at(3.55, 11.55), LETHAL, "box");
    assert_eq!(
        out.at(10.05, 8.05),
        LETHAL,
        "ramp steeper than the lethal angle"
    );
    let foot = out.at(7.95, 8.05);
    assert!(
        (100..=252).contains(&foot),
        "ramp foot is graded, got {foot}"
    );
    assert_eq!(out.at(16.05, 3.05), UNKNOWN, "nodata hole");
    assert_eq!(out.at(1.55, 7.05), UNKNOWN, "transparent orthophoto");
    assert_eq!(out.at(5.05, 0.15), UNKNOWN, "outside the orthophoto");
    assert_eq!(
        out.at(17.55, 12.05),
        FREE,
        "green ground, vegetation rule off"
    );
}

impl Output {
    /// Every cost in row-major order, from local cell `(0, 0)`.
    fn cells(&self) -> Vec<u8> {
        (0..self.size.y as i32)
            .flat_map(|y| (0..self.size.x as i32).map(move |x| glam::IVec2::new(x, y)))
            .map(|cell| *self.grid.get(cell))
            .collect()
    }
}

/// Blocks of 37 cells do not divide the 200 x 160 scene (seams at x = 37, 74, 111, ... and
/// y = 37, 74, 111, ...). The box (x cells 30..50, y cells 100..120) and the 40 degree
/// ramp (x cells 80..120, whose slope window reaches 10 cells further) all straddle
/// seams, so wrong halos or block offsets would change the output.
#[test]
fn block_size_does_not_change_the_output() {
    let scene = Scene::new();
    let rules = lethal_vegetation();
    let single = scene.job("single_block.tif", rules.clone());
    let small = scene.job("small_blocks.tif", rules);
    let one_block = run_in_blocks(&single, 1024).unwrap();
    let many_blocks = run_in_blocks(&small, 37).unwrap();
    assert_eq!(one_block, many_blocks);

    let (single, small) = (Output::open(&single.out), Output::open(&small.out));
    assert_eq!(single.size, small.size);
    let cells = single.cells();
    let differing = cells
        .iter()
        .zip(small.cells())
        .filter(|&(&a, b)| a != b)
        .count();
    assert_eq!(differing, 0, "cells differ between block sizes");
    assert_eq!(small.at(3.55, 11.55), LETHAL, "box across the seam");
    assert_eq!(small.at(14.65, 7.05), LETHAL, "strip left of the seam");
    assert_eq!(small.at(14.85, 7.05), LETHAL, "strip right of the seam");
    assert_eq!(small.at(14.85, 12.05), FREE, "speck on the seam");
    assert_eq!(small.at(10.05, 8.05), LETHAL, "steep ramp across the seam");
    assert!(
        (100..=252).contains(&small.at(7.95, 8.05)),
        "graded ramp foot"
    );
}

#[test]
fn zero_block_size_is_an_error() {
    let scene = Scene::new();
    let job = scene.job("zero.tif", CostRules::default());
    assert!(matches!(run_in_blocks(&job, 0), Err(GeoError::Rules(_))));
    assert!(!job.out.exists());
}

#[test]
fn vegetation_rule_marks_green_areas_and_not_specks() {
    let scene = Scene::new();
    let job = scene.job("veg.tif", lethal_vegetation());
    let summary = run(&job).unwrap();
    let vegetation_box = summary.vegetation_box.unwrap();
    assert_eq!(vegetation_box.cells, 3);
    assert!((vegetation_box.metres - 0.3).abs() < 1e-9, "{summary}");
    assert!(
        summary
            .to_string()
            .contains("vegetation: 3 x 3 cells, 0.3 m box")
    );
    let out = Output::open(&job.out);
    assert_eq!(out.at(17.55, 12.05), LETHAL, "green patch");
    assert_eq!(
        out.at(16.05, 10.05),
        FREE,
        "corner cell: 4 of 9 samples green"
    );
    assert_eq!(
        out.at(16.05, 10.15),
        LETHAL,
        "edge cell: 6 of 9 samples green"
    );
    assert_eq!(out.at(15.95, 12.05), FREE, "just outside the green patch");
    assert_eq!(out.at(14.65, 7.05), LETHAL, "green strip");
    assert_eq!(out.at(14.55, 7.05), FREE, "just outside the green strip");
    assert_eq!(out.at(14.85, 12.05), FREE, "single green pixel");
    assert_eq!(out.at(5.55, 7.05), FREE, "grey ground");
    assert_eq!(out.at(3.55, 11.55), LETHAL, "box stays lethal");
    assert_eq!(out.at(16.05, 3.05), UNKNOWN, "hole stays unknown");
    assert_eq!(
        out.at(1.55, 7.05),
        UNKNOWN,
        "transparent orthophoto stays unknown"
    );
}

#[test]
fn one_cell_window_marks_the_single_green_pixel() {
    let scene = Scene::new();
    let rules = CostRules {
        vegetation_window_m: 0.0,
        ..lethal_vegetation()
    };
    let job = scene.job("veg_cell.tif", rules);
    let summary = run(&job).unwrap();
    assert_eq!(summary.vegetation_box.unwrap().cells, 1);
    let out = Output::open(&job.out);
    assert_eq!(out.at(14.85, 12.05), LETHAL, "single green pixel");
    assert_eq!(out.at(14.95, 12.05), FREE, "neighbour of the pixel");
    assert_eq!(out.at(17.55, 12.05), LETHAL, "green patch");
}

#[test]
fn graded_vegetation_cost_applies_to_green_cells() {
    let scene = Scene::new();
    let rules = CostRules {
        vegetation_cost: 100,
        ..lethal_vegetation()
    };
    let job = scene.job("veg_graded.tif", rules);
    run(&job).unwrap();
    assert_eq!(Output::open(&job.out).at(17.55, 12.05), 100);
}

#[test]
fn summary_has_no_vegetation_line_when_the_rule_is_off() {
    let scene = Scene::new();
    let summary = run(&scene.job("off.tif", CostRules::default())).unwrap();
    assert_eq!(summary.vegetation_box, None);
    assert!(!summary.to_string().contains("vegetation"));
}

#[test]
fn overrides_apply_in_file_order() {
    let scene = Scene::new();
    let mut job = scene.job("zones.tif", CostRules::default());
    job.overrides = Some(scene.write_overrides());
    run(&job).unwrap();
    let out = Output::open(&job.out);
    assert_eq!(out.at(6.55, 13.05), LETHAL, "keep-out zone on free ground");
    assert_eq!(
        out.at(4.55, 10.55),
        FREE,
        "later zone opens a cell of the box"
    );
    assert_eq!(out.at(3.55, 11.55), LETHAL, "rest of the box");
    assert_eq!(out.at(5.55, 7.05), FREE, "outside the zones");
}

#[test]
fn coarser_resolution_keeps_the_features() {
    let scene = Scene::new();
    let rules = CostRules {
        resolution: Some(0.2),
        ..CostRules::default()
    };
    let job = scene.job("coarse.tif", rules);
    let summary = run(&job).unwrap();
    assert_eq!(summary.size, glam::UVec2::new(100, 80));
    let out = Output::open(&job.out);
    assert_eq!(out.at(5.55, 7.05), FREE);
    assert_eq!(out.at(3.55, 11.55), LETHAL);
    assert_eq!(out.at(10.05, 8.05), LETHAL);
    assert_eq!(out.at(16.05, 3.05), UNKNOWN);
}

#[test]
fn run_without_an_orthophoto_leaves_alpha_cells_known() {
    let scene = Scene::new();
    let mut job = scene.job("no_ortho.tif", CostRules::default());
    job.ortho = None;
    run(&job).unwrap();
    let out = Output::open(&job.out);
    assert_eq!(out.at(1.55, 7.05), FREE);
    assert_eq!(out.at(5.05, 0.15), FREE);
    assert_eq!(out.at(16.05, 3.05), UNKNOWN);
}

#[test]
fn vegetation_cost_without_an_orthophoto_is_an_error() {
    let scene = Scene::new();
    let mut job = scene.job(
        "bad.tif",
        CostRules {
            vegetation_cost: 50,
            ..CostRules::default()
        },
    );
    job.ortho = None;
    assert!(matches!(run(&job), Err(GeoError::Rules(_))));
    assert!(!job.out.exists());
}

#[test]
fn vegetation_cost_255_is_rejected_before_anything_is_written() {
    let scene = Scene::new();
    let rules = CostRules {
        vegetation_cost: UNKNOWN,
        ..CostRules::default()
    };
    let job = scene.job("unknown_cost.tif", rules);
    assert!(matches!(run(&job), Err(GeoError::Rules(_))));
    assert!(!job.out.exists());
}

#[test]
fn mismatched_epsg_is_rejected_naming_both_files() {
    let scene = Scene::new();
    let other = scene.dir.path().join("other_crs.tif");
    write_dem(
        &other,
        &Dem {
            size: (20, 20),
            top_left: BASE + DVec2::new(0.0, 2.0),
            pixel: 0.1,
            epsg: 32618,
            height: &|_, _| 100.0,
        },
    );
    let mut job = scene.job("crs.tif", CostRules::default());
    job.dsm = other.clone();
    let message = run(&job).unwrap_err().to_string();
    assert!(
        message.contains("EPSG:32617") && message.contains("EPSG:32618"),
        "{message}"
    );
    assert!(message.contains("other_crs.tif"), "{message}");
}

#[test]
fn raster_without_georeferencing_is_rejected() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("plain.tif");
    {
        let mut encoder = TiffEncoder::new(BufWriter::new(File::create(&path).unwrap())).unwrap();
        encoder
            .write_image::<Gray32Float>(4, 4, &[0.0; 16])
            .unwrap();
    }
    let message = DemRaster::open(&path).err().unwrap().to_string();
    assert!(
        message.contains("plain.tif") && message.contains("ModelPixelScale"),
        "{message}"
    );
}

#[test]
fn wrong_layouts_are_rejected() {
    let scene = Scene::new();
    assert!(DemRaster::open(&scene.ortho).is_err());
    assert!(OrthoRaster::open(&scene.dtm).is_err());
}

#[test]
fn small_chunk_cache_gives_the_same_samples() {
    let scene = Scene::new();
    let mut small = DemRaster::open_with_cache(&scene.dtm, 1).unwrap();
    let mut large = DemRaster::open(&scene.dtm).unwrap();
    for i in 0..400 {
        let x = BASE.x + f64::from(i * 37 % 200) * 0.1 + 0.05;
        let y = BASE.y + f64::from(i * 91 % 160) * 0.1 + 0.05;
        let point = DVec2::new(x, y);
        assert_eq!(small.sample(point).unwrap(), large.sample(point).unwrap());
    }
    let ramp = small
        .sample(BASE + DVec2::new(10.05, 5.05))
        .unwrap()
        .unwrap();
    assert!((f64::from(ramp) - ground(10.05, 5.05)).abs() < 1e-3);
}

#[test]
fn samples_outside_nodata_and_transparent_are_none() {
    let scene = Scene::new();
    let mut dtm = DemRaster::open(&scene.dtm).unwrap();
    assert_eq!(dtm.sample(BASE + DVec2::new(-1.0, 5.0)).unwrap(), None);
    assert_eq!(dtm.sample(BASE + DVec2::new(5.0, 16.5)).unwrap(), None);
    assert_eq!(dtm.sample(BASE + DVec2::new(16.0, 3.0)).unwrap(), None);
    let mut ortho = OrthoRaster::open(&scene.ortho).unwrap();
    assert_eq!(ortho.sample(BASE + DVec2::new(1.5, 7.0)).unwrap(), None);
    assert_eq!(
        ortho.sample(BASE + DVec2::new(5.0, 7.0)).unwrap(),
        Some([150, 140, 120])
    );
    assert_eq!(
        ortho.sample(BASE + DVec2::new(17.0, 12.0)).unwrap(),
        Some([40, 160, 40])
    );
}

#[test]
fn three_band_orthophoto_is_accepted() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("rgb.tif");
    write_ortho(&path, (20, 20), BASE + DVec2::new(0.0, 2.0), 0.1, false);
    let mut ortho = OrthoRaster::open(&path).unwrap();
    assert_eq!(
        ortho.sample(BASE + DVec2::new(1.0, 1.0)).unwrap(),
        Some([150, 140, 120])
    );
}

#[test]
fn binary_writes_the_cost_map_and_prints_a_summary() {
    let scene = Scene::new();
    let out = scene.dir.path().join("cli.tif");
    let output = Command::new(env!("CARGO_BIN_EXE_costmap-from-ortho"))
        .args(["--dsm", scene.dsm.to_str().unwrap()])
        .args(["--dtm", scene.dtm.to_str().unwrap()])
        .args(["--ortho", scene.ortho.to_str().unwrap()])
        .args(["--out", out.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(stdout.contains("200 x 160 cells"), "{stdout}");
    assert!(stdout.contains("EPSG:32617"), "{stdout}");
    assert!(
        stdout.contains("slope: 0.9 m box, 0.8 m baseline"),
        "{stdout}"
    );
    assert!(
        stdout.contains("free")
            && stdout.contains("lethal or inscribed (253-254)")
            && stdout.contains("unknown"),
        "{stdout}"
    );
    assert!(output.stderr.is_empty());
    assert_eq!(Output::open(&out).at(3.55, 11.55), LETHAL);
}

#[test]
fn binary_reports_errors_on_stderr_with_a_failing_status() {
    let scene = Scene::new();
    let output = Command::new(env!("CARGO_BIN_EXE_costmap-from-ortho"))
        .args(["--dsm", "missing.tif"])
        .args(["--dtm", scene.dtm.to_str().unwrap()])
        .args(["--out", scene.dir.path().join("x.tif").to_str().unwrap()])
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.starts_with("error:") && stderr.contains("missing.tif"),
        "{stderr}"
    );
}
