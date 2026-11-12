#![cfg(feature = "geotiff")]

use std::{
    fs::{self, File},
    path::{Path, PathBuf},
    process::Command,
    sync::atomic::{AtomicU32, Ordering},
};

use glam::{DVec2, IVec2, UVec2};
use serde_json::Value;
use tiff::{
    decoder::{Decoder, DecodingResult},
    encoder::{TiffEncoder, colortype},
};

#[path = "common/gdal.rs"]
mod gdal;
use gdal::{gdal_available, run_gdal};

use costmap::{
    CostGeoTiff, DenseGrid, GeoReference, MapInfo, TiledGrid, VoxelError, write_cost_geotiff,
    write_cost_geotiff_region,
};

const UNKNOWN: u8 = 255;
const UTM_17N: GeoReference = GeoReference {
    epsg: Some(32617),
    anchor: DVec2::new(436_845.7, 4_572_650.3),
};

/// Directory under the system temp dir, removed on drop.
struct TempDir(PathBuf);

impl TempDir {
    fn new() -> Self {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let dir = std::env::temp_dir().join(format!(
            "costmap-geotiff-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&dir).unwrap();
        Self(dir)
    }

    fn file(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

/// Cost that differs between neighbouring cells and has no mirror symmetry; every 17th
/// cell on a diagonal is unknown and a few cells are lethal.
fn pattern(cell: IVec2) -> u8 {
    if (cell.x + cell.y).rem_euclid(17) == 0 {
        UNKNOWN
    } else if cell.x.rem_euclid(97) == 0 && cell.y.rem_euclid(89) == 0 {
        254
    } else {
        (cell.x * 3 + cell.y * 5).rem_euclid(200) as u8
    }
}

fn patterned_grid(resolution: f64, chunk_size: u32, min: IVec2, size: UVec2) -> TiledGrid<u8> {
    let mut grid = TiledGrid::new(resolution, chunk_size, UNKNOWN).unwrap();
    let info = MapInfo {
        width: size.x,
        height: size.y,
        resolution,
        origin: grid.cell_origin(min),
    };
    let source = DenseGrid::from_fn(info, |x, y| pattern(min + IVec2::new(x as i32, y as i32)));
    grid.write_window(&source).unwrap();
    grid
}

/// Data in chunks of both signs on both axes.
fn small_grid() -> TiledGrid<u8> {
    patterned_grid(0.25, 16, IVec2::new(-20, -9), UVec2::new(45, 40))
}

/// Wide enough that the writer adds overviews.
fn large_grid() -> TiledGrid<u8> {
    patterned_grid(0.05, 64, IVec2::new(-300, -100), UVec2::new(700, 400))
}

fn assert_same_cells(original: &TiledGrid<u8>, reread: &TiledGrid<u8>, size: UVec2) {
    let (min, _) = original.loaded_cell_bounds().unwrap();
    for y in 0..size.y as i32 {
        for x in 0..size.x as i32 {
            let cell = IVec2::new(x, y);
            assert_eq!(
                reread.get(cell),
                original.get(cell + min),
                "cell {cell} (original {})",
                cell + min
            );
        }
    }
}

fn bounds_size(grid: &TiledGrid<u8>) -> UVec2 {
    let (min, max) = grid.loaded_cell_bounds().unwrap();
    (max - min).as_uvec2()
}

#[test]
fn round_trip_preserves_every_cell() {
    let dir = TempDir::new();
    let path = dir.file("map.tif");
    let grid = small_grid();
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let mut tiff = CostGeoTiff::open(&path).unwrap();
    assert_eq!(tiff.size(), bounds_size(&grid));
    assert_eq!(tiff.resolution(), 0.25);
    for chunk_size in [16, 24, 1000] {
        let reread = tiff.read_all(chunk_size).unwrap();
        assert_eq!(reread.chunk_size(), chunk_size);
        assert_same_cells(&grid, &reread, tiff.size());
    }
}

#[test]
fn raster_rows_run_north_to_south() {
    let dir = TempDir::new();
    let path = dir.file("flip.tif");
    let mut grid = TiledGrid::new(1.0, 4, UNKNOWN).unwrap();
    for cell in [(0, 0), (1, 0), (2, 0), (3, 0)] {
        grid.set(IVec2::new(cell.0, cell.1), 10);
    }
    grid.set(IVec2::new(0, 0), 1);
    grid.set(IVec2::new(0, 3), 2);
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let mut decoder = Decoder::new(File::open(&path).unwrap()).unwrap();
    assert_eq!(decoder.dimensions().unwrap(), (4, 4));
    let DecodingResult::U8(raster) = decoder.read_image().unwrap() else {
        panic!("not a u8 raster");
    };
    // Row 0 is the northern row, which holds cell (0, 3); the southern row holds (0, 0).
    assert_eq!(raster[0], 2);
    assert_eq!(raster[3 * 4], 1);
    assert_eq!(&raster[3 * 4 + 1..], [10, 10, 10]);

    let reread = CostGeoTiff::open(&path).unwrap().read_all(4).unwrap();
    assert_eq!(*reread.get(IVec2::new(0, 3)), 2);
    assert_eq!(*reread.get(IVec2::new(0, 0)), 1);
}

#[test]
fn georeference_survives_at_utm_scale() {
    let dir = TempDir::new();
    let path = dir.file("utm.tif");
    let grid = small_grid();
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let mut tiff = CostGeoTiff::open(&path).unwrap();
    let georef = tiff.georeference();
    assert_eq!(georef.epsg, Some(32617));
    let reread = tiff.read_all(16).unwrap();
    let (min, max) = grid.loaded_cell_bounds().unwrap();
    for original in [min, max - 1, IVec2::new(0, 0), IVec2::new(7, -3)] {
        let shifted = original - min;
        let expected = UTM_17N.to_projected(grid.cell_center(original));
        let actual = georef.to_projected(reread.cell_center(shifted));
        assert!(
            (expected - actual).abs().max_element() < 1e-3,
            "{original}: {expected} vs {actual}"
        );
    }
}

#[test]
fn georeference_without_epsg_round_trips_as_none() {
    let dir = TempDir::new();
    let path = dir.file("no-epsg.tif");
    let georef = GeoReference {
        epsg: None,
        anchor: DVec2::new(10.0, 20.0),
    };
    write_cost_geotiff(&path, &small_grid(), &georef).unwrap();
    assert_eq!(CostGeoTiff::open(&path).unwrap().georeference().epsg, None);
}

#[test]
fn load_region_loads_only_intersecting_chunks_and_matches_read_all() {
    let dir = TempDir::new();
    let path = dir.file("region.tif");
    let grid = patterned_grid(0.1, 32, IVec2::ZERO, UVec2::new(320, 224));
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let mut tiff = CostGeoTiff::open(&path).unwrap();
    let everything = tiff.read_all(32).unwrap();

    let mut partial = TiledGrid::new(0.1, 32, UNKNOWN).unwrap();
    let loaded = tiff
        .load_region(&mut partial, IVec2::new(40, 50), IVec2::new(100, 70))
        .unwrap();
    // Cells 40..100 touch chunks 1..=3 on x, 50..70 touch chunks 1..=2 on y.
    assert_eq!(loaded, 6);
    let mut coords: Vec<_> = partial.loaded_chunks().map(|c| (c.x, c.y)).collect();
    coords.sort_unstable();
    assert_eq!(coords, [(1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (3, 2)]);
    for y in 32..96 {
        for x in 32..128 {
            let cell = IVec2::new(x, y);
            assert_eq!(partial.get(cell), everything.get(cell), "cell {cell}");
        }
    }

    assert_eq!(
        tiff.load_region(&mut partial, IVec2::new(40, 50), IVec2::new(100, 70))
            .unwrap(),
        0
    );
    let more = tiff
        .load_region(&mut partial, IVec2::new(0, 0), IVec2::new(33, 33))
        .unwrap();
    assert_eq!(more, 3);
}

#[test]
fn load_region_outside_raster_loads_nothing() {
    let dir = TempDir::new();
    let path = dir.file("outside.tif");
    write_cost_geotiff(&path, &small_grid(), &UTM_17N).unwrap();
    let mut tiff = CostGeoTiff::open(&path).unwrap();
    let mut grid = TiledGrid::new(0.25, 16, UNKNOWN).unwrap();
    let size = tiff.size().as_ivec2();
    for (min, max) in [
        (size, size + 50),
        (IVec2::splat(-50), IVec2::ZERO),
        (IVec2::new(5, 5), IVec2::new(5, 9)),
    ] {
        assert_eq!(tiff.load_region(&mut grid, min, max).unwrap(), 0);
    }
    assert_eq!(grid.loaded_chunks().count(), 0);
}

#[test]
fn read_all_skips_chunks_that_are_entirely_unknown() {
    let dir = TempDir::new();
    let path = dir.file("sparse.tif");
    let mut grid = TiledGrid::new(0.5, 8, UNKNOWN).unwrap();
    grid.set(IVec2::new(0, 0), 5);
    grid.set(IVec2::new(30, 30), 6);
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let reread = CostGeoTiff::open(&path).unwrap().read_all(8).unwrap();
    let mut coords: Vec<_> = reread.loaded_chunks().map(|c| (c.x, c.y)).collect();
    coords.sort_unstable();
    assert_eq!(coords, [(0, 0), (3, 3)]);
}

#[test]
fn load_region_rejects_mismatched_grids() {
    let dir = TempDir::new();
    let path = dir.file("mismatch.tif");
    write_cost_geotiff(&path, &small_grid(), &UTM_17N).unwrap();
    let mut tiff = CostGeoTiff::open(&path).unwrap();

    let mut wrong_resolution = TiledGrid::new(0.5, 16, UNKNOWN).unwrap();
    let mut wrong_fill = TiledGrid::new(0.25, 16, 0u8).unwrap();
    for grid in [&mut wrong_resolution, &mut wrong_fill] {
        assert!(matches!(
            tiff.load_region(grid, IVec2::ZERO, IVec2::splat(8)),
            Err(VoxelError::InvalidArgument(_))
        ));
    }
}

#[test]
fn region_write_is_not_padded_to_whole_chunks() {
    let dir = TempDir::new();
    let path = dir.file("region.tif");
    let grid = small_grid();
    let (min, max) = (IVec2::new(-20, -9), IVec2::new(25, 31));
    assert_ne!(bounds_size(&grid), UVec2::new(45, 40));
    write_cost_geotiff_region(&path, &grid, &UTM_17N, min, max).unwrap();

    let mut tiff = CostGeoTiff::open(&path).unwrap();
    assert_eq!(tiff.size(), UVec2::new(45, 40));
    let reread = tiff.read_all(16).unwrap();
    for y in 0..40 {
        for x in 0..45 {
            let cell = IVec2::new(x, y);
            assert_eq!(reread.get(cell), grid.get(cell + min), "cell {cell}");
        }
    }
}

#[test]
fn writing_an_empty_region_fails() {
    let dir = TempDir::new();
    let grid = small_grid();
    let result = write_cost_geotiff_region(
        dir.file("empty.tif"),
        &grid,
        &UTM_17N,
        IVec2::ZERO,
        IVec2::new(0, 5),
    );
    assert!(matches!(result, Err(VoxelError::InvalidArgument(_))));
}

#[test]
fn writing_an_empty_grid_fails() {
    let dir = TempDir::new();
    let grid = TiledGrid::new(0.05, 16, UNKNOWN).unwrap();
    assert!(matches!(
        write_cost_geotiff(dir.file("empty.tif"), &grid, &UTM_17N),
        Err(VoxelError::InvalidArgument(_))
    ));
}

#[test]
fn writing_an_epsg_code_beyond_16_bits_fails() {
    let dir = TempDir::new();
    let georef = GeoReference {
        epsg: Some(100_000),
        anchor: DVec2::ZERO,
    };
    assert!(matches!(
        write_cost_geotiff(dir.file("big-epsg.tif"), &small_grid(), &georef),
        Err(VoxelError::Unsupported(_))
    ));
}

#[test]
fn open_rejects_other_sample_formats_and_band_counts() {
    let dir = TempDir::new();

    let float = dir.file("float.tif");
    let mut encoder = TiffEncoder::new(File::create(&float).unwrap()).unwrap();
    encoder
        .write_image::<colortype::Gray32Float>(2, 2, &[0.0; 4])
        .unwrap();
    assert!(matches!(
        CostGeoTiff::open(&float),
        Err(VoxelError::Unsupported(_))
    ));

    let rgb = dir.file("rgb.tif");
    let mut encoder = TiffEncoder::new(File::create(&rgb).unwrap()).unwrap();
    encoder
        .write_image::<colortype::RGB8>(2, 2, &[0; 12])
        .unwrap();
    assert!(matches!(
        CostGeoTiff::open(&rgb),
        Err(VoxelError::Unsupported(_))
    ));
}

#[test]
fn open_rejects_unreferenced_and_non_tiff_files() {
    let dir = TempDir::new();
    let plain = dir.file("plain.tif");
    let mut encoder = TiffEncoder::new(File::create(&plain).unwrap()).unwrap();
    encoder
        .write_image::<colortype::Gray8>(2, 2, &[0; 4])
        .unwrap();
    assert!(matches!(
        CostGeoTiff::open(&plain),
        Err(VoxelError::InvalidMetadata(_))
    ));

    let text = dir.file("text.tif");
    fs::write(&text, b"not a tiff").unwrap();
    assert!(CostGeoTiff::open(&text).is_err());
    assert!(matches!(
        CostGeoTiff::open(dir.file("missing.tif")),
        Err(VoxelError::Io(_))
    ));
}

fn gdalinfo_json(path: &Path) -> Value {
    let output = Command::new("gdalinfo")
        .arg("-json")
        .arg(path)
        .output()
        .unwrap();
    assert!(output.status.success());
    serde_json::from_slice(&output.stdout).unwrap()
}

fn as_f64s(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect()
}

#[test]
fn gdalinfo_reads_the_written_file() {
    if !gdal_available("gdalinfo_reads_the_written_file") {
        return;
    }
    let dir = TempDir::new();
    let path = dir.file("large.tif");
    let grid = large_grid();
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let info = gdalinfo_json(&path);
    let size = bounds_size(&grid);
    assert_eq!(
        as_f64s(&info["size"]),
        [f64::from(size.x), f64::from(size.y)]
    );

    let (min, max) = grid.loaded_cell_bounds().unwrap();
    let res = grid.resolution();
    let transform = as_f64s(&info["geoTransform"]);
    let expected_x = UTM_17N.anchor.x + f64::from(min.x) * res;
    let expected_y = UTM_17N.anchor.y + f64::from(max.y) * res;
    assert!((transform[0] - expected_x).abs() < 1e-6, "{transform:?}");
    assert!((transform[3] - expected_y).abs() < 1e-6, "{transform:?}");
    assert!((transform[1] - res).abs() < 1e-9);
    assert!((transform[5] + res).abs() < 1e-9);

    let wkt = info["coordinateSystem"]["wkt"].as_str().unwrap();
    assert!(wkt.contains("32617"), "{wkt}");
    assert!(wkt.contains("UTM zone 17N"), "{wkt}");

    let band = &info["bands"][0];
    assert_eq!(band["type"], "Byte");
    assert_eq!(band["noDataValue"], 255);
    let overviews = band["overviews"].as_array().unwrap();
    assert_eq!(overviews.len(), 2, "{overviews:?}");
    assert_eq!(as_f64s(&overviews[0]["size"]), [384.0, 224.0]);
    assert_eq!(as_f64s(&overviews[1]["size"]), [192.0, 112.0]);
}

#[test]
fn gdal_reads_flipped_values_at_projected_coordinates() {
    if !gdal_available("gdal_reads_flipped_values_at_projected_coordinates") {
        return;
    }
    let dir = TempDir::new();
    let path = dir.file("location.tif");
    let grid = small_grid();
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let probes = [IVec2::new(-20, -9), IVec2::new(24, 30), IVec2::new(-5, 17)];
    for cell in probes {
        let expected = *grid.get(cell);
        if expected == UNKNOWN {
            continue;
        }
        let projected = UTM_17N.to_projected(grid.cell_center(cell));
        let output = Command::new("gdallocationinfo")
            .args(["-valonly", "-geoloc"])
            .arg(&path)
            .arg(projected.x.to_string())
            .arg(projected.y.to_string())
            .output()
            .unwrap();
        let value = String::from_utf8_lossy(&output.stdout);
        assert_eq!(value.trim(), expected.to_string(), "cell {cell}");
    }
}

#[test]
fn gdal_converted_files_load_back_cell_for_cell() {
    if !gdal_available("gdal_converted_files_load_back_cell_for_cell") {
        return;
    }
    let dir = TempDir::new();
    let source = dir.file("source.tif");
    let grid = large_grid();
    write_cost_geotiff(&source, &grid, &UTM_17N).unwrap();

    let cog = dir.file("cog.tif");
    run_gdal(
        "gdal_translate",
        &[Path::new("-of"), Path::new("COG"), &source, &cog],
    );
    let lzw = dir.file("lzw.tif");
    run_gdal(
        "gdal_translate",
        &[
            Path::new("-of"),
            Path::new("GTiff"),
            Path::new("-co"),
            Path::new("COMPRESS=LZW"),
            &source,
            &lzw,
        ],
    );
    let tiled = dir.file("tiled.tif");
    run_gdal(
        "gdal_translate",
        &[
            Path::new("-of"),
            Path::new("GTiff"),
            Path::new("-co"),
            Path::new("TILED=YES"),
            Path::new("-co"),
            Path::new("BLOCKXSIZE=128"),
            Path::new("-co"),
            Path::new("BLOCKYSIZE=64"),
            &source,
            &tiled,
        ],
    );

    let size = bounds_size(&grid);
    for converted in [&cog, &lzw, &tiled] {
        let mut tiff = CostGeoTiff::open(converted).unwrap();
        assert_eq!(tiff.size(), size, "{converted:?}");
        assert_eq!(tiff.georeference().epsg, Some(32617));
        let anchor = tiff.georeference().anchor;
        let (min, _) = grid.loaded_cell_bounds().unwrap();
        let expected = UTM_17N.anchor + min.as_dvec2() * grid.resolution();
        assert!(
            (anchor - expected).abs().max_element() < 1e-6,
            "{converted:?}"
        );
        for chunk_size in [64, 100] {
            let reread = tiff.read_all(chunk_size).unwrap();
            assert_same_cells(&grid, &reread, size);
        }
    }

    let cog = CostGeoTiff::open(&cog).unwrap();
    assert_eq!(cog.overview_factors(), [2, 4]);
    let overview = cog.overview(0).unwrap();
    assert_eq!(overview.size(), size.map(|s| s.div_ceil(2)));
}

#[test]
fn raster_nodata_other_than_255_reads_as_unknown() {
    if !gdal_available("raster_nodata_other_than_255_reads_as_unknown") {
        return;
    }
    let dir = TempDir::new();
    let source = dir.file("source.tif");
    let grid = small_grid();
    write_cost_geotiff(&source, &grid, &UTM_17N).unwrap();
    let zero_nodata = dir.file("nodata0.tif");
    run_gdal(
        "gdal_translate",
        &[
            Path::new("-a_nodata"),
            Path::new("0"),
            &source,
            &zero_nodata,
        ],
    );

    let mut tiff = CostGeoTiff::open(&zero_nodata).unwrap();
    let reread = tiff.read_all(16).unwrap();
    let (min, _) = grid.loaded_cell_bounds().unwrap();
    let mut zeros = 0;
    for y in 0..tiff.size().y as i32 {
        for x in 0..tiff.size().x as i32 {
            let original = *grid.get(IVec2::new(x, y) + min);
            let expected = if original == 0 { UNKNOWN } else { original };
            zeros += usize::from(original == 0);
            assert_eq!(*reread.get(IVec2::new(x, y)), expected);
        }
    }
    assert!(zeros > 0, "pattern has no zero cells to test with");
}

/// 8x8 blocks of pseudo-random cost with scattered unknown cells: compresses like a real
/// cost map but not so well that tiles become trivial.
fn blocky_noise_grid(side: u32, seed: u64) -> TiledGrid<u8> {
    let mix = |mut z: u64| {
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    };
    let mut grid = TiledGrid::new(1.0, 64, UNKNOWN).unwrap();
    let info = MapInfo {
        width: side,
        height: side,
        resolution: 1.0,
        origin: grid.cell_origin(IVec2::ZERO),
    };
    let source = DenseGrid::from_fn(info, |x, y| {
        let cell = u64::from(y) * u64::from(side) + u64::from(x);
        if mix(seed ^ cell.wrapping_mul(0x9e37_79b9)) % 20 == 0 {
            UNKNOWN
        } else {
            let block = u64::from(y / 8) * u64::from(side) + u64::from(x / 8);
            (mix(seed.wrapping_add(block)) % 255) as u8
        }
    });
    grid.write_window(&source).unwrap();
    grid
}

/// The tiff crate's LZW reader rejects some valid GDAL LZW tiles (partial edge tiles of a
/// COG, for example) with "no lzw end code found". Such files must fail with advice to
/// re-encode, while the same raster with Deflate must load unchanged.
#[test]
fn undecodable_lzw_tile_reports_the_deflate_workaround() {
    if !gdal_available("undecodable_lzw_tile_reports_the_deflate_workaround") {
        return;
    }
    let dir = TempDir::new();
    let source = dir.file("source.tif");
    let grid = blocky_noise_grid(900, 0);
    write_cost_geotiff(&source, &grid, &UTM_17N).unwrap();
    let size = bounds_size(&grid);

    let lzw = dir.file("lzw_cog.tif");
    run_gdal(
        "gdal_translate",
        &[Path::new("-of"), Path::new("COG"), &source, &lzw],
    );
    let deflate = dir.file("deflate_cog.tif");
    run_gdal(
        "gdal_translate",
        &[
            Path::new("-of"),
            Path::new("COG"),
            Path::new("-co"),
            Path::new("COMPRESS=DEFLATE"),
            &source,
            &deflate,
        ],
    );

    let reread = CostGeoTiff::open(&deflate).unwrap().read_all(64).unwrap();
    assert_same_cells(&grid, &reread, size);

    let message = match CostGeoTiff::open(&lzw).unwrap().read_all(64) {
        Err(VoxelError::Unsupported(message)) => message,
        other => panic!("expected an Unsupported error, got {:?}", other.map(|_| ())),
    };
    assert!(message.contains("LZW"), "{message}");
    assert!(message.contains("COMPRESS=DEFLATE"), "{message}");
}

/// Max of the known cells of `grid` in the `factor` x `factor` block of raster pixels
/// whose top-left pixel is `(column, row)`, counting rows from the northern edge of the
/// `size` raster written from `grid`.
fn raster_block_max(grid: &TiledGrid<u8>, size: UVec2, factor: u32, column: u32, row: u32) -> u8 {
    let (min, _) = grid.loaded_cell_bounds().unwrap();
    let mut max = UNKNOWN;
    for r in row * factor..((row + 1) * factor).min(size.y) {
        for c in column * factor..((column + 1) * factor).min(size.x) {
            let cell = min + IVec2::new(c as i32, (size.y - 1 - r) as i32);
            let value = *grid.get(cell);
            if value != UNKNOWN && (max == UNKNOWN || value > max) {
                max = value;
            }
        }
    }
    max
}

#[test]
fn written_overviews_are_listed_and_read_as_max_pooled_levels() {
    let dir = TempDir::new();
    let path = dir.file("overviews.tif");
    let grid = large_grid();
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();
    let base = CostGeoTiff::open(&path).unwrap();
    let size = base.size();
    assert_eq!(base.overview_factors(), [2, 4]);

    let top = |t: &CostGeoTiff| t.georeference().anchor.y + f64::from(t.size().y) * t.resolution();
    for (index, factor) in base.overview_factors().into_iter().enumerate() {
        let mut overview = base.overview(index).unwrap();
        assert_eq!(overview.size(), size.map(|s| s.div_ceil(factor)));
        assert!((overview.resolution() - base.resolution() * f64::from(factor)).abs() < 1e-12);
        assert!(overview.overview_factors().is_empty());
        assert!((top(&overview) - top(&base)).abs() < 1e-3);
        assert_eq!(
            overview.georeference().anchor.x,
            base.georeference().anchor.x
        );

        let reread = overview.read_all(32).unwrap();
        let rows = overview.size().y;
        for row in 0..rows {
            for column in 0..overview.size().x {
                let cell = IVec2::new(column as i32, (rows - 1 - row) as i32);
                assert_eq!(
                    *reread.get(cell),
                    raster_block_max(&grid, size, factor, column, row),
                    "factor {factor}, pixel ({column}, {row})"
                );
            }
        }
    }
    assert!(matches!(
        base.overview(2),
        Err(VoxelError::InvalidArgument(_))
    ));
}

#[test]
fn small_files_have_no_overviews() {
    let dir = TempDir::new();
    let path = dir.file("small.tif");
    write_cost_geotiff(&path, &small_grid(), &UTM_17N).unwrap();
    assert!(
        CostGeoTiff::open(&path)
            .unwrap()
            .overview_factors()
            .is_empty()
    );
}

#[test]
fn streamed_overview_matches_pooling_the_full_grid() {
    let dir = TempDir::new();
    let mut files = vec![dir.file("stripped.tif")];
    write_cost_geotiff(&files[0], &large_grid(), &UTM_17N).unwrap();
    if gdal_available("streamed_overview_matches_pooling_the_full_grid") {
        let tiled = dir.file("tiled.tif");
        run_gdal(
            "gdal_translate",
            &[
                Path::new("-co"),
                Path::new("TILED=YES"),
                Path::new("-co"),
                Path::new("BLOCKXSIZE=96"),
                Path::new("-co"),
                Path::new("BLOCKYSIZE=48"),
                &files[0],
                &tiled,
            ],
        );
        files.push(tiled);
    }

    for path in &files {
        let mut tiff = CostGeoTiff::open(path).unwrap();
        let full = tiff.read_all(64).unwrap();
        for factor in [1, 3, 8] {
            let streamed = tiff.read_overview_max(factor, 16).unwrap();
            let expected = full.overview_max(factor).unwrap();
            assert_eq!(streamed.resolution(), expected.resolution());
            let blocks = tiff.size().map(|s| s.div_ceil(factor)).as_ivec2();
            for y in -1..=blocks.y {
                for x in -1..=blocks.x {
                    let cell = IVec2::new(x, y);
                    assert_eq!(
                        streamed.get(cell),
                        expected.get(cell),
                        "{path:?} factor {factor} block {cell}"
                    );
                }
            }
        }
        assert!(tiff.read_overview_max(0, 16).is_err());
    }
}

/// Copy of `source` without overviews, made by GDAL.
fn without_overviews(dir: &TempDir, source: &Path, name: &str) -> PathBuf {
    let out = dir.file(name);
    run_gdal("gdal_translate", &[source, &out]);
    out
}

/// Run `gdaladdo -r <resampling>` on `path` with the given factors.
fn add_gdal_overviews(path: &Path, resampling: &str, factors: &[u32]) {
    let factors: Vec<PathBuf> = factors
        .iter()
        .map(|f| PathBuf::from(f.to_string()))
        .collect();
    let mut args = vec![
        Path::new("-q"),
        Path::new("-r"),
        Path::new(resampling),
        path,
    ];
    args.extend(factors.iter().map(PathBuf::as_path));
    run_gdal("gdaladdo", &args);
}

#[test]
fn gdal_overview_factors_satisfy_both_axes() {
    if !gdal_available("gdal_overview_factors_satisfy_both_axes") {
        return;
    }
    let dir = TempDir::new();
    let source = dir.file("source.tif");
    let size = UVec2::new(1000, 789);
    let grid = patterned_grid(0.05, 64, IVec2::ZERO, size);
    write_cost_geotiff_region(&source, &grid, &UTM_17N, IVec2::ZERO, size.as_ivec2()).unwrap();
    let path = without_overviews(&dir, &source, "plain.tif");
    add_gdal_overviews(&path, "nearest", &[2, 3, 64]);

    let tiff = CostGeoTiff::open(&path).unwrap();
    // The 64 level is 16 x 13. Factors 63 to 65 all give that size; 1000 / 16 rounds to
    // 63, and the power of two that GDAL was asked for is 64.
    assert_eq!(tiff.overview_factors(), [2, 3, 64]);
    let coarsest = tiff.overview(2).unwrap();
    assert_eq!(coarsest.size(), UVec2::new(16, 13));
    assert!((coarsest.resolution() - 0.05 * 64.0).abs() < 1e-12);
}

/// Free ground with a one-cell lethal fence down column 500, written by this crate.
fn write_fence_map(path: &Path) {
    let size = UVec2::new(1000, 500);
    let mut grid = TiledGrid::new(0.1, 100, UNKNOWN).unwrap();
    let info = MapInfo {
        width: size.x,
        height: size.y,
        resolution: 0.1,
        origin: glam::DVec2::ZERO,
    };
    grid.write_window(&DenseGrid::from_fn(
        info,
        |x, _| if x == 500 { 254 } else { 0 },
    ))
    .unwrap();
    write_cost_geotiff_region(path, &grid, &UTM_17N, IVec2::ZERO, size.as_ivec2()).unwrap();
}

/// For each overview of `path`: its factor, its recorded resampling, whether it is
/// reported max-pooled, and whether the fence survives in it.
fn overview_report(path: &Path) -> Vec<(u32, Option<String>, bool, bool)> {
    let tiff = CostGeoTiff::open(path).unwrap();
    (0..tiff.overview_factors().len())
        .map(|index| {
            let mut level = tiff.overview(index).unwrap();
            let size = level.size();
            let lethal = level.read_all(64).unwrap();
            let has_fence = (0..size.x as i32).any(|x| *lethal.get(IVec2::new(x, 10)) == 254);
            (
                tiff.overview_factors()[index],
                tiff.overview_resampling(index).map(str::to_owned),
                tiff.overview_is_max_pooled(index),
                has_fence,
            )
        })
        .collect()
}

#[test]
fn written_overviews_are_reported_max_pooled() {
    let dir = TempDir::new();
    let path = dir.file("fence.tif");
    write_fence_map(&path);
    let max = Some("MAX".to_string());
    assert_eq!(
        overview_report(&path),
        [(2, max.clone(), true, true), (4, max, true, true)]
    );
}

#[test]
fn gdal_averaged_overviews_are_not_reported_max_pooled() {
    if !gdal_available("gdal_averaged_overviews_are_not_reported_max_pooled") {
        return;
    }
    let dir = TempDir::new();
    let ours = dir.file("fence.tif");
    write_fence_map(&ours);

    // Rebuilding the levels of a file this crate wrote replaces their metadata.
    let rebuilt = dir.file("rebuilt.tif");
    fs::copy(&ours, &rebuilt).unwrap();
    add_gdal_overviews(&rebuilt, "average", &[2, 4]);
    let average = Some("AVERAGE".to_string());
    assert_eq!(
        overview_report(&rebuilt),
        [
            (2, average.clone(), false, false),
            (4, average.clone(), false, false)
        ]
    );

    let plain = without_overviews(&dir, &ours, "plain.tif");
    add_gdal_overviews(&plain, "average", &[2, 4, 8]);
    assert_eq!(
        overview_report(&plain),
        [
            (2, average.clone(), false, false),
            (4, average.clone(), false, false),
            (8, average, false, false)
        ]
    );
}

#[test]
fn gdal_cog_conversion_drops_the_max_pooled_marker() {
    if !gdal_available("gdal_cog_conversion_drops_the_max_pooled_marker") {
        return;
    }
    let dir = TempDir::new();
    let ours = dir.file("fence.tif");
    write_fence_map(&ours);
    let cog = |name: &str, options: &[&str]| {
        let out = dir.file(name);
        let mut args = vec![Path::new("-of"), Path::new("COG")];
        for option in options {
            args.extend([Path::new("-co"), Path::new(option)]);
        }
        args.extend([ours.as_path(), out.as_path()]);
        run_gdal("gdal_translate", &args);
        overview_report(&out)
    };

    // The COG driver writes no RESAMPLING item on the levels it writes, whether it
    // resamples them or copies the existing ones.
    let regenerated = cog("regenerated.tif", &["COMPRESS=DEFLATE"]);
    let copied = cog(
        "copied.tif",
        &["COMPRESS=DEFLATE", "OVERVIEWS=FORCE_USE_EXISTING"],
    );
    for report in [regenerated, copied] {
        let levels: Vec<_> = report.iter().map(|level| (level.0, level.2)).collect();
        assert_eq!(levels, [(2, false), (4, false)], "{report:?}");
    }
}

#[test]
fn load_region_does_not_decode_unknown_chunks_twice() {
    let dir = TempDir::new();
    let path = dir.file("empty_middle.tif");
    let mut grid = TiledGrid::new(0.5, 8, UNKNOWN).unwrap();
    grid.set(IVec2::new(0, 0), 5);
    grid.set(IVec2::new(63, 63), 6);
    write_cost_geotiff(&path, &grid, &UTM_17N).unwrap();

    let mut tiff = CostGeoTiff::open(&path).unwrap();
    let mut tiles = TiledGrid::new(0.5, 8, UNKNOWN).unwrap();
    let (min, max) = (IVec2::new(16, 16), IVec2::new(40, 40));
    assert_eq!(tiff.load_region(&mut tiles, min, max).unwrap(), 0);

    // With the file emptied, any further decode fails, so success means nothing was decoded.
    File::create(&path).unwrap();
    assert_eq!(tiff.load_region(&mut tiles, min, max).unwrap(), 0);
    assert!(
        tiff.load_region(&mut tiles, IVec2::ZERO, IVec2::ONE)
            .is_err()
    );
}

/// Copy of `source` whose pixels are `pixel` metres wide and tall, with the same top-left
/// corner, made by assigning new bounds with GDAL.
fn with_pixel_size(dir: &TempDir, source: &Path, pixel: DVec2) -> PathBuf {
    let tiff = CostGeoTiff::open(source).unwrap();
    let lower_left = tiff.georeference().anchor;
    let top = lower_left.y + f64::from(tiff.size().y) * tiff.resolution();
    let bounds = [
        lower_left.x,
        top,
        lower_left.x + f64::from(tiff.size().x) * pixel.x,
        top - f64::from(tiff.size().y) * pixel.y,
    ]
    .map(|v| PathBuf::from(format!("{v:.9}")));
    let out = dir.file("resized.tif");
    run_gdal(
        "gdal_translate",
        &[
            Path::new("-a_ullr"),
            &bounds[0],
            &bounds[1],
            &bounds[2],
            &bounds[3],
            source,
            &out,
        ],
    );
    out
}

#[test]
fn open_rejects_pixels_that_drift_apart_across_the_raster() {
    if !gdal_available("open_rejects_pixels_that_drift_apart_across_the_raster") {
        return;
    }
    let dir = TempDir::new();
    let source = dir.file("source.tif");
    let size = UVec2::new(400, 6000);
    let grid = patterned_grid(0.05, 20, IVec2::ZERO, size);
    write_cost_geotiff(&source, &grid, &UTM_17N).unwrap();
    let tall = with_pixel_size(&dir, &source, DVec2::new(0.05, 0.04996));

    let message = match CostGeoTiff::open(&tall) {
        Err(VoxelError::Unsupported(message)) => message,
        other => panic!("expected an Unsupported error, got {:?}", other.map(|_| ())),
    };
    assert!(message.contains("0.05 x 0.04996"), "{message}");
    assert!(message.contains("gdalwarp -tr"), "{message}");
}

#[test]
fn open_accepts_pixels_within_a_hundredth_of_a_cell_and_matches_gdal() {
    if !gdal_available("open_accepts_pixels_within_a_hundredth_of_a_cell_and_matches_gdal") {
        return;
    }
    let dir = TempDir::new();
    let source = dir.file("source.tif");
    let size = UVec2::new(400, 300);
    let grid = patterned_grid(0.05, 20, IVec2::ZERO, size);
    write_cost_geotiff(&source, &grid, &UTM_17N).unwrap();
    let nearly_square = with_pixel_size(&dir, &source, DVec2::new(0.05, 0.050_000_1));

    let mut tiff = CostGeoTiff::open(&nearly_square).unwrap();
    let reread = tiff.read_all(64).unwrap();
    let georef = tiff.georeference();
    let mut mismatches = 0;
    for i in 0..400_i32 {
        let cell = IVec2::new((i * 37) % 400, (i * 101 + i / 7) % 300);
        let projected = georef.to_projected(reread.cell_center(cell));
        let output = Command::new("gdallocationinfo")
            .args(["-valonly", "-geoloc"])
            .arg(&nearly_square)
            .arg(format!("{:.9}", projected.x))
            .arg(format!("{:.9}", projected.y))
            .output()
            .unwrap();
        let expected = String::from_utf8_lossy(&output.stdout);
        mismatches += usize::from(expected.trim() != reread.get(cell).to_string());
    }
    assert_eq!(mismatches, 0);
}
