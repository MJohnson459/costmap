//! Sampling of rasters whose x and y pixel sizes differ slightly.

#[path = "../../../tests/common/gdal.rs"]
mod gdal;

use std::{
    fs::File,
    io::BufWriter,
    path::{Path, PathBuf},
    process::Command,
};

use costmap::VoxelError;
use costmap_geo::{GeoError, raster::DemRaster};
use glam::DVec2;
use tempfile::TempDir;
use tiff::{
    encoder::{Compression, DeflateLevel, TiffEncoder, colortype::Gray32Float},
    tags::Tag,
};

const WIDTH: u32 = 400;
const HEIGHT: u32 = 6000;
const PIXEL: DVec2 = DVec2::new(0.05, 0.04996);
const TOP_LEFT: DVec2 = DVec2::new(500_000.0, 4_500_300.0);

/// Float32 raster whose value at each pixel is its own row number, so a lookup shows
/// which row was sampled.
fn write_row_index_dem(path: &Path) {
    let data: Vec<f32> = (0..HEIGHT)
        .flat_map(|row| std::iter::repeat_n(row as f32, WIDTH as usize))
        .collect();
    let file = BufWriter::new(File::create(path).unwrap());
    let mut encoder = TiffEncoder::new(file)
        .unwrap()
        .with_compression(Compression::Deflate(DeflateLevel::Fast));
    let mut image = encoder.new_image::<Gray32Float>(WIDTH, HEIGHT).unwrap();
    image.rows_per_strip(64).unwrap();
    let tags = image.encoder();
    tags.write_tag(Tag::ModelPixelScaleTag, &[PIXEL.x, PIXEL.y, 0.0][..])
        .unwrap();
    tags.write_tag(
        Tag::ModelTiepointTag,
        &[0.0, 0.0, 0.0, TOP_LEFT.x, TOP_LEFT.y, 0.0][..],
    )
    .unwrap();
    let keys: [u16; 16] = [1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 32617];
    tags.write_tag(Tag::GeoKeyDirectoryTag, &keys[..]).unwrap();
    image.write_data(&data).unwrap();
}

fn pixel_centre(column: u32, row: u32) -> DVec2 {
    TOP_LEFT + DVec2::new(f64::from(column) + 0.5, -(f64::from(row) + 0.5)) * PIXEL
}

#[test]
fn anisotropic_dem_is_sampled_with_the_y_pixel_size() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("dem.tif");
    write_row_index_dem(&path);
    let mut dem = DemRaster::open(&path).unwrap();

    for row in [0, 1, 2999, 5990, 5999] {
        let value = dem.sample(pixel_centre(7, row)).unwrap();
        assert_eq!(value, Some(row as f32), "row {row}");
    }
    let (lower_left, upper_right) = dem.extent();
    let expected = TOP_LEFT - DVec2::new(0.0, f64::from(HEIGHT) * PIXEL.y);
    assert!((lower_left - expected).abs().max_element() < 1e-9);
    assert!((upper_right.x - (TOP_LEFT.x + f64::from(WIDTH) * PIXEL.x)).abs() < 1e-9);
    let below_last_row = pixel_centre(0, HEIGHT - 1) - DVec2::new(0.0, PIXEL.y);
    assert_eq!(dem.sample(below_last_row).unwrap(), None);
}

#[test]
fn anisotropic_dem_agrees_with_gdal_at_random_points() {
    if !gdal::gdal_available("anisotropic_dem_agrees_with_gdal_at_random_points") {
        return;
    }
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("dem.tif");
    write_row_index_dem(&path);
    let mut dem = DemRaster::open(&path).unwrap();

    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1_u64 << 53) as f64
    };
    for _ in 0..400 {
        let at = TOP_LEFT
            + DVec2::new(
                next() * f64::from(WIDTH) * PIXEL.x,
                -next() * f64::from(HEIGHT) * PIXEL.y,
            );
        let output = Command::new("gdallocationinfo")
            .args(["-valonly", "-geoloc"])
            .arg(&path)
            .arg(format!("{:.9}", at.x))
            .arg(format!("{:.9}", at.y))
            .output()
            .unwrap();
        let expected: f32 = String::from_utf8_lossy(&output.stdout)
            .trim()
            .parse()
            .unwrap();
        assert_eq!(dem.sample(at).unwrap(), Some(expected), "at {at}");
    }
}

fn mix(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// Float32 raster of per-pixel pseudo-random heights in quarter-metre steps, with 1 m
/// pixels whose top-left corner is at `TOP_LEFT`.
fn write_noise_dem(path: &Path, side: u32) {
    let data: Vec<f32> = (0..u64::from(side) * u64::from(side))
        .map(|cell| (mix(1 + cell) % 255) as f32 * 0.25)
        .collect();
    let file = BufWriter::new(File::create(path).unwrap());
    let mut encoder = TiffEncoder::new(file).unwrap();
    let mut image = encoder.new_image::<Gray32Float>(side, side).unwrap();
    let tags = image.encoder();
    tags.write_tag(Tag::ModelPixelScaleTag, &[1.0, 1.0, 0.0][..])
        .unwrap();
    tags.write_tag(
        Tag::ModelTiepointTag,
        &[0.0, 0.0, 0.0, TOP_LEFT.x, TOP_LEFT.y, 0.0][..],
    )
    .unwrap();
    let keys: [u16; 16] = [1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 32617];
    tags.write_tag(Tag::GeoKeyDirectoryTag, &keys[..]).unwrap();
    image.write_data(&data).unwrap();
}

/// The first error from sampling `dem` on a 32-pixel grid over a `side` x `side` raster.
fn first_sample_error(dem: &mut DemRaster, side: u32) -> Option<GeoError> {
    (0..side)
        .step_by(32)
        .flat_map(|row| (0..side).step_by(32).map(move |column| (column, row)))
        .find_map(|(column, row)| {
            let at = TOP_LEFT + DVec2::new(f64::from(column) + 0.5, -(f64::from(row) + 0.5));
            dem.sample(at).err()
        })
}

/// A GDAL LZW COG with 128-pixel tiles of this raster has tiles that the tiff crate
/// rejects with "no lzw end code found". The error must name the Deflate workaround, and
/// the same raster with Deflate must sample without error.
#[test]
fn undecodable_lzw_dem_tile_reports_the_deflate_workaround() {
    if !gdal::gdal_available("undecodable_lzw_dem_tile_reports_the_deflate_workaround") {
        return;
    }
    let side = 900;
    let dir = TempDir::new().unwrap();
    let source = dir.path().join("source.tif");
    write_noise_dem(&source, side);
    let cog = |name: &str, compression: &str| {
        let out = dir.path().join(name);
        let compression = PathBuf::from(format!("COMPRESS={compression}"));
        gdal::run_gdal(
            "gdal_translate",
            &[
                Path::new("-of"),
                Path::new("COG"),
                Path::new("-co"),
                &compression,
                Path::new("-co"),
                Path::new("BLOCKSIZE=128"),
                &source,
                &out,
            ],
        );
        out
    };

    let mut lzw = DemRaster::open(&cog("lzw.tif", "LZW")).unwrap();
    match first_sample_error(&mut lzw, side) {
        Some(GeoError::Core(VoxelError::Unsupported(message))) => {
            assert!(message.contains("LZW"), "{message}");
            assert!(message.contains("COMPRESS=DEFLATE"), "{message}");
        }
        other => panic!("expected an Unsupported error, got {other:?}"),
    }

    let mut deflate = DemRaster::open(&cog("deflate.tif", "DEFLATE")).unwrap();
    assert!(first_sample_error(&mut deflate, side).is_none());
}
