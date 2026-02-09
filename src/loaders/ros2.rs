use std::path::{Path, PathBuf};

use glam::Vec3;
use image::GenericImageView;
use serde::Deserialize;

use crate::OccupancyGrid;
use crate::types::{
    DEFAULT_FREE_THRESH, DEFAULT_OCCUPIED_THRESH, FREE, MapInfo, OCCUPIED, UNKNOWN, VoxelError,
};

pub struct RosMapLoader;

impl RosMapLoader {
    pub fn load_from_yaml(path: impl AsRef<Path>) -> Result<OccupancyGrid, VoxelError> {
        load_occupancy_grid_from_yaml(path)
    }

    pub fn load_from_image(metadata: &RosMapMetadata) -> Result<OccupancyGrid, VoxelError> {
        load_occupancy_grid_from_image(metadata)
    }

    pub fn load_from_memory(
        image: image::DynamicImage,
        metadata: &RosMapMetadata,
    ) -> Result<OccupancyGrid, VoxelError> {
        load_occupancy_grid_from_memory(image, metadata)
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct RosMapMetadata {
    pub image: PathBuf,
    pub resolution: f32,
    #[serde(deserialize_with = "deserialize_origin")]
    pub origin: Vec3,
    #[serde(deserialize_with = "deserialize_threshold")]
    pub occupied_thresh: f32,
    #[serde(deserialize_with = "deserialize_threshold")]
    pub free_thresh: f32,
    pub negate: Negate,
    pub mode: MapMode,
}

impl Default for RosMapMetadata {
    fn default() -> Self {
        Self {
            image: PathBuf::new(),
            resolution: 0.05,
            origin: Vec3::new(0.0, 0.0, 0.0),
            occupied_thresh: DEFAULT_OCCUPIED_THRESH,
            free_thresh: DEFAULT_FREE_THRESH,
            negate: Negate::Bool(false),
            mode: MapMode::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
pub enum Negate {
    Bool(bool),
    Int(i32),
}

impl Negate {
    fn is_negated(&self) -> bool {
        match self {
            Self::Bool(value) => *value,
            Self::Int(value) => *value != 0,
        }
    }
}

#[derive(Debug, Copy, Clone, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum MapMode {
    #[default]
    Trinary,
    Scale,
    Raw,
}

fn deserialize_origin<'de, D>(deserializer: D) -> Result<Vec3, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let [x, y, z]: [f32; 3] = Deserialize::deserialize(deserializer)?;
    let value = Vec3::new(x, y, z);
    Ok(value)
}

fn deserialize_threshold<'de, D>(deserializer: D) -> Result<f32, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = f32::deserialize(deserializer)?;
    if (0.0..=1.0).contains(&value) {
        Ok(value)
    } else {
        Err(serde::de::Error::custom(
            "thresholds must be in the range [0.0, 1.0]",
        ))
    }
}

/// Loads an occupancy grid from a YAML.
fn load_occupancy_grid_from_yaml(path: impl AsRef<Path>) -> Result<OccupancyGrid, VoxelError> {
    let path = path.as_ref();
    let yaml_str = std::fs::read_to_string(path)?;
    let mut metadata: RosMapMetadata = serde_yaml::from_str(&yaml_str)?;
    metadata.image = resolve_image_path(path, &metadata.image);

    load_occupancy_grid_from_image(&metadata)
}

/// Loads an occupancy grid from an image file.
fn load_occupancy_grid_from_image(metadata: &RosMapMetadata) -> Result<OccupancyGrid, VoxelError> {
    if matches!(metadata.mode, MapMode::Trinary | MapMode::Scale)
        && metadata.occupied_thresh <= metadata.free_thresh
    {
        return Err(VoxelError::InvalidMetadata(
            "occupied_thresh must be greater than free_thresh".to_string(),
        ));
    }

    let image = image::open(&metadata.image)?;
    load_occupancy_grid_from_memory(image, metadata)
}

fn load_occupancy_grid_from_memory(
    image: image::DynamicImage,
    metadata: &RosMapMetadata,
) -> Result<OccupancyGrid, VoxelError> {
    let (data, width, height) = map_image_to_data(image, &metadata)?;
    let info = MapInfo {
        width,
        height,
        resolution: metadata.resolution,
        origin: metadata.origin.truncate(),
    };
    OccupancyGrid::new(info, data)
}

fn resolve_image_path(yaml_path: &Path, image_ref: &Path) -> PathBuf {
    if image_ref.is_absolute() {
        return image_ref.to_path_buf();
    }

    match yaml_path.parent() {
        Some(parent) => parent.join(image_ref),
        None => image_ref.to_path_buf(),
    }
}

fn map_image_to_data(
    image: image::DynamicImage,
    metadata: &RosMapMetadata,
) -> Result<(Vec<i8>, u32, u32), VoxelError> {
    if matches!(metadata.mode, MapMode::Trinary | MapMode::Scale)
        && metadata.occupied_thresh <= metadata.free_thresh
    {
        return Err(VoxelError::InvalidMetadata(
            "occupied_thresh must be greater than free_thresh".to_string(),
        ));
    }

    let (width, height) = image.dimensions();
    let has_alpha = image.color().has_alpha();
    let mut data = vec![UNKNOWN; (width as usize) * (height as usize)];
    let width_usize = width as usize;
    let height_usize = height as usize;

    match metadata.mode {
        MapMode::Scale => {
            let rgba = image.into_rgba8();
            let raw = rgba.into_raw();
            fill_from_rgba(&mut data, &raw, width_usize, height_usize, |r, g, b, a| {
                let shade = shade_rgb(r, g, b);
                let occ = if metadata.negate.is_negated() {
                    shade
                } else {
                    1.0 - shade
                };
                value_from_occ(
                    metadata.mode,
                    occ,
                    a,
                    metadata.occupied_thresh,
                    metadata.free_thresh,
                )
            });
        }
        MapMode::Trinary | MapMode::Raw => {
            if has_alpha && matches!(metadata.mode, MapMode::Trinary) {
                // NOTE: ROS2 averages inverted alpha in Trinary mode when alpha is present.
                // We intentionally ignore alpha here for speed and log a warning instead.
                eprintln!("ros2 loader: alpha channel present; Trinary mode ignores alpha");
            }
            let rgb = image.into_rgb8();
            let raw = rgb.into_raw();
            fill_from_rgb(&mut data, &raw, width_usize, height_usize, |r, g, b| {
                let shade = shade_rgb(r, g, b);
                let occ = if metadata.negate.is_negated() {
                    shade
                } else {
                    1.0 - shade
                };
                value_from_occ(
                    metadata.mode,
                    occ,
                    255,
                    metadata.occupied_thresh,
                    metadata.free_thresh,
                )
            });
        }
    }

    Ok((data, width, height))
}

fn shade_rgb(r: u8, g: u8, b: u8) -> f32 {
    (r as f32 + g as f32 + b as f32) / (3.0 * 255.0)
}

fn value_from_occ(
    mode: MapMode,
    occ: f32,
    alpha: u8,
    occupied_thresh: f32,
    free_thresh: f32,
) -> i8 {
    match mode {
        MapMode::Trinary => trinary_value(occ, occupied_thresh, free_thresh),
        MapMode::Scale => scale_value(occ, alpha, occupied_thresh, free_thresh),
        MapMode::Raw => raw_value(occ),
    }
}

fn trinary_value(occ: f32, occupied_thresh: f32, free_thresh: f32) -> i8 {
    if occ >= occupied_thresh {
        OCCUPIED
    } else if occ <= free_thresh {
        FREE
    } else {
        UNKNOWN
    }
}

fn scale_value(occ: f32, alpha: u8, occupied_thresh: f32, free_thresh: f32) -> i8 {
    if alpha < 255 {
        UNKNOWN
    } else if occ >= occupied_thresh {
        OCCUPIED
    } else if occ <= free_thresh {
        FREE
    } else {
        let ratio = (occ - free_thresh) / (occupied_thresh - free_thresh);
        let occupancy = (ratio * 100.0).round();
        occupancy.clamp(0.0, 100.0) as i8
    }
}

fn raw_value(shade: f32) -> i8 {
    let occupancy = (shade * 255.0).round();
    if (0.0..=100.0).contains(&occupancy) {
        occupancy as i8
    } else {
        UNKNOWN
    }
}

fn fill_from_rgba<F>(data: &mut [i8], raw: &[u8], width: usize, height: usize, mut f: F)
where
    F: FnMut(u8, u8, u8, u8) -> i8,
{
    for y in 0..height {
        let grid_row = (height - 1 - y) * width;
        let src_row = y * width * 4;
        let dest = &mut data[grid_row..grid_row + width];
        let src = &raw[src_row..src_row + width * 4];
        for (dest_cell, rgba) in dest.iter_mut().zip(src.chunks_exact(4)) {
            *dest_cell = f(rgba[0], rgba[1], rgba[2], rgba[3]);
        }
    }
}

fn fill_from_rgb<F>(data: &mut [i8], raw: &[u8], width: usize, height: usize, mut f: F)
where
    F: FnMut(u8, u8, u8) -> i8,
{
    for y in 0..height {
        let grid_row = (height - 1 - y) * width;
        let src_row = y * width * 3;
        let dest = &mut data[grid_row..grid_row + width];
        let src = &raw[src_row..src_row + width * 3];
        for (dest_cell, rgb) in dest.iter_mut().zip(src.chunks_exact(3)) {
            *dest_cell = f(rgb[0], rgb[1], rgb[2]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_yaml_all_values() {
        let test_case = r#"
        image: simple.pgm
        resolution: 1.0
        origin: [0.0, 0.0, 0.0]
        occupied_thresh: 0.65
        free_thresh: 0.196
        negate: 0
        mode: trinary
        "#;

        let metadata: RosMapMetadata =
            serde_yaml::from_str(test_case).expect("metadata should parse");

        assert_eq!(metadata.image, PathBuf::from("simple.pgm"));
        assert_eq!(metadata.resolution, 1.0);
        assert_eq!(metadata.origin, Vec3::ZERO);
        assert_eq!(metadata.occupied_thresh, 0.65);
        assert_eq!(metadata.free_thresh, 0.196);
        assert_eq!(metadata.negate.is_negated(), false);
        assert!(matches!(metadata.mode, MapMode::Trinary));
    }

    #[test]
    fn check_yaml_minimal_values() {
        let test_case = r#"
        image: simple.pgm
        resolution: 0.05
        origin: [0.0, 0.0, 0.0]
        "#;

        let metadata: RosMapMetadata =
            serde_yaml::from_str(test_case).expect("metadata should parse");

        assert_eq!(metadata.image, PathBuf::from("simple.pgm"));
        assert_eq!(metadata.resolution, 0.05);
        assert_eq!(metadata.origin, Vec3::ZERO);
        assert_eq!(metadata.occupied_thresh, 0.65);
        assert_eq!(metadata.free_thresh, 0.196);
        assert_eq!(metadata.negate.is_negated(), false);
        assert!(matches!(metadata.mode, MapMode::Trinary));
    }
}
