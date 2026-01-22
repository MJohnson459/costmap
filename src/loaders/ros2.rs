use std::path::{Path, PathBuf};

use glam::Vec3;
use image::GenericImageView;
use serde::Deserialize;

use crate::grid::OccupancyGrid;
use crate::types::{
    DEFAULT_FREE_THRESH, DEFAULT_OCCUPIED_THRESH, FREE, MapInfo, OCCUPIED, UNKNOWN, VoxelError,
};

#[derive(Debug, Deserialize)]
#[serde(default)]
struct RosMapMetadata {
    image: PathBuf,
    resolution: f32,
    origin: [f32; 3],
    #[serde(deserialize_with = "deserialize_threshold")]
    occupied_thresh: f32,
    #[serde(deserialize_with = "deserialize_threshold")]
    free_thresh: f32,
    negate: Negate,
    mode: MapMode,
}

impl Default for RosMapMetadata {
    fn default() -> Self {
        Self {
            image: PathBuf::new(),
            resolution: 0.05,
            origin: [0.0, 0.0, 0.0],
            occupied_thresh: DEFAULT_OCCUPIED_THRESH,
            free_thresh: DEFAULT_FREE_THRESH,
            negate: Negate::Bool(false),
            mode: MapMode::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum Negate {
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
enum MapMode {
    #[default]
    Trinary,
    Scale,
    Raw,
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

/// Loads an occupancy grid from a YAML or image file.
///
/// If loading an image file, the values of the metadata assume the ROS2 default
/// values with the map origin at [0,0].
pub fn load_occupancy_grid(path: impl AsRef<Path>) -> Result<OccupancyGrid, VoxelError> {
    let path = path.as_ref();
    let metadata = if is_yaml(path) {
        let yaml_str = std::fs::read_to_string(path)?;
        let mut metadata: RosMapMetadata = serde_yaml::from_str(&yaml_str)?;
        metadata.image = resolve_image_path(path, &metadata.image);
        metadata
    } else {
        let image_path = PathBuf::from(path);
        RosMapMetadata {
            image: image_path.clone(),
            ..Default::default()
        }
    };

    if matches!(metadata.mode, MapMode::Trinary | MapMode::Scale)
        && metadata.occupied_thresh <= metadata.free_thresh
    {
        return Err(VoxelError::InvalidMetadata(
            "occupied_thresh must be greater than free_thresh".to_string(),
        ));
    }

    let negate = metadata.negate.is_negated();
    let image = image::open(&metadata.image)?;
    let (data, width, height) = map_image_to_data(
        image,
        metadata.mode,
        metadata.occupied_thresh,
        metadata.free_thresh,
        negate,
    )?;

    let info = MapInfo {
        width,
        height,
        resolution: metadata.resolution,
        origin: Vec3::new(metadata.origin[0], metadata.origin[1], metadata.origin[2]),
    };

    OccupancyGrid::new(info, data)
}

fn is_yaml(path: &Path) -> bool {
    match path.extension().and_then(|ext| ext.to_str()) {
        Some("yaml") | Some("yml") => true,
        _ => false,
    }
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
    mode: MapMode,
    occupied_thresh: f32,
    free_thresh: f32,
    negate: bool,
) -> Result<(Vec<i8>, u32, u32), VoxelError> {
    if matches!(mode, MapMode::Trinary | MapMode::Scale) && occupied_thresh <= free_thresh {
        return Err(VoxelError::InvalidMetadata(
            "occupied_thresh must be greater than free_thresh".to_string(),
        ));
    }

    let (width, height) = image.dimensions();
    let has_alpha = image.color().has_alpha();
    let mut data = vec![UNKNOWN; (width as usize) * (height as usize)];
    let width_usize = width as usize;
    let height_usize = height as usize;

    match mode {
        MapMode::Scale => {
            let rgba = image.into_rgba8();
            let raw = rgba.as_raw();
            fill_from_rgba(&mut data, raw, width_usize, height_usize, |r, g, b, a| {
                let shade = shade_rgb(r, g, b);
                let occ = if negate { shade } else { 1.0 - shade };
                value_from_occ(mode, occ, a, occupied_thresh, free_thresh)
            });
        }
        MapMode::Trinary | MapMode::Raw => {
            if has_alpha && matches!(mode, MapMode::Trinary) {
                // NOTE: ROS2 averages inverted alpha in Trinary mode when alpha is present.
                // We intentionally ignore alpha here for speed and log a warning instead.
                eprintln!("ros2 loader: alpha channel present; Trinary mode ignores alpha");
            }
            let rgb = image.into_rgb8();
            let raw = rgb.as_raw();
            fill_from_rgb(&mut data, raw, width_usize, height_usize, |r, g, b| {
                let shade = shade_rgb(r, g, b);
                let occ = if negate { shade } else { 1.0 - shade };
                value_from_occ(mode, occ, 255, occupied_thresh, free_thresh)
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
        for x in 0..width {
            let base = src_row + x * 4;
            data[grid_row + x] = f(raw[base], raw[base + 1], raw[base + 2], raw[base + 3]);
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
        for x in 0..width {
            let base = src_row + x * 3;
            data[grid_row + x] = f(raw[base], raw[base + 1], raw[base + 2]);
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
        assert_eq!(metadata.origin, [0.0, 0.0, 0.0]);
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
        assert_eq!(metadata.origin, [0.0, 0.0, 0.0]);
        assert_eq!(metadata.occupied_thresh, 0.65);
        assert_eq!(metadata.free_thresh, 0.196);
        assert_eq!(metadata.negate.is_negated(), false);
        assert!(matches!(metadata.mode, MapMode::Trinary));
    }
}
