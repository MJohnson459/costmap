use std::path::{Path, PathBuf};

use glam::Vec3;
use image::GenericImageView;
use serde::Deserialize;

use crate::grid::OccupancyGrid;
use crate::types::{
    DEFAULT_FREE_THRESH, DEFAULT_OCCUPIED_THRESH, FREE, MapInfo, OCCUPIED, UNKNOWN, VoxelError,
};

#[derive(Debug, Deserialize)]
struct RosMapMetadata {
    image: String,
    resolution: f32,
    origin: [f32; 3],
    #[serde(
        default = "default_occupied_thresh",
        deserialize_with = "deserialize_threshold"
    )]
    occupied_thresh: f32,
    #[serde(
        default = "default_free_thresh",
        deserialize_with = "deserialize_threshold"
    )]
    free_thresh: f32,
    #[serde(default = "default_negate")]
    negate: Negate,
    #[serde(default = "default_map_mode")]
    mode: MapMode,
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

fn default_negate() -> Negate {
    Negate::Bool(false)
}

#[derive(Debug, Copy, Clone, Deserialize)]
#[serde(rename_all = "lowercase")]
enum MapMode {
    Trinary,
    Scale,
    Raw,
}

fn default_map_mode() -> MapMode {
    MapMode::Trinary
}

fn default_occupied_thresh() -> f32 {
    DEFAULT_OCCUPIED_THRESH
}

fn default_free_thresh() -> f32 {
    DEFAULT_FREE_THRESH
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

pub fn load_occupancy_grid(yaml_path: impl AsRef<Path>) -> Result<OccupancyGrid, VoxelError> {
    let yaml_path = yaml_path.as_ref();
    let yaml_str = std::fs::read_to_string(yaml_path)?;
    let metadata: RosMapMetadata = serde_yaml::from_str(&yaml_str)?;

    if matches!(metadata.mode, MapMode::Trinary | MapMode::Scale)
        && metadata.occupied_thresh <= metadata.free_thresh
    {
        return Err(VoxelError::InvalidMetadata(
            "occupied_thresh must be greater than free_thresh".to_string(),
        ));
    }

    let negate = metadata.negate.is_negated();
    let image_path = resolve_image_path(yaml_path, &metadata.image);
    let image = image::open(&image_path)?;
    let (width, height) = image.dimensions();
    let rgba = image.to_rgba8();

    let mut data = vec![UNKNOWN; (width as usize) * (height as usize)];

    for y in 0..height {
        for x in 0..width {
            let pixel = rgba.get_pixel(x, y);
            let [r, g, b, a] = pixel.0;
            let mut lightness = (r as f32 + g as f32 + b as f32) / (3.0 * 255.0);
            if negate {
                lightness = 1.0 - lightness;
            }
            let alpha = a as f32 / 255.0;

            let value = match metadata.mode {
                MapMode::Trinary => {
                    if lightness >= metadata.occupied_thresh {
                        OCCUPIED
                    } else if lightness <= metadata.free_thresh {
                        FREE
                    } else {
                        UNKNOWN
                    }
                }
                MapMode::Scale => {
                    if alpha < 1.0 {
                        UNKNOWN
                    } else if lightness >= metadata.occupied_thresh {
                        OCCUPIED
                    } else if lightness <= metadata.free_thresh {
                        FREE
                    } else {
                        let ratio = (lightness - metadata.free_thresh)
                            / (metadata.occupied_thresh - metadata.free_thresh);
                        let occupancy = (ratio * 100.0).round();
                        occupancy.clamp(0.0, 100.0) as i8
                    }
                }
                MapMode::Raw => {
                    let occupancy = (lightness * 100.0).round();
                    if occupancy <= 100.0 {
                        occupancy as i8
                    } else {
                        UNKNOWN
                    }
                }
            };

            let grid_y = height - y - 1;
            let idx = (grid_y as usize) * (width as usize) + (x as usize);
            data[idx] = value;
        }
    }

    let info = MapInfo {
        width,
        height,
        resolution: metadata.resolution,
        origin: Vec3::new(metadata.origin[0], metadata.origin[1], metadata.origin[2]),
    };

    OccupancyGrid::new(info, data)
}

fn resolve_image_path(yaml_path: &Path, image_ref: &str) -> PathBuf {
    let image_path = PathBuf::from(image_ref);
    if image_path.is_absolute() {
        return image_path;
    }

    match yaml_path.parent() {
        Some(parent) => parent.join(image_path),
        None => image_path,
    }
}
