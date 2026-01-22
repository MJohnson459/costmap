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
    occupied_thresh: Option<f32>,
    free_thresh: Option<f32>,
    negate: Option<Negate>,
    mode: Option<String>,
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

#[derive(Debug, Copy, Clone)]
enum MapMode {
    Trinary,
    Scale,
    Raw,
}

impl MapMode {
    fn from_str(value: &str) -> Option<Self> {
        match value.to_lowercase().as_str() {
            "trinary" => Some(Self::Trinary),
            "scale" => Some(Self::Scale),
            "raw" => Some(Self::Raw),
            _ => None,
        }
    }
}

pub fn load_occupancy_grid(yaml_path: impl AsRef<Path>) -> Result<OccupancyGrid, VoxelError> {
    let yaml_path = yaml_path.as_ref();
    let yaml_str = std::fs::read_to_string(yaml_path)?;
    let metadata: RosMapMetadata = serde_yaml::from_str(&yaml_str)?;

    let mode_value = metadata.mode.as_deref().unwrap_or("trinary");
    let mode = MapMode::from_str(mode_value).ok_or_else(|| {
        VoxelError::InvalidMetadata(format!("unsupported map mode '{mode_value}'"))
    })?;

    let occupied_thresh = metadata
        .occupied_thresh
        .unwrap_or(DEFAULT_OCCUPIED_THRESH);
    let free_thresh = metadata.free_thresh.unwrap_or(DEFAULT_FREE_THRESH);

    if !(0.0..=1.0).contains(&occupied_thresh) || !(0.0..=1.0).contains(&free_thresh) {
        return Err(VoxelError::InvalidMetadata(
            "thresholds must be in the range [0.0, 1.0]".to_string(),
        ));
    }

    if matches!(mode, MapMode::Trinary | MapMode::Scale) && occupied_thresh <= free_thresh {
        return Err(VoxelError::InvalidMetadata(
            "occupied_thresh must be greater than free_thresh".to_string(),
        ));
    }

    let negate = metadata.negate.map_or(false, |value| value.is_negated());
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

            let value = match mode {
                MapMode::Trinary => {
                    if lightness >= occupied_thresh {
                        OCCUPIED
                    } else if lightness <= free_thresh {
                        FREE
                    } else {
                        UNKNOWN
                    }
                }
                MapMode::Scale => {
                    if alpha < 1.0 {
                        UNKNOWN
                    } else if lightness >= occupied_thresh {
                        OCCUPIED
                    } else if lightness <= free_thresh {
                        FREE
                    } else {
                        let ratio = (lightness - free_thresh) / (occupied_thresh - free_thresh);
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
