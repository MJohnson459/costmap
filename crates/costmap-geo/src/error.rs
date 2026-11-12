use std::path::PathBuf;

use costmap::VoxelError;

/// Failure while reading rasters, parsing overrides or writing the cost map.
#[derive(Debug, thiserror::Error)]
pub enum GeoError {
    /// A rule parameter is out of range.
    #[error("invalid rule parameter: {0}")]
    Rules(String),
    /// An input raster cannot be used; `source` says why.
    #[error("{}: {source}", path.display())]
    Input {
        path: PathBuf,
        source: Box<GeoError>,
    },
    /// A raster has an unsupported layout or inconsistent contents.
    #[error("{0}")]
    Raster(String),
    /// The inputs are not in the same coordinate reference system.
    #[error("{0}")]
    Crs(String),
    /// The overrides GeoJSON is malformed.
    #[error("{0}")]
    Overrides(String),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("tiff error: {0}")]
    Tiff(#[from] tiff::TiffError),
    #[error(transparent)]
    Core(#[from] VoxelError),
}

impl GeoError {
    /// Attach the file the error came from.
    pub fn at(self, path: impl Into<PathBuf>) -> Self {
        Self::Input {
            path: path.into(),
            source: Box::new(self),
        }
    }
}
