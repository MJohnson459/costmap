use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum VoxelError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("yaml error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("image error: {0}")]
    Image(#[from] image::ImageError),
    #[cfg(feature = "geotiff")]
    #[error("tiff error: {0}")]
    Tiff(#[from] tiff::TiffError),
    #[error("out of bounds: {0}")]
    OutOfBounds(String),
    #[error("invalid metadata: {0}")]
    InvalidMetadata(String),
    #[error("invalid argument: {0}")]
    InvalidArgument(String),
    #[error("unsupported: {0}")]
    Unsupported(String),
}
