use thiserror::Error;

#[derive(Debug, Error)]
pub enum VoxelError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("yaml error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("image error: {0}")]
    Image(#[from] image::ImageError),
    #[error("invalid metadata: {0}")]
    InvalidMetadata(String),
}
