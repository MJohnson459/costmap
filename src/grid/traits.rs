use glam::UVec3;

use crate::types::{MapInfo, VoxelError};

/// Shared grid interface for 2D and 3D backends.
pub trait Grid {
    type Cell: Clone;

    fn info(&self) -> &MapInfo;
    fn width(&self) -> u32 {
        self.info().width
    }
    fn height(&self) -> u32 {
        self.info().height
    }
    fn depth(&self) -> u32 {
        self.info().depth
    }

    /// Get the cell with bounds checking.
    fn get(&self, pos: &UVec3) -> Option<Self::Cell>;
    /// Set the cell with bounds checking.
    fn set(&mut self, pos: &UVec3, value: Self::Cell) -> Result<(), VoxelError>;
}
