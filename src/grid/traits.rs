use glam::UVec3;

use crate::types::{MapInfo, VoxelError};

pub trait Grid {
    type Cell: Copy;

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

    fn get(&self, pos: &UVec3) -> Option<Self::Cell>;
    fn set(&mut self, pos: &UVec3, value: Self::Cell) -> Result<(), VoxelError>;
}
