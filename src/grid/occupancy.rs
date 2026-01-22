use crate::types::{MapInfo, VoxelError};

#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    info: MapInfo,
    data: Vec<i8>,
}

impl OccupancyGrid {
    pub fn new(info: MapInfo, data: Vec<i8>) -> Result<Self, VoxelError> {
        let expected_len = (info.width as usize) * (info.height as usize);
        if data.len() != expected_len {
            return Err(VoxelError::InvalidMetadata(format!(
                "data length {} does not match map size {}",
                data.len(),
                expected_len
            )));
        }

        Ok(Self { info, data })
    }

    pub fn info(&self) -> &MapInfo {
        &self.info
    }

    pub fn width(&self) -> u32 {
        self.info.width
    }

    pub fn height(&self) -> u32 {
        self.info.height
    }

    pub fn get(&self, x: u32, y: u32) -> Option<i8> {
        if x >= self.info.width || y >= self.info.height {
            return None;
        }
        let idx = self.index(x, y);
        Some(self.data[idx])
    }

    fn index(&self, x: u32, y: u32) -> usize {
        (y as usize) * (self.info.width as usize) + (x as usize)
    }
}
