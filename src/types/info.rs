//! Map metadata.

use glam::Vec2;

#[derive(Debug, Clone, PartialEq)]
pub struct MapInfo {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    /// Origin of cell (0, 0) in world coordinates (meters).
    pub origin: Vec2,
}

impl Default for MapInfo {
    fn default() -> Self {
        Self {
            width: 100,
            height: 100,
            resolution: 0.05,
            origin: Vec2::ZERO,
        }
    }
}

impl MapInfo {
    pub fn square(width: u32, resolution: f32) -> Self {
        Self {
            width,
            height: width,
            resolution,
            ..Default::default()
        }
    }

    /// Width of the map in world units (meters).
    #[inline]
    pub fn world_width(&self) -> f32 {
        self.width as f32 * self.resolution
    }

    /// Height of the map in world units (meters).
    #[inline]
    pub fn world_height(&self) -> f32 {
        self.height as f32 * self.resolution
    }

    /// Center of the map in 2D world coordinates.
    #[inline]
    pub fn world_center(&self) -> Vec2 {
        self.origin + Vec2::new(0.5 * self.world_width(), 0.5 * self.world_height())
    }
}
