//! Map metadata.

use glam::DVec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapInfo {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    /// Origin of cell (0, 0) in world coordinates (meters).
    pub origin: DVec2,
}

impl Default for MapInfo {
    fn default() -> Self {
        Self {
            width: 100,
            height: 100,
            resolution: 0.05,
            origin: DVec2::ZERO,
        }
    }
}

impl MapInfo {
    pub fn square(width: u32, resolution: f64) -> Self {
        Self {
            width,
            height: width,
            resolution,
            ..Default::default()
        }
    }

    /// Width of the map in world units (meters).
    #[inline]
    pub fn world_width(&self) -> f64 {
        f64::from(self.width) * self.resolution
    }

    /// Height of the map in world units (meters).
    #[inline]
    pub fn world_height(&self) -> f64 {
        f64::from(self.height) * self.resolution
    }

    /// Center of the map in 2D world coordinates.
    #[inline]
    pub fn world_center(&self) -> DVec2 {
        self.origin + DVec2::new(0.5 * self.world_width(), 0.5 * self.world_height())
    }
}
