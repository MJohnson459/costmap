use glam::{Vec2, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub struct MapInfo {
    pub width: u32,
    pub height: u32,
    pub depth: u32,
    pub resolution: f32,
    /// (x, y, z) in world coordinates.
    pub origin: Vec3,
}

impl MapInfo {
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

    /// The 2D (x, y) origin in world coordinates, discarding z.
    #[inline]
    pub fn origin_xy(&self) -> Vec2 {
        self.origin.truncate()
    }

    /// Center of the map in 2D world coordinates.
    #[inline]
    pub fn world_center(&self) -> Vec2 {
        self.origin_xy() + Vec2::new(0.5 * self.world_width(), 0.5 * self.world_height())
    }
}
