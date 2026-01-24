use glam::Vec3;

#[derive(Debug, Clone, PartialEq)]
pub struct MapInfo {
    pub width: u32,
    pub height: u32,
    pub depth: u32,
    pub resolution: f32,
    /// (x, y, z) in world coordinates.
    pub origin: Vec3,
}
