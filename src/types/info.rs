use glam::Vec3;

#[derive(Debug, Clone)]
pub struct MapInfo {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    /// (x, y, yaw) in map coordinates.
    pub origin: Vec3,
}
