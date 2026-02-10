use glam::UVec2;

pub mod dda;
pub mod grid_step;
mod utils;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RayHit2D {
    /// Grid cell that contains the first hit.
    pub cell: UVec2,
    /// Distance from the ray origin to the cell boundary hit (world units).
    pub hit_distance: f32,
}

impl RayHit2D {
    /// Extract hit distance, or return `default` if miss.
    pub fn distance_or(hit: Option<Self>, default: f32) -> f32 {
        hit.map(|h| h.hit_distance).unwrap_or(default)
    }
}
