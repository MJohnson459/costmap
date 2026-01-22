use glam::IVec2;

pub mod dda;
pub mod grid_step;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RayHit2D {
    /// Grid cell that contains the first hit.
    pub cell: IVec2,
    /// Distance from the ray origin to the cell boundary hit (world units).
    pub hit_distance: f32,
}
