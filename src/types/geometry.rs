//! Geometric and spatial types used across the grid and costmap APIs.

use glam::{UVec2, Vec2};

/// Robot pose in world coordinates (meters).
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    pub position: Vec2,
    pub yaw: f32,
}

impl Pose2 {
    pub fn new(position: Vec2, yaw: f32) -> Self {
        Self { position, yaw }
    }
}

/// World-axis-aligned rectangle in meters.
/// Convention: [min.x, max.x) x [min.y, max.y) in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: Vec2,
    pub max: Vec2,
}

impl Bounds {
    /// Create bounds that represent "no region" (empty). Use this as the initial
    /// value before layers expand it; layers should only expand, never shrink.
    pub fn empty() -> Self {
        Self {
            min: Vec2::new(f32::INFINITY, f32::INFINITY),
            max: Vec2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    /// Returns true if no layer has expanded the bounds (min > max in either axis).
    pub fn is_empty(&self) -> bool {
        self.min.x >= self.max.x || self.min.y >= self.max.y
    }

    /// Expand this bounds to include the point (in place).
    pub fn expand_to_include(&mut self, p: Vec2) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
    }

    /// Expand by a margin in meters in all directions (e.g. for inflation halo).
    pub fn expand_by(&mut self, margin: f32) {
        self.min.x -= margin;
        self.min.y -= margin;
        self.max.x += margin;
        self.max.y += margin;
    }
}

/// Update window in cell indices. Region is [min.x, max.x) x [min.y, max.y).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellRegion {
    pub min: UVec2,
    pub max: UVec2,
}

impl CellRegion {
    pub fn new(min: UVec2, max: UVec2) -> Self {
        Self { min, max }
    }

    /// Returns true if the cell position is inside this region (half-open bounds).
    #[inline]
    pub fn contains(&self, pos: UVec2) -> bool {
        pos.x >= self.min.x && pos.x < self.max.x && pos.y >= self.min.y && pos.y < self.max.y
    }
}

/// Footprint: polygon in world coordinates (meters). Used by layers for footprint callbacks.
#[derive(Debug, Clone, Default)]
pub struct Footprint {
    pub points: Vec<Vec2>,
}

impl Footprint {
    /// Rectangular footprint centered at the origin (robot frame).
    /// `length` is along x, `width` along y; vertices in counterclockwise order.
    pub fn rectangle(length: f32, width: f32) -> Self {
        let half_length = length / 2.0;
        let half_width = width / 2.0;
        Self {
            points: vec![
                Vec2::new(half_length, half_width),
                Vec2::new(half_length, -half_width),
                Vec2::new(-half_length, -half_width),
                Vec2::new(-half_length, half_width),
            ],
        }
    }

    /// Transform footprint points from robot-local frame to world frame.
    pub fn transform(&self, pose: Pose2) -> Vec<Vec2> {
        let cos_yaw = pose.yaw.cos();
        let sin_yaw = pose.yaw.sin();
        self.points
            .iter()
            .map(|p| {
                let rotated =
                    Vec2::new(p.x * cos_yaw - p.y * sin_yaw, p.x * sin_yaw + p.y * cos_yaw);
                pose.position + rotated
            })
            .collect()
    }

    pub fn is_valid(&self) -> bool {
        self.points.len() >= 3
    }
}
