//! Geometric and spatial types used across the grid and costmap APIs.

use glam::{DVec2, IVec2, UVec2, Vec2};

/// Robot pose in world coordinates (meters).
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    pub position: DVec2,
    pub yaw: f32,
}

impl Pose2 {
    pub fn new(position: DVec2, yaw: f32) -> Self {
        Self { position, yaw }
    }
}

/// World-axis-aligned rectangle in meters.
/// Convention: [min.x, max.x) x [min.y, max.y) in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: DVec2,
    pub max: DVec2,
}

impl Bounds {
    /// Create bounds that represent "no region" (empty). Use this as the initial
    /// value before layers expand it; layers should only expand, never shrink.
    pub fn empty() -> Self {
        Self {
            min: DVec2::INFINITY,
            max: DVec2::NEG_INFINITY,
        }
    }

    /// Returns true if no layer has expanded the bounds (min > max in either axis).
    pub fn is_empty(&self) -> bool {
        self.min.x >= self.max.x || self.min.y >= self.max.y
    }

    /// Expand this bounds to include the point (in place).
    pub fn expand_to_include(&mut self, p: DVec2) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
    }

    /// Expand by a margin in meters in all directions (e.g. for inflation halo).
    pub fn expand_by(&mut self, margin: f32) {
        let margin = f64::from(margin);
        self.min.x -= margin;
        self.min.y -= margin;
        self.max.x += margin;
        self.max.y += margin;
    }
}

/// Update window in cell positions. Region is [min.x, max.x) x [min.y, max.y).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellRegion {
    pub min: IVec2,
    pub max: IVec2,
}

impl CellRegion {
    pub fn new(min: IVec2, max: IVec2) -> Self {
        Self { min, max }
    }

    /// `size` cells from `min`. A size beyond the `i32` cell range saturates.
    #[inline]
    pub fn sized(min: IVec2, size: UVec2) -> Self {
        Self::new(min, min.saturating_add_unsigned(size))
    }

    /// `size` cells from (0, 0).
    #[inline]
    pub fn from_size(size: UVec2) -> Self {
        Self::sized(IVec2::ZERO, size)
    }

    /// `size` cells whose middle cell is `center`: `size / 2` cells lie below it on
    /// each axis. Near the limits of the `i32` cell range the region moves inward so
    /// it keeps its size; a side longer than that range saturates.
    pub fn centered(center: IVec2, size: UVec2) -> Self {
        let axis = |center: i32, size: u32| {
            let (size, lowest) = (i64::from(size), i64::from(i32::MIN));
            let highest = (i64::from(i32::MAX) - size).max(lowest);
            (i64::from(center) - size / 2).clamp(lowest, highest) as i32
        };
        Self::sized(
            IVec2::new(axis(center.x, size.x), axis(center.y, size.y)),
            size,
        )
    }

    /// This region with `cells` more on every side, saturating at the `i32` limits.
    #[inline]
    pub fn grown(&self, cells: u32) -> Self {
        Self::new(
            self.min.saturating_sub_unsigned(UVec2::splat(cells)),
            self.max.saturating_add_unsigned(UVec2::splat(cells)),
        )
    }

    /// Whether every cell of `other` is in this region. An empty `other` is in any region.
    #[inline]
    pub fn contains_region(&self, other: &CellRegion) -> bool {
        other.size().cmpeq(UVec2::ZERO).any()
            || (self.min.cmple(other.min).all() && self.max.cmpge(other.max).all())
    }

    /// Whether the two regions share a cell.
    #[inline]
    pub fn overlaps(&self, other: &CellRegion) -> bool {
        self.min.max(other.min).cmplt(self.max.min(other.max)).all()
    }

    /// Width and height in cells; zero on an axis where `max` is not above `min`.
    #[inline]
    pub fn size(&self) -> UVec2 {
        let size = self.max.saturating_sub(self.min).max(IVec2::ZERO);
        size.as_uvec2()
    }

    /// Returns true if the cell position is inside this region (half-open bounds).
    #[inline]
    pub fn contains(&self, pos: IVec2) -> bool {
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
    pub fn transform(&self, pose: Pose2) -> Vec<DVec2> {
        let cos_yaw = pose.yaw.cos();
        let sin_yaw = pose.yaw.sin();
        self.points
            .iter()
            .map(|p| {
                let rotated =
                    Vec2::new(p.x * cos_yaw - p.y * sin_yaw, p.x * sin_yaw + p.y * cos_yaw);
                pose.position + rotated.as_dvec2()
            })
            .collect()
    }

    pub fn is_valid(&self) -> bool {
        self.points.len() >= 3
    }
}
