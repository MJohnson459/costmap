//! Obstacle inflation for `Grid2d<u8>` costmaps.
//!
//! Provides a generic inflate operation that expands lethal cells outward with
//! a distance-based cost falloff. The default cost curve is a simple linear
//! decay; Nav2-compatible exponential decay can be layered on top via
//! [`inscribed_inflation_cost`].
//!
//! [`InflationLayer`] implements the layered costmap [`Layer`](crate::grid::Layer)
//! trait so it can be used inside [`LayeredGrid2d`](crate::grid::LayeredGrid2d).

use glam::UVec2;

use crate::Grid2d;
use crate::grid::{Bounds, CellRegion, Layer, Pose2};
use crate::types::{COST_FREE, COST_LETHAL, MapInfo};

/// Convert an inflation radius in world units (meters) to a cell count.
///
/// Returns `ceil(radius / resolution)`. If resolution is zero or negative, or
/// if the result would be non-positive, returns 0.
#[inline]
pub fn inflation_radius_to_cells(radius_m: f32, resolution: f32) -> u32 {
    if resolution <= 0.0 || radius_m <= 0.0 {
        return 0;
    }
    (radius_m / resolution).ceil() as u32
}

/// Compute an inflation cost using a linear falloff.
///
/// Returns [`COST_LETHAL`] at distance 0, linearly decaying to 0 at
/// `radius_cells`. Values outside `[0, radius_cells]` are clamped.
pub fn linear_inflation_cost(distance_cells: f32, radius_cells: f32) -> u8 {
    if distance_cells <= 0.0 {
        return COST_LETHAL;
    }
    let t = (1.0 - distance_cells / radius_cells).clamp(0.0, 1.0);
    (COST_LETHAL as f32 * t).round() as u8
}

/// Compute an inflation cost using exponential decay with an inscribed radius.
///
/// The cost curve is:
///   - Distance ≤ 0        → `COST_LETHAL` (254)
///   - Distance ≤ inscribed → `COST_INSCRIBED` (253)
///   - Otherwise            → `exp(-cost_scaling_factor * (distance - inscribed_radius)) * (COST_INSCRIBED - 1)`
///
/// All distances are in cell units.
pub fn inscribed_inflation_cost(
    distance_cells: f32,
    inscribed_radius_cells: f32,
    cost_scaling_factor: f32,
) -> u8 {
    if distance_cells <= 0.0 {
        return COST_LETHAL;
    }
    if distance_cells <= inscribed_radius_cells {
        return crate::types::COST_INSCRIBED;
    }
    let value = ((-cost_scaling_factor * (distance_cells - inscribed_radius_cells)).exp()
        * (crate::types::COST_INSCRIBED as f32 - 1.0))
        .round() as u8;
    value.max(COST_FREE)
}

/// Inflate lethal obstacles from `source` into `dest` using the given radius
/// and cost function.
///
/// Non-lethal cells (including unknown) are copied from `source` to `dest` so
/// the inflated map preserves the free/unknown distinction. Lethal cells and
/// their inflation halos are then stamped on top, taking the maximum cost at
/// each cell.
///
/// Both grids must have the same resolution. The grids may have different sizes
/// and origins (the function operates in cell-local coordinates).
pub fn inflate(
    source: &Grid2d<u8>,
    dest: &mut Grid2d<u8>,
    radius_m: f32,
    cost_fn: fn(f32, f32) -> u8,
) {
    // Copy source into dest so unknown/free cells are preserved.
    for (cell, cost) in source.iter_cells() {
        let _ = dest.set(cell, *cost);
    }

    let resolution = source.info().resolution;
    let radius_cells = inflation_radius_to_cells(radius_m, resolution);
    if radius_cells == 0 {
        return;
    }

    let radius_i = radius_cells as i32;
    let radius_f = radius_cells as f32;

    for (cell, cost) in source.iter_cells() {
        if *cost != COST_LETHAL {
            continue;
        }

        let cx = cell.x as i32;
        let cy = cell.y as i32;

        for dy in -radius_i..=radius_i {
            for dx in -radius_i..=radius_i {
                let dist = ((dx * dx + dy * dy) as f32).sqrt();
                if dist > radius_f {
                    continue;
                }

                let nx = cx + dx;
                let ny = cy + dy;
                if nx < 0 || ny < 0 {
                    continue;
                }

                let pos = UVec2::new(nx as u32, ny as u32);
                if pos.x >= dest.width() || pos.y >= dest.height() {
                    continue;
                }

                let inflated_cost = cost_fn(dist, radius_f);
                let current = dest.get(pos).copied().unwrap_or(COST_FREE);
                if inflated_cost > current {
                    let _ = dest.set(pos, inflated_cost);
                }
            }
        }
    }
}

/// Inflate using inscribed exponential cost (Nav2-style). Same as [`inflate`] but
/// uses [`inscribed_inflation_cost`] with the given params (cell units).
pub fn inflate_inscribed(
    source: &Grid2d<u8>,
    dest: &mut Grid2d<u8>,
    radius_m: f32,
    inscribed_radius_cells: f32,
    cost_scaling_factor: f32,
) {
    for (cell, cost) in source.iter_cells() {
        let _ = dest.set(cell, *cost);
    }

    let resolution = source.info().resolution;
    let radius_cells = inflation_radius_to_cells(radius_m, resolution);
    if radius_cells == 0 {
        return;
    }

    let radius_i = radius_cells as i32;
    let radius_f = radius_cells as f32;

    for (cell, cost) in source.iter_cells() {
        if *cost != COST_LETHAL {
            continue;
        }

        let cx = cell.x as i32;
        let cy = cell.y as i32;

        for dy in -radius_i..=radius_i {
            for dx in -radius_i..=radius_i {
                let dist = ((dx * dx + dy * dy) as f32).sqrt();
                if dist > radius_f {
                    continue;
                }

                let nx = cx + dx;
                let ny = cy + dy;
                if nx < 0 || ny < 0 {
                    continue;
                }

                let pos = UVec2::new(nx as u32, ny as u32);
                if pos.x >= dest.width() || pos.y >= dest.height() {
                    continue;
                }

                let inflated_cost =
                    inscribed_inflation_cost(dist, inscribed_radius_cells, cost_scaling_factor);
                let current = dest.get(pos).copied().unwrap_or(COST_FREE);
                if inflated_cost > current {
                    let _ = dest.set(pos, inflated_cost);
                }
            }
        }
    }
}

/// Cost function type for inflation: (distance_cells, radius_cells) -> cost.
pub type InflationCostFn = fn(f32, f32) -> u8;

impl Grid2d<u8> {
    /// Inflate lethal obstacles with linear falloff, returning a new grid.
    pub fn inflated(&self, radius_m: f32) -> Grid2d<u8> {
        let mut dest = Grid2d::<u8>::empty(*self.info());
        inflate(self, &mut dest, radius_m, linear_inflation_cost);
        dest
    }
}

/// Cost curve for the inflation layer.
#[derive(Debug, Clone)]
pub enum InflationCostCurve {
    /// Linear falloff from lethal to zero over the radius.
    Linear,
    /// Nav2-style inscribed + exponential decay. Params in cell units.
    Inscribed {
        inscribed_radius_cells: f32,
        cost_scaling_factor: f32,
    },
    /// User-provided function pointer.
    Custom(InflationCostFn),
}

/// Layer that inflates lethal obstacles within the update region.
/// Caches temporary grids to avoid per-frame allocation.
#[derive(Debug)]
pub struct InflationLayer {
    inflation_radius_m: f32,
    curve: InflationCostCurve,
    /// Cached temp grids for the current region size; invalidated by match_size.
    temp_src: Option<Grid2d<u8>>,
    temp_dest: Option<Grid2d<u8>>,
}

impl InflationLayer {
    /// Create an inflation layer with the given radius (meters) and cost curve.
    pub fn new(inflation_radius_m: f32, curve: InflationCostCurve) -> Self {
        Self {
            inflation_radius_m,
            curve,
            temp_src: None,
            temp_dest: None,
        }
    }

    /// Create using linear cost decay (simple linear falloff).
    pub fn linear(inflation_radius_m: f32) -> Self {
        Self {
            inflation_radius_m,
            curve: InflationCostCurve::Linear,
            temp_src: None,
            temp_dest: None,
        }
    }

    /// Create using inscribed exponential decay (Nav2-style).
    /// Params in cell units; convert from meters using resolution if needed.
    pub fn inscribed(
        inflation_radius_m: f32,
        inscribed_radius_cells: f32,
        cost_scaling_factor: f32,
    ) -> Self {
        Self {
            inflation_radius_m,
            curve: InflationCostCurve::Inscribed {
                inscribed_radius_cells,
                cost_scaling_factor,
            },
            temp_src: None,
            temp_dest: None,
        }
    }
}

impl Layer for InflationLayer {
    fn reset(&mut self) {}

    fn is_clearable(&self) -> bool {
        false
    }

    fn update_bounds(&mut self, _robot: Pose2, bounds: &mut Bounds) {
        bounds.expand_by(self.inflation_radius_m);
    }

    fn update_costs(&mut self, master: &mut Grid2d<u8>, region: CellRegion) {
        let w = region.max.x - region.min.x;
        let h = region.max.y - region.min.y;
        if w == 0 || h == 0 {
            return;
        }

        let resolution = master.info().resolution;
        let reuse = self.temp_src.as_ref().map_or(false, |g| {
            g.width() == w && g.height() == h && g.info().resolution == resolution
        });

        if !reuse {
            let info = MapInfo {
                width: w,
                height: h,
                resolution,
                ..Default::default()
            };
            self.temp_src = Some(Grid2d::<u8>::filled(info, 0));
            self.temp_dest = Some(Grid2d::<u8>::filled(
                MapInfo {
                    width: w,
                    height: h,
                    resolution,
                    ..Default::default()
                },
                0,
            ));
        } else {
            self.temp_src.as_mut().unwrap().clear();
            self.temp_dest.as_mut().unwrap().clear();
        }

        for y in region.min.y..region.max.y {
            for x in region.min.x..region.max.x {
                let cell = UVec2::new(x, y);
                let cost = master.get(cell).copied().unwrap_or(COST_FREE);
                let tx = x - region.min.x;
                let ty = y - region.min.y;
                let _ = self
                    .temp_src
                    .as_mut()
                    .unwrap()
                    .set(UVec2::new(tx, ty), cost);
            }
        }

        let temp_src = self.temp_src.as_ref().unwrap();
        let temp_dest = self.temp_dest.as_mut().unwrap();
        match &self.curve {
            InflationCostCurve::Linear => {
                inflate(
                    temp_src,
                    temp_dest,
                    self.inflation_radius_m,
                    linear_inflation_cost,
                );
            }
            InflationCostCurve::Inscribed {
                inscribed_radius_cells,
                cost_scaling_factor,
            } => {
                inflate_inscribed(
                    temp_src,
                    temp_dest,
                    self.inflation_radius_m,
                    *inscribed_radius_cells,
                    *cost_scaling_factor,
                );
            }
            InflationCostCurve::Custom(f) => {
                inflate(temp_src, temp_dest, self.inflation_radius_m, *f);
            }
        }

        for y in region.min.y..region.max.y {
            for x in region.min.x..region.max.x {
                let tx = x - region.min.x;
                let ty = y - region.min.y;
                let cost = temp_dest
                    .get(UVec2::new(tx, ty))
                    .copied()
                    .unwrap_or(COST_FREE);
                let _ = master.set(UVec2::new(x, y), cost);
            }
        }
    }

    fn match_size(&mut self, _info: &MapInfo) {
        self.temp_src = None;
        self.temp_dest = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{CellRegion, Layer, LayeredGrid2d, Pose2};
    use crate::{MapInfo, types::COST_UNKNOWN};
    use glam::Vec2;

    #[test]
    fn radius_to_cells_basic() {
        assert_eq!(inflation_radius_to_cells(0.5, 0.1), 5);
        assert_eq!(inflation_radius_to_cells(0.05, 0.1), 1);
        assert_eq!(inflation_radius_to_cells(0.0, 0.1), 0);
        assert_eq!(inflation_radius_to_cells(1.0, 0.0), 0);
        assert_eq!(inflation_radius_to_cells(-1.0, 0.1), 0);
    }

    #[test]
    fn radius_to_cells_rounds_up() {
        // 0.15 / 0.1 = 1.5 → ceil → 2
        assert_eq!(inflation_radius_to_cells(0.15, 0.1), 2);
    }

    #[test]
    fn linear_cost_at_zero_is_lethal() {
        assert_eq!(linear_inflation_cost(0.0, 5.0), COST_LETHAL);
    }

    #[test]
    fn linear_cost_at_radius_is_zero() {
        assert_eq!(linear_inflation_cost(5.0, 5.0), 0);
    }

    #[test]
    fn linear_cost_midpoint() {
        let cost = linear_inflation_cost(2.5, 5.0);
        // 50% of COST_LETHAL = 127
        assert_eq!(cost, 127);
    }

    #[test]
    fn linear_cost_beyond_radius_is_zero() {
        assert_eq!(linear_inflation_cost(10.0, 5.0), 0);
    }

    #[test]
    fn inscribed_cost_at_zero_is_lethal() {
        assert_eq!(inscribed_inflation_cost(0.0, 2.0, 3.0), COST_LETHAL);
    }

    #[test]
    fn inscribed_cost_within_inscribed_is_inscribed() {
        assert_eq!(
            inscribed_inflation_cost(1.0, 2.0, 3.0),
            crate::types::COST_INSCRIBED
        );
    }

    #[test]
    fn inscribed_cost_decays_beyond_inscribed() {
        let cost = inscribed_inflation_cost(3.0, 1.0, 1.0);
        assert!(cost < crate::types::COST_INSCRIBED);
        assert!(cost > 0);
    }

    fn make_grid(width: u32, height: u32, data: Vec<u8>) -> Grid2d<u8> {
        Grid2d::new(
            MapInfo {
                width,
                height,
                resolution: 1.0,
                ..Default::default()
            },
            data,
        )
        .unwrap()
    }

    #[test]
    fn inflate_single_lethal_cell() {
        // 5×5 grid with a single lethal cell in the center.
        let mut data = vec![COST_FREE; 25];
        data[2 * 5 + 2] = COST_LETHAL; // cell (2, 2)
        let source = make_grid(5, 5, data);

        let mut dest = Grid2d::<u8>::empty(source.info().clone());
        inflate(&source, &mut dest, 2.0, linear_inflation_cost);

        // Center should be lethal.
        assert_eq!(dest.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        // Adjacent cells should have non-zero cost.
        assert!(dest.get(UVec2::new(3, 2)).copied().unwrap_or(0) > 0);
        assert!(dest.get(UVec2::new(1, 2)).copied().unwrap_or(0) > 0);
        // Corners (distance ~2.83) should be unaffected at radius 2.
        assert_eq!(dest.get(UVec2::new(0, 0)).copied(), Some(COST_FREE));
    }

    #[test]
    fn inflate_empty_grid_stays_free() {
        let source = Grid2d::<u8>::empty(MapInfo {
            width: 10,
            height: 10,
            resolution: 0.5,
            ..Default::default()
        });
        let mut dest = Grid2d::<u8>::empty(source.info().clone());
        inflate(&source, &mut dest, 1.0, linear_inflation_cost);

        for (_, cost) in dest.iter_cells() {
            assert_eq!(*cost, COST_FREE);
        }
    }

    #[test]
    fn inflation_layer_in_layered_grid2d() {
        let info = MapInfo {
            width: 5,
            height: 5,
            resolution: 0.2,
            ..Default::default()
        };

        struct OneLethalLayer;
        impl Layer for OneLethalLayer {
            fn reset(&mut self) {}
            fn is_clearable(&self) -> bool {
                true
            }
            fn update_bounds(&mut self, robot: Pose2, bounds: &mut crate::grid::Bounds) {
                bounds.expand_to_include(robot.position);
                bounds.expand_by(1.0);
            }
            fn update_costs(&mut self, master: &mut Grid2d<u8>, _region: CellRegion) {
                let _ = master.set(UVec2::new(2, 2), COST_LETHAL);
            }
        }

        let mut layered = LayeredGrid2d::new(info, 0, false);
        layered.add_layer(Box::new(OneLethalLayer));
        layered.add_layer(Box::new(InflationLayer::linear(0.5)));
        layered.update_map(Pose2 {
            position: Vec2::new(0.5, 0.5),
            yaw: 0.0,
        });

        let m = layered.master();
        assert_eq!(m.get(UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        let near = m.get(UVec2::new(3, 2)).copied().unwrap_or(0);
        assert!(
            near > 0 && near < COST_LETHAL,
            "inflated cost should be in (0, LETHAL)"
        );
    }

    #[test]
    fn inflate_preserves_unknown_cells() {
        // 5×5 grid: mostly unknown, one free corridor, one lethal cell.
        let mut data = vec![COST_UNKNOWN; 25];
        // Free corridor along row 2.
        for x in 0..5 {
            data[2 * 5 + x] = COST_FREE;
        }
        // Lethal obstacle at (4, 2).
        data[2 * 5 + 4] = COST_LETHAL;
        let source = make_grid(5, 5, data);

        let mut dest = Grid2d::<u8>::empty(source.info().clone());
        inflate(&source, &mut dest, 1.2, linear_inflation_cost);

        // Unknown cells far from any lethal should remain unknown.
        assert_eq!(dest.get(UVec2::new(0, 0)).copied(), Some(COST_UNKNOWN));
        assert_eq!(dest.get(UVec2::new(1, 4)).copied(), Some(COST_UNKNOWN));

        // Free cells far from the lethal should remain free.
        assert_eq!(dest.get(UVec2::new(0, 2)).copied(), Some(COST_FREE));

        // The lethal cell itself should still be lethal.
        assert_eq!(dest.get(UVec2::new(4, 2)).copied(), Some(COST_LETHAL));

        // Adjacent to lethal should have inflation cost (> free).
        assert!(dest.get(UVec2::new(3, 2)).copied().unwrap_or(0) > COST_FREE);
    }
}
