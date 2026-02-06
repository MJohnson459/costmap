//! Obstacle inflation for `Grid2d<u8>` costmaps.
//!
//! Provides a generic inflate operation that expands lethal cells outward with
//! a distance-based cost falloff. The default cost curve is a simple linear
//! decay; Nav2-compatible exponential decay can be layered on top via
//! [`inscribed_inflation_cost`].

use glam::UVec2;

use crate::Grid2d;
use crate::types::{COST_FREE, COST_LETHAL};

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
        let _ = dest.set(&cell, *cost);
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
                let current = dest.get(&pos).copied().unwrap_or(COST_FREE);
                if inflated_cost > current {
                    let _ = dest.set(&pos, inflated_cost);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;
    use crate::{MapInfo, types::COST_UNKNOWN};

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
                depth: 1,
                resolution: 1.0,
                origin: Vec3::ZERO,
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
        assert_eq!(dest.get(&UVec2::new(2, 2)).copied(), Some(COST_LETHAL));
        // Adjacent cells should have non-zero cost.
        assert!(dest.get(&UVec2::new(3, 2)).copied().unwrap_or(0) > 0);
        assert!(dest.get(&UVec2::new(1, 2)).copied().unwrap_or(0) > 0);
        // Corners (distance ~2.83) should be unaffected at radius 2.
        assert_eq!(dest.get(&UVec2::new(0, 0)).copied(), Some(COST_FREE));
    }

    #[test]
    fn inflate_empty_grid_stays_free() {
        let source = Grid2d::<u8>::empty(MapInfo {
            width: 10,
            height: 10,
            depth: 1,
            resolution: 0.5,
            origin: Vec3::ZERO,
        });
        let mut dest = Grid2d::<u8>::empty(source.info().clone());
        inflate(&source, &mut dest, 1.0, linear_inflation_cost);

        for (_, cost) in dest.iter_cells() {
            assert_eq!(*cost, COST_FREE);
        }
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
        assert_eq!(dest.get(&UVec2::new(0, 0)).copied(), Some(COST_UNKNOWN));
        assert_eq!(dest.get(&UVec2::new(1, 4)).copied(), Some(COST_UNKNOWN));

        // Free cells far from the lethal should remain free.
        assert_eq!(dest.get(&UVec2::new(0, 2)).copied(), Some(COST_FREE));

        // The lethal cell itself should still be lethal.
        assert_eq!(dest.get(&UVec2::new(4, 2)).copied(), Some(COST_LETHAL));

        // Adjacent to lethal should have inflation cost (> free).
        assert!(dest.get(&UVec2::new(3, 2)).copied().unwrap_or(0) > COST_FREE);
    }
}
