//! Cost scale conversion utilities for mapping physical/semantic values to u8 costs.

use super::constants::{COST_FREE, COST_LETHAL};

/// Convert a value in [0, 1] to a costmap cost in [COST_FREE, COST_LETHAL].
///
/// Values outside [0, 1] are clamped: values ≤ 0 → COST_FREE, values ≥ 1 → COST_LETHAL.
/// Values in between are linearly interpolated and rounded to the nearest u8.
pub fn cost_from_unit(x: f32) -> u8 {
    let clamped = x.clamp(0.0, 1.0);
    let cost_range = (COST_LETHAL as f32) - (COST_FREE as f32);
    let scaled = (COST_FREE as f32) + clamped * cost_range;
    scaled.round() as u8
}

/// Convert a value in range [lo, hi] to a costmap cost in [COST_FREE, COST_LETHAL].
///
/// The value `x` is normalized into [0, 1] based on the range [lo, hi],
/// then converted to a cost using [`cost_from_unit`].
/// If `hi <= lo`, returns COST_FREE (no valid range).
pub fn cost_from_range(x: f32, lo: f32, hi: f32) -> u8 {
    if hi <= lo {
        return COST_FREE;
    }
    let normalized = (x - lo) / (hi - lo);
    cost_from_unit(normalized)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cost_from_unit_boundaries() {
        assert_eq!(cost_from_unit(0.0), COST_FREE);
        assert_eq!(cost_from_unit(1.0), COST_LETHAL);
    }

    #[test]
    fn test_cost_from_unit_clamping() {
        // Out of range values should be clamped
        assert_eq!(cost_from_unit(-1.0), COST_FREE);
        assert_eq!(cost_from_unit(-100.0), COST_FREE);
        assert_eq!(cost_from_unit(2.0), COST_LETHAL);
        assert_eq!(cost_from_unit(100.0), COST_LETHAL);
    }

    #[test]
    fn test_cost_from_unit_midpoint() {
        let mid = cost_from_unit(0.5);
        // Should be approximately in the middle of the range
        let expected = ((COST_FREE as f32 + COST_LETHAL as f32) / 2.0).round() as u8;
        // Allow ±1 for rounding differences
        assert!((mid as i16 - expected as i16).abs() <= 1);
    }

    #[test]
    fn test_cost_from_range_valid() {
        // Range [0, 10] with value 5 should map to 0.5 unit → ~127
        let cost = cost_from_range(5.0, 0.0, 10.0);
        let expected_unit = cost_from_unit(0.5);
        assert_eq!(cost, expected_unit);
    }

    #[test]
    fn test_cost_from_range_invalid() {
        // hi <= lo should return COST_FREE
        assert_eq!(cost_from_range(5.0, 10.0, 10.0), COST_FREE);
        assert_eq!(cost_from_range(5.0, 10.0, 5.0), COST_FREE);
    }

    #[test]
    fn test_cost_from_range_bounds() {
        assert_eq!(cost_from_range(0.0, 0.0, 10.0), COST_FREE);
        assert_eq!(cost_from_range(10.0, 0.0, 10.0), COST_LETHAL);
    }
}
