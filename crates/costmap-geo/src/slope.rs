//! Ground slope from a height grid, measured at the scale of a window rather than a cell.
//!
//! Heights are first averaged over a square box. The slope at a cell is the central
//! difference of two box means one box radius either side of it. Averaging over hundreds
//! of cells keeps centimetre-scale noise in the heights from reading as steep ground.
//!
//! The box is the largest odd number of cells that fits in the window, and at least 3 so
//! a central difference exists. Its edge is therefore `2 * radius + 1` cells, never more
//! than the window unless the window is under 3 cells, and the baseline between the two
//! box centres is `2 * radius` cells.

use costmap::DenseGrid;
use glam::UVec2;

use crate::frame::cells;

/// A box mean needs at least this fraction of the box's cells to hold data.
const MIN_VALID_FRACTION: f64 = 0.5;

/// Slack when dividing the window by the resolution, so a 0.3 m window of 0.1 m cells
/// counts 3 cells, not 2.999...
const CELL_COUNT_EPSILON: f64 = 1e-9;

/// Smallest slope box radius: a box of 3 cells, so a central difference exists.
const MIN_SLOPE_RADIUS: u32 = 1;

/// Radius in cells of the square box that fits in a window of `window_m` metres.
///
/// The box is `2 * radius + 1` cells wide: the largest odd cell count that does not
/// exceed the window, and at least `2 * min_radius + 1`. Every box-shaped rule derives its
/// box from this function.
pub fn odd_box_radius(resolution: f64, window_m: f64, min_radius: u32) -> u32 {
    let cells = (window_m / resolution + CELL_COUNT_EPSILON).floor() as u32;
    (cells.saturating_sub(1) / 2).max(min_radius)
}

/// Radius in cells of the slope averaging box for a window of `window_m` metres.
///
/// The box is `2 * radius + 1` cells wide: the largest odd cell count that does not
/// exceed the window, and at least 3.
pub fn box_radius(resolution: f64, window_m: f64) -> u32 {
    odd_box_radius(resolution, window_m, MIN_SLOPE_RADIUS)
}

/// Edge length in metres of the averaging box of `radius` cells.
pub fn box_metres(resolution: f64, radius: u32) -> f64 {
    (2.0 * f64::from(radius) + 1.0) * resolution
}

/// Distance in metres between the two box means a slope difference is taken over.
pub fn baseline_metres(resolution: f64, radius: u32) -> f64 {
    2.0 * f64::from(radius) * resolution
}

/// Cells of surrounding data a block needs for its edge cells to have a full slope window.
pub fn halo_cells(resolution: f64, window_m: f64) -> u32 {
    box_radius(resolution, window_m).saturating_mul(2)
}

/// Slope angle in degrees of every cell of `heights`, where NaN marks missing data.
///
/// An axis whose two box means are not both available contributes no gradient, so a
/// cell without enough data nearby reads as flat. Cells within two box radii of the
/// grid edge also lack neighbours; callers keep a halo of [`halo_cells`] around the
/// cells they use.
pub fn slope_degrees(heights: &DenseGrid<f32>, resolution: f64, window_m: f64) -> DenseGrid<f32> {
    let radius = i64::from(box_radius(resolution, window_m));
    let means = BoxMeans::new(heights, radius);
    let baseline = baseline_metres(resolution, radius as u32);
    DenseGrid::new_with(cells(heights.size()), |x, y| {
        let (x, y) = (i64::from(x), i64::from(y));
        let gradient = |a: Option<f64>, b: Option<f64>| match (a, b) {
            (Some(a), Some(b)) => (b - a) / baseline,
            _ => 0.0,
        };
        let gx = gradient(means.at(x - radius, y), means.at(x + radius, y));
        let gy = gradient(means.at(x, y - radius), means.at(x, y + radius));
        gx.hypot(gy).atan().to_degrees() as f32
    })
}

/// Box means of a height grid from summed-area tables, so each mean costs four lookups.
struct BoxMeans {
    size: UVec2,
    radius: i64,
    /// Sums and counts over `[0, x) x [0, y)`, stored with one extra row and column.
    sums: Vec<f64>,
    counts: Vec<u32>,
    min_count: u32,
}

impl BoxMeans {
    fn new(heights: &DenseGrid<f32>, radius: i64) -> Self {
        let size = heights.size();
        let stride = size.x as usize + 1;
        let mut sums = vec![0.0; stride * (size.y as usize + 1)];
        let mut counts = vec![0; stride * (size.y as usize + 1)];
        for y in 0..size.y {
            let (mut row_sum, mut row_count) = (0.0, 0);
            for x in 0..size.x {
                let height = heights[UVec2::new(x, y)];
                if !height.is_nan() {
                    row_sum += f64::from(height);
                    row_count += 1;
                }
                let here = (y as usize + 1) * stride + x as usize + 1;
                let above = here - stride;
                sums[here] = sums[above] + row_sum;
                counts[here] = counts[above] + row_count;
            }
        }
        let side = 2.0 * radius as f64 + 1.0;
        Self {
            size,
            radius,
            sums,
            counts,
            min_count: (MIN_VALID_FRACTION * side * side).ceil() as u32,
        }
    }

    /// Mean of the box centred on `(x, y)`, clipped to the grid, or `None` if the centre is
    /// outside the grid or the box holds too few valid cells.
    fn at(&self, x: i64, y: i64) -> Option<f64> {
        if x < 0 || y < 0 || x >= i64::from(self.size.x) || y >= i64::from(self.size.y) {
            return None;
        }
        let stride = self.size.x as usize + 1;
        let x0 = (x - self.radius).max(0) as usize;
        let y0 = (y - self.radius).max(0) as usize;
        let x1 = ((x + self.radius + 1).min(i64::from(self.size.x))) as usize;
        let y1 = ((y + self.radius + 1).min(i64::from(self.size.y))) as usize;
        let corner = |cx: usize, cy: usize| cy * stride + cx;
        let count = self.counts[corner(x1, y1)] + self.counts[corner(x0, y0)]
            - self.counts[corner(x0, y1)]
            - self.counts[corner(x1, y0)];
        if count < self.min_count {
            return None;
        }
        let sum = self.sums[corner(x1, y1)] + self.sums[corner(x0, y0)]
            - self.sums[corner(x0, y1)]
            - self.sums[corner(x1, y0)];
        Some(sum / f64::from(count))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RES: f64 = 0.05;
    const SIZE: UVec2 = UVec2::new(120, 120);

    fn plane(angle_deg: f64) -> DenseGrid<f32> {
        let gradient = angle_deg.to_radians().tan();
        DenseGrid::new_with(cells(SIZE), |x, _| {
            (100.0 + f64::from(x) * RES * gradient) as f32
        })
    }

    fn centre_slope(heights: &DenseGrid<f32>) -> f32 {
        slope_degrees(heights, RES, 1.0)[UVec2::new(60, 60)]
    }

    #[test]
    fn box_is_the_largest_odd_cell_count_within_the_window() {
        let cases = [
            (0.05, 1.0, 9, 0.95),
            (0.2, 1.0, 2, 1.0),
            (0.6, 1.0, 1, 1.8),
            (1.0, 1.0, 1, 3.0),
            (0.1, 0.3, 1, 0.3),
            (0.1, 0.25, 1, 0.3),
            (0.05, 0.01, 1, 0.15),
            (0.05, 2.0, 19, 1.95),
        ];
        for (resolution, window, radius, box_m) in cases {
            assert_eq!(
                box_radius(resolution, window),
                radius,
                "{resolution} {window}"
            );
            assert!(
                (box_metres(resolution, radius) - box_m).abs() < 1e-9,
                "{resolution} {window}"
            );
        }
        assert_eq!(halo_cells(0.05, 1.0), 18);
    }

    #[test]
    fn box_fits_the_window_except_below_three_cells() {
        for resolution in [0.03, 0.05, 0.1, 0.2, 0.25, 0.6, 1.0] {
            for window in [0.5, 1.0, 1.5, 3.0, 7.3] {
                let radius = box_radius(resolution, window);
                let fits = box_metres(resolution, radius) <= window + 1e-9;
                assert!(fits || radius == 1, "{resolution} {window} {radius}");
                assert!(radius == 1 || box_metres(resolution, radius + 1) > window);
                assert!(
                    (baseline_metres(resolution, radius)
                        - (box_metres(resolution, radius) - resolution))
                        .abs()
                        < 1e-9
                );
            }
        }
    }

    #[test]
    fn tilted_planes_give_their_angle() {
        for angle in [0.0, 5.0, 15.0, 30.0] {
            let measured = centre_slope(&plane(angle));
            assert!(
                (f64::from(measured) - angle).abs() < 0.05,
                "{angle}: {measured}"
            );
        }
    }

    #[test]
    fn slope_is_the_gradient_magnitude_for_a_diagonal_plane() {
        let gradient = 10.0_f64.to_radians().tan();
        let heights = DenseGrid::new_with(cells(SIZE), |x, y| {
            (100.0 + (f64::from(x) + f64::from(y)) * RES * gradient) as f32
        });
        let expected = (gradient * 2.0_f64.sqrt()).atan().to_degrees();
        assert!((f64::from(centre_slope(&heights)) - expected).abs() < 0.05);
    }

    #[test]
    fn noisy_flat_ground_stays_nearly_flat() {
        let mut state = 12345_u32;
        let mut noise = || {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (f64::from(state >> 8) / f64::from(1_u32 << 24) - 0.5) * 0.1
        };
        let heights = DenseGrid::new_with(cells(SIZE), |_, _| (100.0 + noise()) as f32);
        let slopes = slope_degrees(&heights, RES, 1.0);
        let worst = (40..80)
            .flat_map(|y| (40..80).map(move |x| (x, y)))
            .map(|(x, y)| slopes[UVec2::new(x, y)])
            .fold(0.0, f32::max);
        assert!(worst < 1.0, "worst slope {worst}");
    }

    #[test]
    fn missing_cells_are_left_out_of_the_mean() {
        let mut heights = plane(15.0);
        for y in 0..SIZE.y {
            for x in 0..SIZE.x {
                if (x * 7 + y * 13) % 5 == 0 {
                    heights[UVec2::new(x, y)] = f32::NAN;
                }
            }
        }
        let measured = centre_slope(&heights);
        assert!((measured - 15.0).abs() < 0.3, "{measured}");
    }

    #[test]
    fn too_little_data_reads_as_flat() {
        let mut heights = plane(30.0);
        for y in 0..SIZE.y {
            for x in 0..SIZE.x {
                if x % 4 != 0 || y % 4 != 0 {
                    heights[UVec2::new(x, y)] = f32::NAN;
                }
            }
        }
        assert_eq!(centre_slope(&heights), 0.0);
    }

    #[test]
    fn cells_without_neighbours_read_as_flat() {
        assert_eq!(
            slope_degrees(&plane(30.0), RES, 1.0)[UVec2::new(0, 60)],
            0.0
        );
    }
}
