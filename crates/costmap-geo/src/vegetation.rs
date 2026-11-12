//! Vegetation from the orthophoto, judged over a neighbourhood rather than per pixel.
//!
//! A sample is green when its excess-green index exceeds the threshold. A cell is vegetation
//! when at least a fraction of the known samples in the square box centred on it are green,
//! so isolated green pixels do not count while a crop row a few boxes wide does. Samples
//! without data, outside the grid or with no orthophoto are in neither count.
//!
//! The box comes from [`odd_box_radius`], the rule the slope box uses, with a minimum of one
//! cell: a window under one cell reduces to the per-cell test. Both counts come from
//! summed-area tables, so a cell costs a few lookups whatever the window.

use costmap::DenseGrid;
use glam::UVec2;

use crate::frame::cells;

use crate::{
    rules::{Colour, CostRules, excess_green},
    slope::odd_box_radius,
};

/// Smallest vegetation box radius: a single cell.
const MIN_VEGETATION_RADIUS: u32 = 0;

/// Radius in cells of the vegetation box for a window of `window_m` metres.
pub fn box_radius(resolution: f64, window_m: f64) -> u32 {
    odd_box_radius(resolution, window_m, MIN_VEGETATION_RADIUS)
}

/// True if the colour is green enough to count towards vegetation.
fn is_green(rgb: [u8; 3], exg_threshold: f32) -> bool {
    excess_green(rgb).is_some_and(|exg| exg > exg_threshold)
}

/// Whether each cell of `colours` is vegetation under `rules`, using boxes of
/// `2 * radius + 1` cells. Boxes are clipped at the grid edge, so callers keep a halo of
/// `radius` cells around the cells they use.
pub fn vegetation_mask(
    colours: &DenseGrid<Colour>,
    rules: &CostRules,
    radius: u32,
) -> DenseGrid<bool> {
    let known = AreaTable::new(colours, |colour| matches!(colour, Colour::Rgb(_)));
    let green = AreaTable::new(
        colours,
        |colour| matches!(colour, Colour::Rgb(rgb) if is_green(rgb, rules.exg_threshold)),
    );
    DenseGrid::from_fn(cells(colours.size()), |x, y| {
        let known = known.box_sum(x, y, radius);
        known > 0
            && f64::from(green.box_sum(x, y, radius)) / f64::from(known)
                >= rules.vegetation_min_fraction
    })
}

/// Counts of cells satisfying a predicate, as a summed-area table.
struct AreaTable {
    size: UVec2,
    /// Count over `[0, x) x [0, y)`, stored with one extra row and column.
    sums: Vec<u32>,
}

impl AreaTable {
    fn new(colours: &DenseGrid<Colour>, counts: impl Fn(Colour) -> bool) -> Self {
        let size = colours.size();
        let stride = size.x as usize + 1;
        let mut sums = vec![0; stride * (size.y as usize + 1)];
        for y in 0..size.y {
            let mut row_count = 0;
            for x in 0..size.x {
                row_count += u32::from(counts(colours[UVec2::new(x, y)]));
                let here = (y as usize + 1) * stride + x as usize + 1;
                sums[here] = sums[here - stride] + row_count;
            }
        }
        Self { size, sums }
    }

    /// Count in the box of `2 * radius + 1` cells centred on `(x, y)`, clipped to the grid.
    fn box_sum(&self, x: u32, y: u32, radius: u32) -> u32 {
        let stride = self.size.x as usize + 1;
        let low = |centre: u32| centre.saturating_sub(radius) as usize;
        let high =
            |centre: u32, extent: u32| centre.saturating_add(radius + 1).min(extent) as usize;
        let (x0, y0) = (low(x), low(y));
        let (x1, y1) = (high(x, self.size.x), high(y, self.size.y));
        let corner = |cx: usize, cy: usize| self.sums[cy * stride + cx];
        corner(x1, y1) + corner(x0, y0) - corner(x0, y1) - corner(x1, y0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GREEN: Colour = Colour::Rgb([40, 160, 40]);
    const BARE: Colour = Colour::Rgb([150, 140, 120]);
    /// 0.15 m at 0.05 m cells: the default window, a box of 3 cells.
    const RES: f64 = 0.05;
    const DEFAULT_RADIUS: u32 = 1;

    fn rules(min_fraction: f64) -> CostRules {
        CostRules {
            vegetation_min_fraction: min_fraction,
            ..CostRules::default()
        }
    }

    fn scene(size: u32, colour: impl Fn(u32, u32) -> Colour) -> DenseGrid<Colour> {
        DenseGrid::from_fn(cells(UVec2::splat(size)), colour)
    }

    fn centre_is_vegetation(patch: [[Colour; 3]; 3], min_fraction: f64) -> bool {
        let colours = scene(3, |x, y| patch[y as usize][x as usize]);
        vegetation_mask(&colours, &rules(min_fraction), 1)[UVec2::new(1, 1)]
    }

    #[test]
    fn default_window_is_a_three_cell_box_at_five_centimetres() {
        let defaults = CostRules::default();
        assert_eq!(
            box_radius(RES, defaults.vegetation_window_m),
            DEFAULT_RADIUS
        );
        assert_eq!(box_radius(0.03, defaults.vegetation_window_m), 2);
    }

    #[test]
    fn window_below_two_cells_is_a_single_cell() {
        assert_eq!(box_radius(0.05, 0.0), 0);
        assert_eq!(box_radius(0.05, 0.04), 0);
        assert_eq!(box_radius(0.2, 0.15), 0);
        assert_eq!(box_radius(0.05, 0.05), 0);
        assert_eq!(box_radius(0.05, 0.1), 0);
    }

    #[test]
    fn solid_band_is_vegetation_across_its_width_and_no_further() {
        let band = 15..25;
        let colours = scene(40, |x, _| if band.contains(&x) { GREEN } else { BARE });
        for radius in [DEFAULT_RADIUS, 3] {
            let mask = vegetation_mask(&colours, &rules(0.5), radius);
            for y in [0, 20, 39] {
                for x in 0..40 {
                    assert_eq!(
                        mask[UVec2::new(x, y)],
                        band.contains(&x),
                        "r {radius} x {x} y {y}"
                    );
                }
            }
        }
    }

    #[test]
    fn single_green_pixel_is_vegetation_only_at_a_one_cell_window() {
        let colours = scene(20, |x, y| if (x, y) == (10, 10) { GREEN } else { BARE });
        let default = vegetation_mask(&colours, &rules(0.5), DEFAULT_RADIUS);
        assert!(default.as_slice().iter().all(|&v| !v));
        let per_cell = vegetation_mask(&colours, &rules(0.5), 0);
        let count = per_cell.as_slice().iter().filter(|&&v| v).count();
        assert_eq!((count, per_cell[UVec2::new(10, 10)]), (1, true));
    }

    #[test]
    fn fraction_exactly_at_the_threshold_counts() {
        let (g, b, n) = (GREEN, BARE, Colour::NoData);
        assert!(centre_is_vegetation([[g, g, g], [g, b, n], [b, b, b]], 0.5));
        assert!(!centre_is_vegetation(
            [[g, g, g], [g, b, b], [b, b, b]],
            0.5
        ));
        assert!(centre_is_vegetation([[g, g, g], [g, g, b], [b, b, b]], 0.5));
        assert!(centre_is_vegetation(
            [[g, g, g], [b, b, b], [b, b, b]],
            1.0 / 3.0
        ));
        assert!(!centre_is_vegetation(
            [[g, g, b], [b, b, b], [b, b, b]],
            1.0 / 3.0
        ));
        assert!(centre_is_vegetation([[g, g, g], [g, g, g], [g, g, g]], 1.0));
        assert!(!centre_is_vegetation(
            [[g, g, g], [g, g, g], [g, g, b]],
            1.0
        ));
    }

    #[test]
    fn unknown_samples_are_left_out_of_both_counts() {
        let (g, b, n) = (GREEN, BARE, Colour::NoData);
        assert!(centre_is_vegetation([[g, g, n], [b, b, n], [n, n, n]], 0.5));
        assert!(!centre_is_vegetation(
            [[g, b, n], [b, b, n], [n, n, n]],
            0.5
        ));
    }

    #[test]
    fn cell_without_known_samples_is_not_vegetation() {
        let n = Colour::NoData;
        assert!(!centre_is_vegetation([[n; 3]; 3], 0.5));
        let absent = Colour::Absent;
        assert!(!centre_is_vegetation([[absent; 3]; 3], 0.5));
    }

    #[test]
    fn samples_outside_the_grid_are_left_out_of_both_counts() {
        let colours = scene(10, |_, _| GREEN);
        let mask = vegetation_mask(&colours, &rules(1.0), 3);
        assert!(mask.as_slice().iter().all(|&v| v));
    }

    #[test]
    fn threshold_decides_which_samples_are_green() {
        let colours = scene(5, |_, _| Colour::Rgb([60, 100, 60]));
        let exg = excess_green([60, 100, 60]).unwrap();
        let below = CostRules {
            exg_threshold: exg - 0.01,
            ..CostRules::default()
        };
        let above = CostRules {
            exg_threshold: exg + 0.01,
            ..CostRules::default()
        };
        assert!(vegetation_mask(&colours, &below, 1)[UVec2::new(2, 2)]);
        assert!(!vegetation_mask(&colours, &above, 1)[UVec2::new(2, 2)]);
    }
}
