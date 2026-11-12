//! Cost rules for a single cell. Every rule is a pure function of its inputs.

use costmap::types::{COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN};

use crate::error::GeoError;

/// Highest graded cost; 253 and above are reserved for inscribed, lethal and unknown.
pub const COST_MAX_GRADED: u8 = COST_INSCRIBED - 1;

/// Highest cost the vegetation rule may set: lethal. Unknown (255) is never a rule output.
pub const VEGETATION_COST_MAX: u8 = COST_LETHAL;

/// Tunable parameters of the cost rules.
#[derive(Debug, Clone, PartialEq)]
pub struct CostRules {
    /// Output cell size in metres; `None` uses the DTM's pixel size.
    pub resolution: Option<f64>,
    /// Height above ground (DSM minus DTM) from which a cell is lethal, in metres.
    pub obstacle_height: f32,
    /// Edge length in metres of the area the ground slope is averaged over.
    pub slope_window_m: f64,
    /// Slope in degrees up to which a cell costs nothing.
    pub slope_free_deg: f32,
    /// Slope in degrees from which a cell is lethal.
    pub slope_lethal_deg: f32,
    /// Minimum cost of vegetated cells, up to 254 (lethal); 0 disables the vegetation rule.
    pub vegetation_cost: u8,
    /// Excess-green index above which an orthophoto sample counts as green.
    pub exg_threshold: f32,
    /// Edge in metres of the square box around a cell in which green samples are counted.
    pub vegetation_window_m: f64,
    /// Fraction of the known samples in the box that must be green for the cell to be
    /// vegetation, in `(0, 1]`.
    pub vegetation_min_fraction: f64,
}

impl Default for CostRules {
    fn default() -> Self {
        Self {
            resolution: None,
            obstacle_height: 0.3,
            slope_window_m: 1.0,
            slope_free_deg: 10.0,
            slope_lethal_deg: 25.0,
            vegetation_cost: 0,
            exg_threshold: 0.1,
            vegetation_window_m: 0.15,
            vegetation_min_fraction: 0.5,
        }
    }
}

/// What the orthophoto says about a cell.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Colour {
    /// No orthophoto was given.
    #[default]
    Absent,
    /// The orthophoto has no data at the cell.
    NoData,
    /// Red, green and blue of the cell.
    Rgb([u8; 3]),
}

/// Everything the rules need to know about one cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellInput {
    /// Surface height in metres, if known.
    pub dsm: Option<f32>,
    /// Ground height in metres, if known.
    pub dtm: Option<f32>,
    /// Ground slope in degrees.
    pub slope_deg: f32,
    /// An orthophoto was given and has no data at the cell.
    pub ortho_nodata: bool,
    /// Enough of the orthophoto around the cell is green; see [`crate::vegetation`].
    pub vegetation: bool,
}

impl CostRules {
    /// Check the parameters once, before any raster is read.
    pub fn validate(&self) -> Result<(), GeoError> {
        let bad = |message: &str| Err(GeoError::Rules(message.into()));
        if self.resolution.is_some_and(|r| !(r.is_finite() && r > 0.0)) {
            return bad("resolution must be a positive number of metres");
        }
        if !(self.obstacle_height.is_finite() && self.obstacle_height > 0.0) {
            return bad("obstacle height must be a positive number of metres");
        }
        if !(self.slope_window_m.is_finite() && self.slope_window_m > 0.0) {
            return bad("slope window must be a positive number of metres");
        }
        let angles_ok = self.slope_free_deg.is_finite()
            && self.slope_lethal_deg.is_finite()
            && self.slope_free_deg >= 0.0
            && self.slope_free_deg < self.slope_lethal_deg
            && self.slope_lethal_deg <= 90.0;
        if !angles_ok {
            return bad("slope angles must satisfy 0 <= free < lethal <= 90 degrees");
        }
        if self.vegetation_cost > VEGETATION_COST_MAX {
            return bad("vegetation cost must be at most 254");
        }
        if !self.exg_threshold.is_finite() {
            return bad("excess-green threshold must be finite");
        }
        if !(self.vegetation_window_m.is_finite() && self.vegetation_window_m >= 0.0) {
            return bad("vegetation window must be a non-negative number of metres");
        }
        if !(self.vegetation_min_fraction > 0.0 && self.vegetation_min_fraction <= 1.0) {
            return bad("vegetation minimum fraction must be above 0 and at most 1");
        }
        Ok(())
    }

    /// True if the surface stands at least the obstacle height above the ground.
    pub fn is_obstacle(&self, dsm: f32, dtm: f32) -> bool {
        dsm - dtm >= self.obstacle_height
    }

    /// Cost of a ground slope: 0 up to the free angle, a linear ramp to 252 at the
    /// lethal angle, and lethal from there.
    pub fn slope_cost(&self, slope_deg: f32) -> u8 {
        if slope_deg >= self.slope_lethal_deg {
            return COST_LETHAL;
        }
        if slope_deg <= self.slope_free_deg {
            return 0;
        }
        let fraction =
            (slope_deg - self.slope_free_deg) / (self.slope_lethal_deg - self.slope_free_deg);
        (fraction * f32::from(COST_MAX_GRADED)).round() as u8
    }

    /// Cost of a cell from all rules except overrides.
    pub fn cell_cost(&self, cell: &CellInput) -> u8 {
        let (Some(dsm), Some(dtm)) = (cell.dsm, cell.dtm) else {
            return COST_UNKNOWN;
        };
        if cell.ortho_nodata {
            return COST_UNKNOWN;
        }
        if self.is_obstacle(dsm, dtm) {
            return COST_LETHAL;
        }
        let cost = self.slope_cost(cell.slope_deg);
        if cell.vegetation {
            cost.max(self.vegetation_cost)
        } else {
            cost
        }
    }
}

/// Excess-green index `(2G - R - B) / (R + G + B)`, or `None` for a black pixel.
pub fn excess_green([r, g, b]: [u8; 3]) -> Option<f32> {
    let sum = f32::from(r) + f32::from(g) + f32::from(b);
    (sum > 0.0).then(|| (2.0 * f32::from(g) - f32::from(r) - f32::from(b)) / sum)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(dsm: Option<f32>, dtm: Option<f32>, slope_deg: f32, vegetation: bool) -> CellInput {
        CellInput {
            dsm,
            dtm,
            slope_deg,
            ortho_nodata: false,
            vegetation,
        }
    }

    fn flat(height: f32) -> CellInput {
        cell(Some(100.0 + height), Some(100.0), 0.0, false)
    }

    fn vegetated(cost: u8) -> CostRules {
        CostRules {
            vegetation_cost: cost,
            ..CostRules::default()
        }
    }

    #[test]
    fn defaults_validate() {
        assert!(CostRules::default().validate().is_ok());
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        let default = CostRules::default();
        let cases = [
            CostRules {
                resolution: Some(0.0),
                ..default.clone()
            },
            CostRules {
                resolution: Some(f64::NAN),
                ..default.clone()
            },
            CostRules {
                obstacle_height: 0.0,
                ..default.clone()
            },
            CostRules {
                slope_window_m: -1.0,
                ..default.clone()
            },
            CostRules {
                slope_free_deg: 25.0,
                ..default.clone()
            },
            CostRules {
                slope_free_deg: -1.0,
                ..default.clone()
            },
            CostRules {
                slope_lethal_deg: 91.0,
                ..default.clone()
            },
            CostRules {
                vegetation_cost: 255,
                ..default.clone()
            },
            CostRules {
                exg_threshold: f32::INFINITY,
                ..default.clone()
            },
            CostRules {
                vegetation_window_m: -0.1,
                ..default.clone()
            },
            CostRules {
                vegetation_window_m: f64::NAN,
                ..default.clone()
            },
            CostRules {
                vegetation_min_fraction: 0.0,
                ..default.clone()
            },
            CostRules {
                vegetation_min_fraction: 1.01,
                ..default.clone()
            },
            CostRules {
                vegetation_min_fraction: f64::NAN,
                ..default
            },
        ];
        for rules in cases {
            assert!(rules.validate().is_err(), "{rules:?}");
        }
    }

    #[test]
    fn height_rule_switches_at_the_obstacle_height() {
        let rules = CostRules::default();
        assert!(!rules.is_obstacle(100.29, 100.0));
        assert!(rules.is_obstacle(100.31, 100.0));
        assert_eq!(rules.cell_cost(&flat(0.1)), 0);
        assert_eq!(rules.cell_cost(&flat(2.0)), COST_LETHAL);
    }

    #[test]
    fn negative_height_is_not_an_obstacle() {
        assert_eq!(CostRules::default().cell_cost(&flat(-0.5)), 0);
    }

    #[test]
    fn slope_cost_ramps_between_free_and_lethal_angles() {
        let rules = CostRules::default();
        assert_eq!(rules.slope_cost(5.0), 0);
        assert_eq!(rules.slope_cost(10.0), 0);
        assert_eq!(rules.slope_cost(15.0), 84);
        assert_eq!(rules.slope_cost(17.5), 126);
        assert_eq!(rules.slope_cost(24.99), 252);
        assert_eq!(rules.slope_cost(25.0), COST_LETHAL);
        assert_eq!(rules.slope_cost(30.0), COST_LETHAL);
    }

    #[test]
    fn slope_cost_never_reaches_reserved_values_below_the_lethal_angle() {
        let rules = CostRules::default();
        let max = (0..2500)
            .map(|i| rules.slope_cost(i as f32 / 100.0))
            .filter(|&c| c != COST_LETHAL)
            .max();
        assert_eq!(max, Some(COST_MAX_GRADED));
    }

    #[test]
    fn missing_surface_or_ground_is_unknown() {
        let rules = CostRules::default();
        let ground = Some(100.0);
        assert_eq!(
            rules.cell_cost(&cell(None, ground, 0.0, false)),
            COST_UNKNOWN
        );
        assert_eq!(
            rules.cell_cost(&cell(ground, None, 0.0, false)),
            COST_UNKNOWN
        );
    }

    #[test]
    fn missing_orthophoto_data_is_unknown_even_over_an_obstacle() {
        let rules = CostRules::default();
        let blank = CellInput {
            ortho_nodata: true,
            ..cell(Some(103.0), Some(100.0), 0.0, false)
        };
        assert_eq!(rules.cell_cost(&blank), COST_UNKNOWN);
    }

    #[test]
    fn absent_orthophoto_does_not_make_cells_unknown() {
        assert_eq!(CostRules::default().cell_cost(&flat(0.0)), 0);
    }

    #[test]
    fn excess_green_of_known_colours() {
        assert_eq!(excess_green([0, 0, 0]), None);
        assert_eq!(excess_green([0, 255, 0]), Some(2.0));
        assert_eq!(excess_green([100, 100, 100]), Some(0.0));
        let red = excess_green([200, 50, 50]);
        assert!(red.is_some_and(|v| v < 0.0));
    }

    #[test]
    fn vegetation_rule_raises_vegetated_cells_only() {
        let rules = vegetated(120);
        let ground = |vegetation| cell(Some(100.0), Some(100.0), 0.0, vegetation);
        assert_eq!(rules.cell_cost(&ground(true)), 120);
        assert_eq!(rules.cell_cost(&ground(false)), 0);
    }

    #[test]
    fn vegetation_cost_254_is_accepted_and_lethal() {
        let rules = vegetated(VEGETATION_COST_MAX);
        assert!(rules.validate().is_ok());
        let green = cell(Some(100.0), Some(100.0), 0.0, true);
        assert_eq!(rules.cell_cost(&green), COST_LETHAL);
    }

    #[test]
    fn vegetation_cost_255_is_rejected() {
        assert!(vegetated(COST_UNKNOWN).validate().is_err());
    }

    #[test]
    fn vegetation_rule_never_lowers_a_cost() {
        let rules = vegetated(120);
        let steep = cell(Some(100.0), Some(100.0), 20.0, true);
        assert_eq!(rules.cell_cost(&steep), rules.slope_cost(20.0));
        assert!(rules.slope_cost(20.0) > 120);
        let tall = cell(Some(103.0), Some(100.0), 0.0, true);
        let cliff = cell(Some(100.0), Some(100.0), 40.0, true);
        assert_eq!(rules.cell_cost(&tall), COST_LETHAL);
        assert_eq!(rules.cell_cost(&cliff), COST_LETHAL);
    }

    #[test]
    fn vegetation_rule_leaves_unknown_alone() {
        let rules = vegetated(VEGETATION_COST_MAX);
        let hole = cell(None, Some(100.0), 0.0, true);
        let blank = CellInput {
            ortho_nodata: true,
            ..cell(Some(100.0), Some(100.0), 0.0, true)
        };
        assert_eq!(rules.cell_cost(&hole), COST_UNKNOWN);
        assert_eq!(rules.cell_cost(&blank), COST_UNKNOWN);
    }

    #[test]
    fn vegetation_rule_is_off_by_default() {
        let green = cell(Some(100.0), Some(100.0), 0.0, true);
        assert_eq!(CostRules::default().cell_cost(&green), 0);
    }
}
