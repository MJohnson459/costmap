pub mod edt;
mod wavefront;

pub use edt::DistanceField;
pub use wavefront::WavefrontInflationLayer;

/// Configuration for the inflation layer.
///
/// Groups parameters that control inflation behaviour. Defaults match Nav2.
#[derive(Debug, Clone)]
pub struct InflationConfig {
    /// When true, allow overwriting unknown cells with inflation cost if cost > FREE.
    pub inflate_unknown: bool,
    /// When true, treat COST_UNKNOWN cells as obstacle seeds (same as lethal).
    pub inflate_around_unknown: bool,
    /// Inflation radius in meters.
    pub inflation_radius_m: f32,
    /// Cost scaling factor for exponential decay (world units).
    pub cost_scaling_factor: f32,
    /// Inscribed radius in meters (inside this, cost is COST_INSCRIBED).
    pub inscribed_radius_m: f32,
}

impl Default for InflationConfig {
    fn default() -> Self {
        Self {
            inflate_unknown: false,
            inflate_around_unknown: false,
            inflation_radius_m: 0.55,
            cost_scaling_factor: 10.0,
            inscribed_radius_m: 0.0,
        }
    }
}
