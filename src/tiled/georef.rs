use glam::DVec2;

/// Where the local frame sits in a projected coordinate reference system.
///
/// UTM eastings are around 500,000 m, and `f32` has a 24-bit mantissa, so an
/// `f32` near that value resolves only about 3 cm. World positions are therefore
/// `f64`; per-cell arithmetic narrows to `f32` metres relative to a grid origin or
/// [`anchor`](Self::anchor).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoReference {
    /// EPSG code of the projected coordinate reference system, if known.
    pub epsg: Option<u32>,
    /// Projected coordinates (easting, northing) in metres of the local frame origin.
    pub anchor: DVec2,
}

impl GeoReference {
    /// Convert projected coordinates to the local frame.
    ///
    pub fn to_local(&self, projected: DVec2) -> DVec2 {
        projected - self.anchor
    }

    /// Convert local-frame metres to projected coordinates.
    pub fn to_projected(&self, local: DVec2) -> DVec2 {
        self.anchor + local
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const UTM: GeoReference = GeoReference {
        epsg: Some(32617),
        anchor: DVec2::new(436_845.7, 4_572_650.3),
    };

    #[test]
    fn local_round_trip_is_sub_millimetre_within_5_km() {
        let offsets = [
            DVec2::new(0.0, 0.0),
            DVec2::new(4999.9, -4999.9),
            DVec2::new(-4321.123, 1234.567),
            DVec2::new(0.013, 4999.99),
        ];
        for local in offsets {
            let back = UTM.to_local(UTM.to_projected(local));
            assert!(
                (back - local).abs().max_element() < 1e-3,
                "{local} -> {back}"
            );
        }
    }

    #[test]
    fn projected_round_trip_is_sub_millimetre_within_5_km() {
        let projected = UTM.anchor + DVec2::new(4321.9876, -4999.1234);
        let back = UTM.to_projected(UTM.to_local(projected));
        assert!((back - projected).abs().max_element() < 1e-3);
    }
}
