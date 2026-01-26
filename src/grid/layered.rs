use glam::UVec3;

use crate::grid::Grid;
use crate::types::VoxelError;

/// Optional metadata describing a layer's origin or purpose.
#[derive(Debug, Clone, Default)]
pub struct LayerMeta {
    pub name: Option<String>,
    pub source: Option<String>,
}

/// Aggregates per-layer values into a single result.
pub trait AggregatePolicy<Value> {
    fn combine(&self, values: &[Value]) -> Value;
}

/// Aggregation policy that prioritizes OCCUPIED, then FREE, then UNKNOWN.
#[derive(Debug, Clone, Copy)]
pub struct OccupiedDominant;

impl AggregatePolicy<i8> for OccupiedDominant {
    fn combine(&self, values: &[i8]) -> i8 {
        if values.is_empty() {
            return crate::types::UNKNOWN;
        }
        let mut saw_free = false;
        let mut saw_occupied = false;

        for value in values {
            if *value == crate::types::OCCUPIED {
                saw_occupied = true;
            } else if *value == crate::types::FREE {
                saw_free = true;
            }
        }

        if saw_occupied {
            crate::types::OCCUPIED
        } else if saw_free {
            crate::types::FREE
        } else {
            crate::types::UNKNOWN
        }
    }
}

/// A layered grid wrapper that aggregates values across layers.
///
/// # Example
/// ```
/// use glam::{UVec2, UVec3, Vec3};
/// use voxel_grid::grid::{LayeredGrid, OccupiedDominant};
/// use voxel_grid::types::{FREE, OCCUPIED, MapInfo};
/// use voxel_grid::OccupancyGrid;
///
/// let info = MapInfo {
///     width: 2,
///     height: 2,
///     depth: 1,
///     resolution: 1.0,
///     origin: Vec3::new(0.0, 0.0, 0.0),
/// };
/// let layer_a = OccupancyGrid::new(info.clone(), vec![FREE; 4]).unwrap();
/// let mut layer_b = OccupancyGrid::new(info, vec![FREE; 4]).unwrap();
/// layer_b.set(&UVec2::new(1, 1), OCCUPIED).unwrap();
///
/// let layered = LayeredGrid::new(vec![layer_a, layer_b], OccupiedDominant).unwrap();
/// assert_eq!(layered.aggregate_at(&UVec3::new(1, 1, 0)), Some(OCCUPIED));
/// ```
#[derive(Debug, Clone)]
pub struct LayeredGrid<G, P> {
    layers: Vec<G>,
    policy: P,
    meta: Vec<LayerMeta>,
}

impl<G, P> LayeredGrid<G, P> {
    pub fn layers(&self) -> &[G] {
        &self.layers
    }

    pub fn layers_mut(&mut self) -> &mut [G] {
        &mut self.layers
    }

    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    pub fn meta(&self) -> &[LayerMeta] {
        &self.meta
    }
}

impl<G, P> LayeredGrid<G, P>
where
    G: Grid,
    P: AggregatePolicy<G::Cell>,
{
    pub fn new(layers: Vec<G>, policy: P) -> Result<Self, VoxelError> {
        Self::new_with_meta(layers, policy, Vec::new())
    }

    /// Creates a layered grid with explicit layer metadata.
    pub fn new_with_meta(
        layers: Vec<G>,
        policy: P,
        meta: Vec<LayerMeta>,
    ) -> Result<Self, VoxelError> {
        if layers.is_empty() {
            return Err(VoxelError::InvalidMetadata(
                "layered grid requires at least one layer".to_string(),
            ));
        }

        let base_info = layers[0].info().clone();
        for layer in &layers[1..] {
            if layer.info() != &base_info {
                return Err(VoxelError::InvalidMetadata(
                    "layer metadata does not match base layer".to_string(),
                ));
            }
        }

        let meta = if meta.is_empty() {
            vec![LayerMeta::default(); layers.len()]
        } else if meta.len() != layers.len() {
            return Err(VoxelError::InvalidMetadata(
                "layer metadata length does not match layer count".to_string(),
            ));
        } else {
            meta
        };

        Ok(Self {
            layers,
            policy,
            meta,
        })
    }

    pub fn get_layer(&self, layer: usize) -> Option<&G> {
        self.layers.get(layer)
    }

    pub fn get_layer_mut(&mut self, layer: usize) -> Option<&mut G> {
        self.layers.get_mut(layer)
    }

    /// Aggregates values across all layers at a position.
    pub fn aggregate_at(&self, pos: &UVec3) -> Option<G::Cell> {
        let mut values = Vec::with_capacity(self.layers.len());
        for layer in &self.layers {
            values.push(layer.get(pos)?);
        }
        Some(self.policy.combine(&values))
    }
}

#[cfg(test)]
mod tests {
    use glam::{UVec2, Vec3};

    use super::*;
    use crate::grid::Grid2d;
    use crate::types::{FREE, MapInfo, OCCUPIED, UNKNOWN};

    #[test]
    fn layered_grid_alignment_validation() {
        let info = MapInfo {
            width: 2,
            height: 2,
            depth: 1,
            resolution: 1.0,
            origin: Vec3::new(0.0, 0.0, 0.0),
        };
        let mismatched = MapInfo {
            width: 3,
            ..info.clone()
        };

        let layer_a = Grid2d::new(info, vec![FREE; 4]).unwrap();
        let layer_b = Grid2d::new(mismatched, vec![FREE; 6]).unwrap();

        let result = LayeredGrid::new(vec![layer_a, layer_b], OccupiedDominant);
        assert!(result.is_err());
    }

    #[test]
    fn occupied_dominant_aggregation() {
        let info = MapInfo {
            width: 2,
            height: 2,
            depth: 1,
            resolution: 1.0,
            origin: Vec3::new(0.0, 0.0, 0.0),
        };

        let layer_a = Grid2d::new(info.clone(), vec![FREE; 4]).unwrap();
        let mut layer_b = Grid2d::new(info, vec![FREE; 4]).unwrap();

        layer_b.set(&UVec2::new(1, 1), OCCUPIED).unwrap();
        layer_b.set(&UVec2::new(0, 0), UNKNOWN).unwrap();

        let layered = LayeredGrid::new(vec![layer_a, layer_b], OccupiedDominant).unwrap();

        assert_eq!(layered.aggregate_at(&UVec3::new(1, 1, 0)), Some(OCCUPIED));
        assert_eq!(layered.aggregate_at(&UVec3::new(0, 0, 0)), Some(FREE));
        assert_eq!(layered.aggregate_at(&UVec3::new(0, 1, 0)), Some(FREE));
        assert_eq!(layered.aggregate_at(&UVec3::new(0, 1, 0)), Some(FREE));
    }
}
