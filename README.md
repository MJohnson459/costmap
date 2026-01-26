# Voxel Grid

This is an implementation of a Voxel Grid and a collection of ray casting
algorithms for use with it.

Primarily aimed at use within Robotics.

## Quick start

Load a ROS2 occupancy grid (YAML + image) and render a grayscale preview:

```rust
use voxel_grid::{load_occupancy_grid, visualization::occupancy_grid_to_image};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let grid = load_occupancy_grid("map.yaml")?;
    let image = occupancy_grid_to_image(&grid);
    image.save("map_preview.png")?;
    Ok(())
}
```

## TODO (Operations)

### Core
- [x] Core grid storage for 2D occupancy
- [x] Bounds-checked get/set helpers
- [ ] Grid iterators: row-major cells, column iter, layer iter
- [ ] Grid iterators: sparse active cells (occupied/known)
- [ ] Region queries: rect/box scans by bounds
- [ ] Region queries: circle/sphere scans by radius
- [x] World <-> grid coordinate transforms
- [ ] Origin updates (translate/rotate)
- [ ] Resolution changes (resample)

### Layers
- [x] Layered grid wrapper + metadata
- [x] Aggregation policy trait + default policy
- [ ] Per-layer updates: set/get by layer index
- [ ] Per-layer updates: set/get by layer name
- [ ] Layer masks: include/exclude subsets
- [ ] Layer filtering: apply transforms to selected layers
- [ ] Occupancy semantics: thresholds and conversions
- [ ] Occupancy semantics: unknown handling policies

### Raycasting
- [x] Grid-step raycast (2D)
- [x] DDA raycast (2D)
- [ ] 3D raycast variants
- [ ] Configurable hit rules (thresholds)

### Grid map (grid_map-style)
- [ ] Layered numeric fields (elevation, variance, semantics)
- [ ] Interpolation: bilinear/nearest sampling
- [ ] Gradient/slope computation from elevation
- [ ] Layer math: add/subtract, normalize, clamp
- [ ] Spatial filters: smoothing, dilation/erosion
- [ ] Region ops: submap extraction and reproject
- [ ] Map fusion: merge/overwrite layers by policy

### Costmap
- [ ] Layer lifecycle: initialize, reset, clear by bounds
- [ ] Obstacle marking: add points, add rays (clear free space)
- [ ] Obstacle clearing: clear by raytrace and by footprint
- [ ] Inflation: compute inflated costs with radius + scaling
- [ ] Footprint checks: collision and inscribed radius tests
- [ ] Rolling window: shift origin, recenter, and clear stale
- [ ] Cost queries: cost at cell, cost at pose, cost bounds

### IO
- [x] ROS2 map loader (YAML + image)
- [ ] ROS2 map writer
- [ ] Other formats (e.g., PGM/PNG with metadata)
