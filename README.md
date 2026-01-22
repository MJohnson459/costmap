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
