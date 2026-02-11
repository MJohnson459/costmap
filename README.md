# costmap

[![Crates.io](https://img.shields.io/crates/v/costmap.svg)](https://crates.io/crates/costmap)
[![Documentation](https://docs.rs/costmap/badge.svg)](https://docs.rs/costmap)
[![License](https://img.shields.io/crates/l/costmap.svg)](https://github.com/MJohnson459/costmap#license)

A Rust library for 2D costmaps, occupancy grids, and raycasting — aimed at robotics navigation.

The goal is to provide a standalone, composable alternative to [costmap_2d](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d) from Nav2, implemented as an independent Rust crate with no ROS dependency. The Rust robotics ecosystem needs more self-contained libraries that can be mixed and matched outside of any single framework.

> **Status:** Work in progress. The core grid, raycasting, coordinate transforms, inflation, and ROS2 map loading are functional. The Nav2 compatibility layer and advanced costmap features are partially stubbed.

## Goals

- **Easy to use** — clear API with sensible defaults; load a map and start querying in a few lines.
- **Performant** — match or exceed the performance of Nav2's C++ costmap_2d implementation.
- **Composable** — no framework lock-in. Use the grid on its own, plug in your own layers, or integrate with any robotics stack.

## Who is this for?

This library is designed for:

- **Rust robotics developers** building mobile robots, autonomous vehicles, or drone navigation systems
- **ROS2 users** wanting to prototype or deploy navigation algorithms in pure Rust
- **Game developers** needing efficient 2D spatial queries and line-of-sight calculations
- **Researchers** experimenting with costmap algorithms without the overhead of a full robotics framework

## Features

- **Rolling window costmaps** — `update_origin` shifts the grid while preserving overlapping data, enabling efficient robot-centered local costmaps.
- **Raycasting** — DDA and grid-step algorithms for sensor simulation and collision detection in world coordinates.
- **Obstacle inflation** — expand obstacles with Nav2-style inscribed exponential decay for safe navigation.
- **Line and polygon iterators** — iterate cells along a ray or inside a convex polygon, with read/write access.
- **2D occupancy grids & costmaps** — `Grid2d<i8>` and `Grid2d<u8>` with configurable resolution, origin, and coordinate transforms.
- **Nav2-compatible semantics** — lethal / inscribed / free / unknown cost values matching costmap_2d conventions.
- **ROS2 map loading** — load occupancy grids from the standard YAML + image format.
- **Rerun visualisation** (optional) — log grids and costmaps as textured 3D planes with the RViz colour palette.
- **Nav2 compatibility layer** (optional) — a `Costmap2D`-like API surface for incremental migration.

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
costmap = "0.1.0"

# Optional features
costmap = { version = "0.1.0", features = ["rerun"] }
```

## Quick start

Load a map and perform raycasting for obstacle detection:

```rust
use costmap::RosMapLoader;
use glam::Vec2;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Load a ROS2 map (YAML + image format)
    let grid = RosMapLoader::load_from_yaml("map.yaml")?;

    // Raycast from origin in a given direction
    let origin = Vec2::new(10.0, 10.0);  // meters, world coordinates
    let direction = Vec2::new(1.0, 0.0); // unit vector
    let max_range = 30.0;

    // Check for obstacles along the ray
    if let Some(hit) = grid.raycast_dda(&origin, &direction, max_range) {
        println!("Hit obstacle at distance: {:.2}m", hit.hit_distance);
        println!("Cell coordinates: {:?}", hit.cell);
    } else {
        println!("No obstacles within range");
    }

    Ok(())
}
```

## Examples

### [footprint_checking](examples/footprint_checking.rs)

Footprint-based collision checking: a footprint moves through a funnel-shaped corridor. Its color matches the costmap (white when free, gradient when elevated, cyan when lethal). Demonstrates `footprint_cost()` for pose validation.

```bash
cargo run --example footprint_checking --features rerun
```

### [occupancy_raycast](examples/occupancy_raycast.rs)

Demonstrates the core raycasting API with an animated 360° sweep. Shows how to:

- Load ROS2 map files
- Perform DDA raycasting for obstacle detection
- Handle hit/miss cases

![Raycast example](docs/images/rerun_raycast.png)

### [local_costmap_lidar](examples/local_costmap_lidar.rs)

Complete local costmap workflow for mobile robot navigation. Shows how to:

- Create and update rolling window costmaps
- Integrate simulated lidar data using `clear_ray()`
- Apply inflation layers for safe path planning
- Move the costmap origin to follow the robot

![Costmap example](docs/images/rerun_costmap.png)

## Roadmap

### Core

- [x] 2D grid storage with bounds-checked access
- [x] World / map coordinate transforms
- [x] Row-major cell iterators (immutable and mutable)
- [x] Origin updates with data preservation (rolling window)
- [x] Configurable fill value for new / reset cells
- [x] Map resize and region reset
- [ ] Sparse active-cell iterators
- [ ] Region queries (rect, circle)
- [ ] Resolution resampling

### Raycasting

- [x] Grid-step raycast (2D)
- [x] DDA raycast (2D)
- [x] Line iterators (position, value, mutable value)
- [x] `clear_ray` — clear along a ray with optional endpoint marking
- [ ] 3D raycast variants
- [ ] Configurable hit thresholds

### Costmap

- [x] Costmap constants (free / inscribed / lethal / unknown)
- [x] Inflation with Nav2-style inscribed exponential decay
- [x] Unknown-cell preservation through inflation
- [x] Polygon iterators and footprint cost queries
- [ ] Full Nav2 layer lifecycle (initialize, reset, update bounds/costs)
- [ ] Obstacle clearing by footprint
- [ ] Cost queries at pose

### Layers

- [x] Layered grid wrapper with metadata
- [x] Aggregation policy trait
- [ ] Per-layer get/set by index and name
- [ ] Layer masks and filtering

### IO

- [x] ROS2 map loader (YAML + image)
- [ ] ROS2 map writer
- [ ] Other formats (PGM/PNG with metadata sidecar)

### Visualisation

- [x] Greyscale image export for occupancy grids
- [x] Rerun: log occupancy grids and costmaps as textured planes
- [x] Rerun: RViz-compatible costmap colour palette
- [ ] Pixel-buffer viewer (optional `viewer` feature)

## Contributing

Contributions are welcome! Please feel free to:

- Open issues for bugs, feature requests, or questions
- Submit pull requests (please open an issue first for major changes)
- Improve documentation and examples
- Share your use cases and feedback

When contributing code, please ensure it follows the existing style and includes appropriate tests.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
