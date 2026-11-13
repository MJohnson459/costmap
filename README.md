# costmap

[![Crates.io](https://img.shields.io/crates/v/costmap.svg)](https://crates.io/crates/costmap)
[![Documentation](https://docs.rs/costmap/badge.svg)](https://docs.rs/costmap)
[![License](https://img.shields.io/crates/l/costmap.svg)](https://github.com/MJohnson459/costmap#license)

2D costmaps for robot navigation in Rust: grids, raycasting, obstacle and inflation layers, planning, and maps too large to hold in memory. It comes with a ROS 2 node that serves such maps to Nav2.

The library is an independent alternative to Nav2's [costmap_2d](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d) with no ROS dependency. The Rust robotics ecosystem needs more self-contained libraries that can be mixed and matched outside of any single framework.

> **AI Disclaimer:** I have been using this project as a way to test different workflows with AI generation. The majority of the code has been written by AI using a mix of Cursor Composer/Auto and Claude Opus 4.6. In all cases I had them generate a plan before implementation began. After generation, I have reviewed all code, however there are still many "odd decisions" that have embedded themselves in the code. Contributions in code or experience are welcome!

> **Status:** Work in progress. Everything listed below works and is tested. The Nav2 node has run with a simulated robot, not a physical one.

## What is here

| Crate | Use it to | Start at |
| --- | --- | --- |
| `costmap` (repository root) | build costmaps and plan on them in a Rust program | [Using the library](#using-the-library) |
| [`costmap_ros`](crates/costmap_ros) | give Nav2 a map too large for `map_server`, with no change to Nav2 | [its README](crates/costmap_ros) |
| [`costmap-geo`](crates/costmap-geo) | turn a drone survey or a ROS map into a cost GeoTIFF | [its README](crates/costmap-geo) |

## With Nav2

`costmap_ros` is a ROS 2 package. Its node, `costmap_window_server`, takes the place of `nav2_map_server`: it publishes only the part of the map around the robot, and plans routes across the whole map, of which Nav2's own planner plans the next 15 m. Nav2 runs unmodified, on `rmw_zenoh_cpp`.

```bash
colcon build --packages-select costmap_ros
ros2 launch costmap_ros costmap.launch.py map:=/maps/site.yaml
```

Measured against `nav2_map_server` with a global costmap of the whole map, on a 36 M-cell survey:

| | Nav2 alone | With this node |
| --- | --- | --- |
| Resident memory | 762 MB | 157 MB |
| Plan of a 99 m trip, median | 311 ms | 5.2 ms |
| CPU at rest, share of one core | 2.65 % | 3.05 %, or 2.05 % with a pose topic |

[`crates/costmap_ros`](crates/costmap_ros) has the install routes (Debian package, colcon, conda), the five steps to replace `map_server`, and the limits.

## Using the library

```toml
[dependencies]
costmap = "0.3"

# Optional features
costmap = { version = "0.3", features = ["rerun", "geotiff"] }
```

Upgrading from 0.2? See the [migration guide](MIGRATION.md) and the [changelog](CHANGELOG.md).

Load a map and cast a ray:

```rust
use costmap::RosMapLoader;
use glam::{DVec2, Vec2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Load a ROS2 map (YAML + image format)
    let grid = RosMapLoader::load_from_yaml("map.yaml")?;

    // Raycast from origin in a given direction
    let origin = DVec2::new(10.0, 10.0);  // meters, world coordinates
    let direction = Vec2::new(1.0, 0.0); // unit vector
    let max_range = 30.0;

    // Check for obstacles along the ray
    if let Some(hit) = grid.raycast_dda(origin, direction, max_range) {
        println!("Hit obstacle at distance: {:.2}m", hit.hit_distance);
        println!("Cell coordinates: {:?}", hit.cell);
    } else {
        println!("No obstacles within range");
    }

    Ok(())
}
```

### Features

- **Occupancy grids and costmaps** — `DenseGrid<i8>` and `DenseGrid<u8>` with resolution, origin and coordinate transforms. Cost values follow costmap_2d: free, graded, inscribed, lethal, unknown.
- **Rolling windows** — `update_origin` shifts a grid and keeps the overlapping cells, for a costmap that follows the robot.
- **Raycasting** — DDA and grid-step algorithms in world coordinates.
- **Obstacle layer** — `ObstacleLayer` marks sensor hits, clears along beams, and forgets obstacles after a set time, including when the sensor stream stops.
- **Inflation** — Nav2-style inscribed radius and exponential decay.
- **Planning** — `plan` is an 8-connected, cost-weighted A* with no corner cutting. `TileGraph` plans across a map a tile at a time: 28 ms where `plan` takes 2,270 ms on a 30.5 M-cell map, at most 0.9 % costlier, reading only the tiles on the route.
- **Line and polygon iterators** — the cells along a ray or inside a convex polygon, with read and write access.
- **ROS 2 map loading** — the standard YAML and image format.
- **Tiled maps** — `TiledGrid<T>` stores a map as sparse fixed-size chunks with signed cell coordinates. `window()` returns a `DenseGrid<T>`, so every algorithm in the crate runs on a piece of a map of any size.
- **GeoTIFF maps** (feature `geotiff`) — cost maps are single-band `u8` GeoTIFFs that QGIS and GDAL open. `CostGeoTiff::load_region` reads only the chunks a region needs.
- **Rerun visualisation** (feature `rerun`) — grids and costmaps as `GridMap` archetypes with the RViz colormaps.

### Examples

| Example | Shows | Run |
| --- | --- | --- |
| [footprint_checking](examples/footprint_checking.rs) | `footprint_cost()` for a footprint moving through a narrowing corridor | `cargo run --example footprint_checking --features rerun` |
| [occupancy_raycast](examples/occupancy_raycast.rs) | loading a ROS map and a 360° DDA raycast sweep | `cargo run --example occupancy_raycast --features rerun` |
| [local_costmap_lidar](examples/local_costmap_lidar.rs) | a rolling local costmap fed by simulated lidar, with inflation | `cargo run --example local_costmap_lidar --features rerun` |
| [learned_traversability](examples/learned_traversability.rs) | a costmap layer from a traversability estimate | `cargo run --example learned_traversability --features rerun` |
| [farm_demo](examples/farm_demo) | a simulated robot crossing a cost GeoTIFF it never holds whole | `cargo run --release --example farm_demo --features rerun,geotiff` |

The examples open the [Rerun](https://rerun.io) viewer, which must be installed (`cargo install rerun-cli`).

![Raycast example](docs/images/rerun_raycast.png)

![Costmap example](docs/images/rerun_costmap.png)

## Large maps

A map of a field is larger than a robot's costmap and is georeferenced. The crates cover that case end to end:

1. **Survey.** Process a drone survey with OpenDroneMap with `--dsm --dtm`. It writes an orthophoto, a surface model and a terrain model.
2. **Cost map.** `costmap-from-ortho` turns those three rasters into a single-band `u8` cost GeoTIFF: obstacles from height above ground, graded cost from slope. See [`crates/costmap-geo`](crates/costmap-geo) for the rules, row crops and the keep-out polygon overrides.
3. **Inspect and edit.** Open the cost GeoTIFF in QGIS. Value 0 is free, 254 lethal, 255 unknown.
4. **Navigate.** Serve it to Nav2 with [`costmap_ros`](crates/costmap_ros), or load it with `CostGeoTiff` into a `TiledGrid` in a Rust program, as the [farm demo](examples/farm_demo) does.

```bash
cargo run --release -p costmap-geo --bin costmap-from-ortho -- \
    --dsm odm_dem/dsm.tif --dtm odm_dem/dtm.tif \
    --ortho odm_orthophoto/odm_orthophoto.tif --out cost.tif
```

A ROS map too large for `map_server` converts with `costmap-from-ros-map site.yaml site.tif`.

## Not here yet

- **Frames and time in the library.** The library takes coordinates in one frame; callers convert. `costmap_ros` reads `/tf` for the robot pose only. The [`transforms`](https://github.com/deniz-hofmeister/transforms) crate is the candidate for a proper frame tree and has not been evaluated.
- **3D.** Grids are 2D.
- **A release on the ROS build farm.** See [`crates/costmap_ros`](crates/costmap_ros#not-on-the-ros-build-farm).

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
