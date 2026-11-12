# costmap

[![Crates.io](https://img.shields.io/crates/v/costmap.svg)](https://crates.io/crates/costmap)
[![Documentation](https://docs.rs/costmap/badge.svg)](https://docs.rs/costmap)
[![License](https://img.shields.io/crates/l/costmap.svg)](https://github.com/MJohnson459/costmap#license)

A Rust library for 2D costmaps, occupancy grids, and raycasting — aimed at robotics navigation.

The goal is to provide a standalone, composable alternative to [costmap_2d](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d) from Nav2, implemented as an independent Rust crate with no ROS dependency. The Rust robotics ecosystem needs more self-contained libraries that can be mixed and matched outside of any single framework.

> **AI Disclaimer:** I have been using this project as a way to test different workflows with AI generation. The majority of the code has been written by AI using a mix of Cursor Composer/Auto and Claude Opus 4.6. In all cases I had them generate a plan before implementation began. After generation, I have reviewed all code, however there are still many "odd decisions" that have embedded themselves in the code. Contributions in code or experience are welcome!

> **Status:** Work in progress. The core grid, raycasting, inflation, obstacle layer, A* planner, tiled storage, GeoTIFF loading and the Nav2 bridge work and are tested.

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
- **Obstacle layer** — `ObstacleLayer` marks sensor hits, clears along beams, and forgets obstacles after a set time, including when the sensor stream stops.
- **Obstacle inflation** — expand obstacles with Nav2-style inscribed exponential decay for safe navigation.
- **Planning** — 8-connected, cost-weighted A* over a costmap, with no corner cutting.
- **Line and polygon iterators** — iterate cells along a ray or inside a convex polygon, with read/write access.
- **2D occupancy grids & costmaps** — `DenseGrid<i8>` and `DenseGrid<u8>` with configurable resolution, origin, and coordinate transforms.
- **Nav2-compatible semantics** — lethal / inscribed / free / unknown cost values matching costmap_2d conventions.
- **ROS2 map loading** — load occupancy grids from the standard YAML + image format.
- **Rerun visualisation** (optional) — log grids and costmaps as native `GridMap` archetypes with the built-in RViz colormaps.
- **Tiled maps** — `TiledGrid<T>` stores a map as sparse fixed-size chunks with signed cell coordinates. `window()` returns a dense `DenseGrid<T>`, so every algorithm in the crate runs on a piece of a map of any size. `TiledMapLayer` stamps the part under a rolling costmap.
- **GeoTIFF maps** (feature `geotiff`) — cost maps are single-band `u8` GeoTIFFs that QGIS and GDAL open. `CostGeoTiff::load_region` reads only the chunks a region needs. `GeoReference` keeps projected coordinates in `f64` and per-cell maths in `f32`.

## Workspace

| Crate | What it is |
| --- | --- |
| `costmap` (repository root) | The library. No ROS dependency. |
| [`crates/costmap-geo`](crates/costmap-geo) | `costmap-from-ortho` builds a cost GeoTIFF from an OpenDroneMap DSM, DTM and orthophoto. |
| [`crates/costmap_ros`](crates/costmap_ros) | `costmap_window_server` publishes a rolling window of a map to Nav2's `StaticLayer`, and `costmap_planner_server` plans Nav2's global paths across the whole map, over [Hiroz](https://github.com/ZettaScaleLabs/hiroz) (Zenoh). It builds with plain `cargo` and no ROS install. |

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
costmap = "0.2.0"

# Optional features
costmap = { version = "0.2.0", features = ["rerun", "geotiff"] }
```

Upgrading from 0.2? See the [migration guide](MIGRATION.md).

## Quick start

Load a map and perform raycasting for obstacle detection:

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

## Farm-scale maps

A map of a field is larger than a robot's costmap and is georeferenced. The pieces above cover that case end to end:

1. **Survey.** Process a drone survey with OpenDroneMap with `--dsm --dtm`. It writes an orthophoto, a surface model and a terrain model.
2. **Cost map.** `costmap-from-ortho` turns those three rasters into a single-band `u8` cost GeoTIFF: obstacles from height above ground, graded cost from slope. See [`crates/costmap-geo`](crates/costmap-geo) for the rules and the keep-out polygon overrides.
3. **Inspect and edit.** Open the cost GeoTIFF in QGIS. Value 0 is free, 254 lethal, 255 unknown.
4. **Navigate.** Load it with `CostGeoTiff` into a `TiledGrid`, as the farm demo below does, or serve it to Nav2 with `costmap_window_server` and `costmap_planner_server`. See [`crates/costmap_ros`](crates/costmap_ros).

```bash
cargo run --release -p costmap-geo --bin costmap-from-ortho -- \
    --dsm odm_dem/dsm.tif --dtm odm_dem/dtm.tif \
    --ortho odm_orthophoto/odm_orthophoto.tif --out cost.tif
```

### Farm demo

[`examples/farm_demo`](examples/farm_demo) drives a simulated robot across a cost GeoTIFF without ever holding the whole map:

- The route is planned on a 0.4 m overview of the whole map. The overview is read from one of the file's overview images if the file records it as max-pooled (`RESAMPLING=MAX`, which `write_cost_geotiff` writes). Otherwise it is max-pooled while streaming the raster, and the program prints why. `gdaladdo` and the COG driver write averaged or unrecorded overviews, which can drop fences. For maps with cells coarser than 0.25 m the overview is the full map.
- Tile chunks are loaded from the file near the robot and evicted beyond a larger radius. The full-resolution raster is held only as these chunks.
- A 30 m rolling costmap combines the paged map, simulated lidar and inflation. Lidar beams stop at the map's no-data cells, so unknown ground stays unknown and the robot does not enter it.
- Two obstacles that the map does not contain are placed on the route, and the local planner goes round them. If the robot can reach no point of the route, it marks the obstacles it has seen on the overview and replans the route. It stops, blocked, with the reason, if that fails.
- Safety is measured against ground truth: the robot's 0.4 m footprint, every half cell along its path, against the map's lethal cells and the surprise obstacles. The summary reports the steps with an overlap and the minimum clearance.
- The program prints a summary and exits non-zero unless the goal is reached with no safety violations.

Run it on a generated 400 m × 300 m farm. No data is needed: the farm is written to a temporary GeoTIFF and read back through the same path as a real map.

```bash
cargo run --release --example farm_demo --features rerun,geotiff
```

Run it on your own cost GeoTIFF. Start and goal are in metres east and north of the map's lower-left corner; without them the demo picks two free cells far apart.

```bash
cargo run --release --example farm_demo --features rerun,geotiff -- \
    --map cost.tif --start 300,80 --goal 20,170
```

Both commands open the [Rerun](https://rerun.io) viewer, which must be installed (`cargo install rerun-cli`). Other outputs:

| Flag | Effect |
| --- | --- |
| `--save out.rrd` | Write a Rerun recording instead of opening the viewer. |
| `--headless` | Log nothing; print the summary only. |
| `--frames DIR` | Also write PNG frames, drawn by the demo itself. |
| `--speed MPS` | Robot speed in m/s, 0.1 to 10. The default is 1.5. |
| `--overview-m M` | Largest cell size of the route-planning overview in metres, 0.05 to 5. The default is 0.5. The route can only pass gaps a few overview cells wide. |

Each flag may be given once.

Record a video without a display:

```bash
cargo run --release --example farm_demo --features rerun,geotiff -- --headless --frames frames
ffmpeg -framerate 30 -i frames/frame_%05d.png -pix_fmt yuv420p farm_demo.mp4
```

### Row crops

The process was run end to end on a vineyard survey ([UAV AgroTwin Vineyard Dataset 2024](https://zenodo.org/records/12168516), 322 nadir images, rows 2.3 m apart). Two settings differ from the defaults:

- ODM did not reconstruct the vine canopy, so the height rule marked almost none of the rows. The colour rule marks them from the orthophoto. See "Thin crops" in [`crates/costmap-geo`](crates/costmap-geo) for the measurements and the limits of that rule.
- At the default 0.5 m overview the lanes between rows close up and no route enters them. `--overview-m 0.12` keeps them open.

```bash
cargo run --release -p costmap-geo --bin costmap-from-ortho -- \
    --dsm odm_dem/dsm.tif --dtm odm_dem/dtm.tif --ortho odm_orthophoto/odm_orthophoto.tif \
    --vegetation-cost 254 --exg-threshold 0.15 --vegetation-window-m 0.21 --out cost.tif

cargo run --release --example farm_demo --features rerun,geotiff -- \
    --map cost.tif --overview-m 0.12 --start 53.5,30 --goal 100.1,80
```

### Nav2

Nav2's `map_server` publishes a whole map as one message, and its planner searches a global costmap that holds all of it. A survey-scale map does not fit: the vineyard above is 36 M cells. Two binaries take their place:

- `costmap_window_server` replaces `map_server`. It publishes only the part of the map around the robot, as `nav_msgs/OccupancyGrid` on `/map`, for the `StaticLayer` of Nav2's local costmap.
- `costmap_planner_server` replaces `planner_server` and the global costmap. It answers Nav2's `compute_path_to_pose` action with a path across the whole map.

Nav2 needs no plugin and keeps its behaviour tree, controller, local costmap and recoveries. A goal anywhere on the map works.

On a robot that already runs Nav2:

1. Build the binaries on a development machine and copy them to the robot. They link only glibc. There is no colcon or ament package and no build-farm release.

   ```bash
   cargo build --release -p costmap_ros --bins
   ```

2. Start them where `map_server` and `planner_server` were launched, and take `planner_server` out of the lifecycle manager's `node_names`. Both connect to the Zenoh router the robot already runs for `rmw_zenoh_cpp`.

   ```bash
   costmap_window_server --map cost.tif --window-m 20
   costmap_planner_server --map cost.tif
   ```

3. In the local costmap set `rolling_window: true` and the map's `resolution`; at another resolution `StaticLayer` drops small obstacles. It must be smaller than `--window-m` by a margin that depends on the robot's speed: see [Choosing the window size](crates/costmap_ros#choosing-the-window-size).
4. Provide `map -> base_link` on `/tf`. The `map` frame of a GeoTIFF is its lower-left corner, x east and y north; the window server logs that corner's projected coordinates at start.

Requirements and limits:

- Every node that talks to the servers runs `rmw_zenoh_cpp` (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`). There is no DDS support.
- The servers are plain binaries. They need no ROS install to build, and cannot be built by colcon or started with `ros2 run`.
- The planner works on a 0.25 m overview of the static map. It closes gaps narrower than that and does not see sensed obstacles.
- It has run against Nav2 1.5.2 on ROS Lyrical with a simulated robot, not on a physical one.

Two scripts run it against a real Nav2 from a [pixi](https://pixi.sh) environment:

```bash
cd crates/costmap_ros/nav2
# Integration test: compare every cell a Nav2 costmap holds with the source map.
bash test/run.sh
# Demo: Nav2 drives a simulated robot to one goal across a cost GeoTIFF.
DRIVE_MAP=cost.tif DRIVE_START=53.5,30 DRIVE_GOAL=100.1,80 bash demo/run.sh
```

In the demo Nav2 drove 97.0 m to a goal 50 m outside the 20 m window it held, on paths planned in 6 to 10 ms. [`crates/costmap_ros`](crates/costmap_ros) has the servers' options, the Nav2 parameters and the results.

## Roadmap

### Coordinate frame transforms (tf)

Costmaps are inherently multi-frame: sensor data arrives in sensor frames, the
local costmap rolls in an `odom`/`base_link` frame, and the global costmap lives
in `map`. Fusing sensor readings, scrolling the rolling window (`update_origin`),
and checking a footprint at a given pose all require relating frames over time —
the job ROS2's `tf2` does for `costmap_2d`. Today this crate has no notion of
named frames or time-stamped poses; callers must convert coordinates themselves.

The plan is **not** to build a transform library. The
[`transforms`](https://github.com/deniz-hofmeister/transforms) crate is an
actively-maintained, pure-Rust, middleware-free tf2 equivalent (frame tree,
time-buffered history, SLERP/lerp interpolation, static transforms). We intend
to depend on it and add a thin adapter at the costmap boundary. The main
integration cost is type conversion: `transforms` uses its own
`Vector3`/`Quaternion` while this crate is glam-based, so the adapter bridges the
two and exposes frame-aware costmap queries.

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
