# costmap_ros

`costmap_ros` publishes windows of a `costmap` grid to ROS 2 as
`nav_msgs/OccupancyGrid`. Nav2's stock `nav2_costmap_2d::StaticLayer`
subscribes to that topic, so Nav2 needs no plugin and no C++ from this crate.

The crate uses [Hiroz](https://github.com/ZettaScaleLabs/hiroz) 0.2.1, a ROS 2
client written in Rust on Zenoh. It builds with cargo alone and needs no ROS
install. It talks to ROS 2 nodes that run `rmw_zenoh_cpp`.

The intended use is a map far larger than Nav2 can hold. The server publishes
only a square window of the map around the robot. Nav2 runs its costmap with
`rolling_window: true` and the StaticLayer reads every new window.

## Contents

- Library:
  - `convert`: cost ↔ occupancy mapping, and `Costmap` ↔ `nav_msgs/OccupancyGrid`.
  - `source`: the `MapSource` trait. A dense in-memory `Costmap` and
    `GeoTiffSource`, which reads a cost GeoTIFF chunk by chunk, implement it.
  - `republish`: when the server sends a new window.
  - `tf`: a transform tree fed from `/tf` and `/tf_static`, and a resolver for
    the robot pose.
- Binary: `costmap_window_server`.
- Examples:
  - `yaml_to_cost_geotiff` converts a ROS map YAML to a cost GeoTIFF.
  - `route_waypoints` plans a route across a whole cost GeoTIFF with `costmap`'s
    A* and prints waypoints along it.
- `nav2_e2e/`:
  - `run_e2e.sh`: an end-to-end test against a real Nav2 costmap node.
  - `run_drive.sh`: Nav2 plans and drives a simulated robot across a cost GeoTIFF.

## Running the server

A Zenoh router must be running, for example `ros2 run rmw_zenoh_cpp rmw_zenohd`
from a ROS 2 install that has `rmw_zenoh_cpp`. Then, from the repository root:

```sh
cargo run --release -p costmap_ros --bin costmap_window_server -- --map tests/fixtures/warehouse.yaml
```

`--map` selects the map source by file extension. Any other extension is a
startup error.

| Extension | Source |
| --- | --- |
| `.yaml`, `.yml` | ROS map, loaded whole into memory |
| `.tif`, `.tiff` | cost GeoTIFF, loaded chunk by chunk near the robot |

For a ROS map the server loads the YAML and its image with
`costmap::RosMapLoader`. Occupied cells become cost 254, free cells 0 and
unknown cells 255. The robot
pose is the latest `map -> base_link` transform. The server checks the pose
`--rate-hz` times a second and publishes when one of these holds:

- no window has been published yet;
- the robot is `--min-move-m` or more from the centre of the last window;
- the last window is `--keep-alive-s` old.

| Option | Default | Meaning |
| --- | --- | --- |
| `--map` | required | ROS map YAML or cost GeoTIFF |
| `--window-m` | 20 | side of the square window, metres |
| `--topic` | `map` | output topic |
| `--map-frame` | `map` | frame of the map and of the published grid |
| `--base-frame` | `base_link` | robot frame |
| `--rate-hz` | 2 | pose checks per second |
| `--min-move-m` | 1 | movement that triggers a new window, metres |
| `--keep-alive-s` | 10 | age that triggers a new window, seconds |

Between windows the robot can move `--min-move-m` plus the distance it covers
in one `--rate-hz` period. Nav2's window must stay inside the published window
over that distance, so `(window_m - nav2_window) / 2` must exceed it. With the
defaults and a 16 m Nav2 window the margin is 2 m: 1 m for `--min-move-m` and
1 m of travel in one 0.5 s period, which allows 2 m/s.

The window origin is a whole number of cells from the map origin. A cell keeps
its world position and value in every window, so Nav2 never sees a cell
resampled.

## GeoTIFF maps

A GeoTIFF map is a single-band unsigned 8-bit raster in a projected CRS whose
pixel values are costs (255 is unknown), as written by
`costmap::write_cost_geotiff`. The server holds only the 64 x 64-cell chunks
near the robot, so a 20,000 x 20,000-cell map is never in memory whole.

- A window that lies inside the region loaded last time reads nothing.
- Any other window loads the chunks under the window plus 5 m on every side.
  The chunks are decoded from the TIFF strips or tiles that cover them.
- Before loading, the server drops every chunk that lies more than 25 m beyond
  that loaded region. A robot that moves back and forth across the edge of the
  loaded region therefore does not reload the same chunks.
- Chunks that are entirely unknown are not allocated, and the file is read
  again for them when a window next leaves the loaded region.

At debug level each publish logs the chunks loaded, evicted and held since
start. At info level the server logs the same totals on shutdown.

The map frame is the GeoTIFF's local frame: x east and y north in metres from
the raster's lower-left corner. The published window's `info.origin` is in this
frame, and the robot pose from `map -> base_link` must be in it. At start the
server logs the file's EPSG code and the projected coordinates of that corner.
Projected coordinates of a point are the logged corner plus its `map`
coordinates. The server does no reprojection and no datum handling.

The `yaml_to_cost_geotiff` example converts a ROS map to a cost GeoTIFF whose
lower-left corner is the YAML origin:

```sh
cargo run --release -p costmap_ros --example yaml_to_cost_geotiff -- \
    tests/fixtures/warehouse.yaml warehouse.tif
```

The converted map shares the YAML map's frame only when the YAML origin is
(0, 0), as in the fixture. Otherwise `map` coordinates are shifted by the
YAML origin.

The topic uses QoS reliable, transient local, depth 1, which is what
StaticLayer requests with `map_subscribe_transient_local: true`. A Nav2 node
that starts after the server receives the current window.

The server logs one line at info level on start (two for a GeoTIFF) and one
line per publish at debug level. `RUST_LOG=costmap_window_server=debug` shows the publishes.
Ctrl-C or SIGTERM stops it.

The Zenoh session follows `rmw_zenoh_cpp`'s conventions: it connects to a
router at `tcp/localhost:7447`, and reads `ZENOH_SESSION_CONFIG_URI`,
`ZENOH_CONFIG_OVERRIDE` and `ROS_DOMAIN_ID`.

## Nav2 configuration

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      rolling_window: true
      width: 10            # smaller than the server's --window-m
      height: 10
      resolution: 0.05     # the map's resolution
      track_unknown_space: true
      trinary_costmap: false
      lethal_cost_threshold: 100
      plugins: ["static_layer"]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_topic: /map
        map_subscribe_transient_local: true
```

`track_unknown_space`, `trinary_costmap` and `lethal_cost_threshold` are
parameters of the costmap node, not of the layer; StaticLayer reads them from
the node. `inscribed_obstacle_cost_value` (99) and `unknown_cost_value` (255)
must stay at their defaults. Use the map's resolution and leave `origin_x` and
`origin_y` on the map's cell grid, so Nav2's cells coincide with the map's.

The resolution matters for safety. StaticLayer point-samples a map whose
resolution differs from the costmap's. A Nav2 costmap at 0.05 m over a 0.03 m
map dropped a 9 cm obstacle, and Nav2 planned and drove through it.

Nav2 must run with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and reach the same Zenoh
router as the server.

## Cost and occupancy

StaticLayer converts each occupancy value with `StaticLayer::interpretValue`
(navigation2 1.5.2). With the parameters above:

| Cost sent | Occupancy | Cost Nav2 stores |
| --- | --- | --- |
| 0 (free) | 0 | 0 |
| 1..=248 | `ceil(cost * 100 / 254)`, 1..=98 | `floor(occ * 2.54)`: `cost` to `cost + 2` |
| 249..=252 | 99 | 253 |
| 253 (inscribed) | 99 | 253 |
| 254 (lethal) | 100 | 254 |
| 255 (unknown) | -1 | 255 |

An `OccupancyGrid` cell has 101 levels, so costs 1..=252 quantise in steps of
2.54. The occupancy is rounded up: Nav2 never receives a cost lower than the
one sent, and never receives lethal for a cost below 254. `cost_from_occupancy`
is the Nav2 rule itself and `occupancy_from_cost` is its inverse. Unit tests
run every cost through a transcription of the C++.

## End-to-end test

The test needs [pixi](https://pixi.sh) and a Rust toolchain. Run it with bash;
it is not a zsh script.

```sh
cd crates/costmap_ros/nav2_e2e
bash run_e2e.sh
```

The first run installs the ROS Lyrical environment from `pixi.toml` and
`pixi.lock` (Nav2 costmap_2d 1.5.2, rmw_zenoh_cpp 0.10.6, rclpy).

`run_e2e.sh` builds the server and the converter, converts the fixture to a
GeoTIFF, starts `rmw_zenohd`, and then runs one phase per map format (YAML,
GeoTIFF) and published window size (12 m, 100 m). Each phase starts:

1. `check_costmap.py`, which also publishes `map -> base_link` on `/tf`;
2. `costmap_window_server` on `tests/fixtures/warehouse.yaml`, or on its
   GeoTIFF conversion;
3. a standalone Nav2 costmap node with `costmap_params.yaml` (an 8 m rolling
   window with only a StaticLayer), which the script configures and activates.

The checker reads the fixture YAML and image itself, also in the GeoTIFF
phases. It compares every cell
of Nav2's `/costmap_raw` (exact costs) with the fixture cell at the same world
position: lethal must be 254, free 0, unknown 255. It then moves the robot 65 m
five times, each move past the edge of the previous window, and checks again
after each move. Each stage has a timeout. Every process is started in its own
process group and stopped by that group id. If port 7447 is in use, the router
and all clients use the next free port up to 7547.

`SERVER_RATE_HZ` (default 10) sets the server's `--rate-hz`. `NAV2_HZ`
(default 10) sets Nav2's `update_frequency` and `publish_frequency`.

A one-cell shift of the served map fails the check: 1215 of 2225 lethal cells
matched in that run.

Any cost GeoTIFF can be checked instead of the fixture:

```sh
E2E_MAP=/path/cost.tif E2E_POSES="99.6,30 100.1,80 53.5,30" bash run_e2e.sh
```

`E2E_POSES` are `x,y` positions in the map frame (metres from the raster's
lower-left corner). The script runs one phase per window size in
`E2E_WINDOWS_M` (default `12 40`; `100` also works but sends 11 MB at
0.03 m). Nav2's resolution is read from the raster; its window is
`NAV2_WINDOW_M` (default 8, whole metres). The checker reads the raster with
Pillow, derives the cost Nav2 should hold from the table above, and compares
every cell exactly, graded costs included, reporting matched/total for free,
graded, inscribed/lethal and unknown cells. To check that it can fail, serve a
different raster than the checker reads with `E2E_SERVE_MAP=/path/shifted.tif`,
for example the raster shifted one pixel with
`gdal_translate -srcwin 1 0 W-1 H -a_ullr ULX ULY LRX-res LRY in.tif shifted.tif`.

## Drive demo

`run_drive.sh` has Nav2 plan and drive across a cost GeoTIFF that it never
holds whole.

```sh
cd crates/costmap_ros/nav2_e2e
DRIVE_MAP=/path/cost.tif DRIVE_START=53.5,30 DRIVE_GOAL=100.1,80 bash run_drive.sh
pixi run python render_drive.py --map /path/cost.tif \
    --recording /tmp/costmap_ros_drive.XXXXXX/recording.npz --out drive.mp4
```

It starts:

- a Zenoh router;
- `sim_robot.py`, a kinematic differential-drive robot that integrates Nav2's
  `/cmd_vel` and publishes `odom -> base_link` and `/odom`;
- `costmap_window_server` on the GeoTIFF, with 60 m windows republished every 2 m;
- Nav2's planner (NavFn), controller (Regulated Pure Pursuit) and behaviour
  tree navigator, configured by `drive_params.yaml`. The global costmap is a
  50 m rolling window and the local costmap a 6 m one. Each holds a
  StaticLayer on `/map` and an inflation layer, and nothing else;
- `drive_mission.py`, which sends the waypoints as `NavigateToPose` goals.

Nav2 cannot plan to a goal outside its global costmap. `route_waypoints` plans
the whole route on a max-pooled overview of the raster with `costmap`'s A*,
and the mission sends Nav2 one waypoint every 15 m of it.

The mission passes when:

- Nav2 reports success for every waypoint with the robot within 0.5 m of it;
- no lethal cell of the raster, read from the file with Pillow, lies under the
  0.3 m robot disc at any recorded pose.

Result on a 166 x 196 m vineyard survey at 0.03 m (5,547 x 6,525 cells, 36 M):

| | |
| --- | --- |
| Waypoints reached | 7 of 7, each within 0.29 m |
| Distance driven | 96.6 m in 101 s |
| Windows published | 45, of 2,000 x 2,000 cells each |
| Plans | 7, one per waypoint |
| Lethal cells under the robot disc | 0 |
| Least distance from robot centre to a lethal cell | 0.40 m |
| Chunks of the file read | 2,712 of 8,874; 2,235 held at the end |

`render_drive.py` draws the recording as a video: Nav2's global and local
costmaps as Nav2 published them, the outline of the last window served, the
plan and the track, over the dimmed raster.

The demo does not cover:

- sensors. There is no obstacle layer; Nav2 sees only the served map;
- localisation. `map -> odom` is a fixed identity transform;
- dynamics. The robot follows the commanded velocity exactly;
- recovery behaviours. The behaviour tree is the stock
  `navigate_w_replanning_only_if_goal_is_updated.xml`, which plans once per
  goal. With the 1 Hz replanning tree a plan was still running when a waypoint
  was reached, and Nav2 started the next goal on the previous path and
  reported success at once.

## Latency

Latency is the time from the checker publishing a moved tf to the checker
receiving the first `/costmap_raw` that matches the fixture around the new
pose. It covers the server's pose check, building and sending the window,
StaticLayer reading it, and Nav2's next update and publish. Each row is one
run of five moves. Machine: AMD Ryzen 7 8745HS (8 cores, 16 threads),
everything on localhost, server built with `--release`.

| Server `--rate-hz` | Nav2 update/publish | Window | Message | min | median | max |
| --- | --- | --- | --- | --- | --- | --- |
| 10 | 10 Hz | 12 m (240 × 240) | 57.6 kB | 98 ms | 99 ms | 200 ms |
| 10 | 10 Hz | 12 m (240 × 240) | 57.6 kB | 100 ms | 100 ms | 199 ms |
| 10 | 10 Hz | 100 m (2000 × 2000) | 4 MB | 100 ms | 140 ms | 264 ms |
| 10 | 10 Hz | 100 m (2000 × 2000) | 4 MB | 103 ms | 106 ms | 189 ms |
| 50 | 50 Hz | 12 m (240 × 240) | 57.6 kB | 19 ms | 19 ms | 61 ms |
| 50 | 50 Hz | 12 m (240 × 240) | 57.6 kB | 18 ms | 38 ms | 57 ms |
| 50 | 50 Hz | 100 m (2000 × 2000) | 4 MB | 37 ms | 59 ms | 71 ms |
| 50 | 50 Hz | 100 m (2000 × 2000) | 4 MB | 56 ms | 59 ms | 71 ms |

At 10 Hz the latency comes in steps of about 100 ms. The 100 ms periods of
the server's pose check and Nav2's update loop set it, more than the window
size does. At 50 Hz every number falls by a factor of 2 to 5, and the
100 m window costs about 20 to 40 ms more than the 12 m window at the median.

## Limitations

- The robot pose uses the latest transform of each frame, composed with no
  interpolation to a common time.
- `OccupancyGrid` carries 101 levels; costs 1..=252 arrive quantised as in the
  table above.
- Every publish is a full window. `map_msgs/OccupancyGridUpdate` patches a
  fixed map, and the window origin moves, so updates are not used.
- Nav2 must run `rmw_zenoh_cpp`, and a Zenoh router must connect both sides.
- `MapInfo` has no yaw, so maps with a rotated origin are rejected.
- A Nav2 goal must lie inside Nav2's global costmap, so a goal farther away
  than the window needs intermediate waypoints. See the drive demo.
