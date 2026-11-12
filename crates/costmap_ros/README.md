# costmap_ros

`costmap_ros` publishes windows of a `costmap` grid to ROS 2 as
`nav_msgs/OccupancyGrid`. Nav2's stock `nav2_costmap_2d::StaticLayer`
subscribes to that topic, so Nav2 needs no plugin and no C++ from this crate.

The crate uses [Hiroz](https://github.com/ZettaScaleLabs/hiroz) 0.2.1, a ROS 2
client written in Rust on Zenoh. It builds with cargo alone and needs no ROS
install. It talks to ROS 2 nodes that run `rmw_zenoh_cpp`.

## Why

Nav2's `map_server` loads a whole map into memory and publishes it as one
message, and every costmap with a StaticLayer keeps its own copy. A warehouse
fits. A drone survey does not: a 166 x 196 m vineyard at 0.03 m is 36 M cells,
a 36 MB message, and a square kilometre at that resolution is 1.1 G cells.

Two binaries add what Nav2 lacks for such maps:

- `costmap_window_server` replaces `map_server`. It publishes only a square
  window of the map around the robot, on the same topic and with the same
  message type. Nav2 runs its costmaps with `rolling_window: true` and the
  StaticLayer reads every new window.
- `costmap_planner_server` plans a route across the whole map. Nav2's planner
  searches its global costmap, so on a window it cannot reach a goal outside
  it. A behaviour tree asks this server for the route and Nav2's own planner
  for the stretch ahead.

Nothing of Nav2 is given up. Its planner plugins, footprint checks, costmap
filters, sensor layers, controller and recovery behaviours all still act on
the path the robot drives. The route only says which way to go. A goal
anywhere on a cost GeoTIFF of any size then works, such as one written by
[`costmap-from-ortho`](../costmap-geo) from an OpenDroneMap survey.

## On an existing ROS 2 robot

1. Install the two binaries on the robot. See
   [Building and installing](#building-and-installing).
2. Stop launching `map_server`. Start `costmap_window_server --map cost.tif`
   in its place. It publishes `/map`, as `map_server` did.
3. Start `costmap_planner_server --map cost.tif` beside Nav2's own
   `planner_server`, which stays.
4. Point `bt_navigator` at
   [`behavior_trees/navigate_to_pose_on_route.xml`](behavior_trees/navigate_to_pose_on_route.xml)
   with its `default_nav_to_pose_bt_xml` parameter. See
   [The behaviour tree](#the-behaviour-tree).
5. In both costmaps set `rolling_window: true`, the map's `resolution`, and a
   `width` and `height` chosen as in
   [Choosing the window sizes](#choosing-the-window-sizes). See
   [Nav2 configuration](#nav2-configuration).
6. Provide `map -> base_link` on `/tf`, as Nav2 already requires. For a
   GeoTIFF the `map` frame is the raster's lower-left corner, x east and y
   north. A robot localised in projected coordinates (RTK GNSS in UTM, say)
   needs a static transform from that frame to `map`; the window server logs
   the corner's coordinates at start.

Both binaries connect to the Zenoh router the robot already runs for
`rmw_zenoh_cpp`.

What this asks of the robot:

- **Zenoh.** Every node that talks to the server runs `rmw_zenoh_cpp` and
  reaches the same Zenoh router. A robot on a DDS middleware needs a bridge,
  which this crate does not provide and has not been tested with.
- **No ROS package.** The servers are plain binaries, not an ament package.
  They cannot be built by colcon, started with `ros2 run`, or released through the
  ROS build farm. Start them from a launch file with `ExecuteProcess`, or from
  systemd.
- **Static map only.** Both servers read the file. Obstacles seen by sensors
  go in Nav2's own obstacle layers, as usual. Nav2's planner routes around
  them within its global costmap; the route across the map does not change.

It has been run against Nav2 1.5.2 on ROS Lyrical with a simulated robot. It
has not been run on a physical robot.

## Contents

- Library:
  - `convert`: `Costmap` ↔ `nav_msgs/OccupancyGrid`.
  - `source`: the `MapSource` trait. A dense in-memory `Costmap` and
    `GeoTiffSource`, which reads a cost GeoTIFF chunk by chunk, implement it.
  - `republish`: when the server sends a new window.
  - `route`: `RoutePlanner`, which plans across a whole cost GeoTIFF on a
    max-pooled overview of it.
  - `nav2_msgs`: the `ComputePathToPose` action type, generated at build time
    from `msgs/nav2_msgs`. `hiroz-msgs` does not bundle Nav2's interfaces.
  - `tf`: a transform tree fed from `/tf` and `/tf_static`, and a resolver for
    the robot pose. It is a stand-in: Hiroz has no tf2 port. It keeps the
    latest transform per frame and nothing else. See [Limitations](#limitations).
- Binaries: `costmap_window_server` and `costmap_planner_server`.
- `behavior_trees/navigate_to_pose_on_route.xml`: the Nav2 behaviour tree that
  uses the route. See [The behaviour tree](#the-behaviour-tree).
- [`nav2/`](nav2): a test rig and a demo that run the server against a real
  Nav2. Nothing in it is installed on a robot, and CI does not run it.
  - `nav2/test/run.sh`: an integration test. It compares every cell a Nav2
    costmap node holds with the source map.
  - `nav2/demo/run.sh`: a demo. Nav2 plans and drives a simulated robot across
    a cost GeoTIFF, and the script checks the track.

The only parts meant to run on a robot are the two binaries.

## Building and installing

Build on a development machine or in CI, not on the robot:

```sh
cargo build --release -p costmap_ros --bins
strip target/release/costmap_window_server target/release/costmap_planner_server
```

Each is one executable that links only glibc (`libc`, `libm`, `libgcc_s`).
Stripped, each is about 24 MB; unstripped about 480 MB, because the
workspace's release profile keeps debug information. Copy them to a robot of
the same architecture whose glibc is at least as new as the build machine's.
Building for another architecture, such as arm64, has not been tried.

There is no ament or colcon integration: no `package.xml`, no install rules
and no release on the ROS build farm.

## Running the server

The server connects to a Zenoh router. A robot that runs `rmw_zenoh_cpp`
already has one. To try the server on a development machine, start a router
with `ros2 run rmw_zenoh_cpp rmw_zenohd`, then from the repository root:

```sh
cargo run --release -p costmap_ros --bin costmap_window_server -- --map tests/fixtures/warehouse.yaml
```

`--map` selects the map source by file extension. Any other extension is a
startup error.

| Extension | Source |
| --- | --- |
| `.yaml`, `.yml` | ROS map, loaded whole into memory |
| `.tif`, `.tiff` | cost GeoTIFF, loaded chunk by chunk near the robot |

A ROS map is held whole because of its image format. A PNG or PGM cannot be
read in part: the decoder starts at the first pixel. A GeoTIFF is stored in
strips or tiles that are read on their own. The published windows are the same
either way; only the server's memory differs, at one byte per map cell for a
ROS map. [`costmap-from-ros-map`](../costmap-geo#ros-maps) converts a ROS map
that is too large for that.

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

The window origin is a whole number of cells from the map origin. A cell keeps
its world position and value in every window, so Nav2 never sees a cell
resampled.

## Planner server

```sh
costmap_planner_server --map cost.tif
```

At start it reads a max-pooled overview of the whole raster, one strip or tile
at a time, and inflates it for the robot. It then serves an action of type
`nav2_msgs/action/ComputePathToPose` under the name `compute_route_to_pose`,
and publishes each route on `/route`.

- The route starts at the goal's `start` pose when `use_start` is set, and at
  the latest `map -> base_link` transform otherwise. It passes through the
  goal's `viapoints` in order.
- It is planned with `costmap`'s A* on the overview, shortcut into straight
  segments, and returned as poses at most 0.1 m apart that face along it.
- A start or goal on an impassable overview cell moves to the nearest passable
  one within 1 m.
- Poses must be in the map frame. A pose in another frame aborts the goal with
  `TF_ERROR`; the server does not transform goals.
- Failures abort the goal with Nav2's error codes: `TF_ERROR`,
  `START_OCCUPIED`, `GOAL_OCCUPIED`, `NO_VALID_PATH`.
- A goal cancelled while its route is being planned ends as cancelled.
- There is one planner, so `planner_id` is not read.

| Option | Default | Meaning |
| --- | --- | --- |
| `--map` | required | cost GeoTIFF |
| `--action` | `compute_route_to_pose` | action served |
| `--route-topic` | `route` | topic each route is also published on |
| `--map-frame` | `map` | frame of the map and of the routes |
| `--base-frame` | `base_link` | robot frame |
| `--overview-m` | 0.25 | largest overview cell, metres |
| `--robot-radius-m` | 0.3 | robot radius, metres |
| `--inflation-m` | 1.5 | distance over which routes are pushed from obstacles, metres |

On the 36 M-cell vineyard survey the overview is 694 x 816 cells at 0.24 m,
read in 0.1 s, and a 99.9 m route takes up to 10 ms.

What it does not do:

- **Narrow gaps.** The overview keeps the worst cost in each block, so it
  never opens a gap that is not there, but it can close one that is: an
  obstacle that touches a block fills it. A gap must be up to two overview
  cells wider than the robot to stay open. With the defaults (0.3 m radius,
  0.24 m cells) that is 1.08 m, so a 0.9 m doorway can read as closed and the
  goal aborts with `NO_VALID_PATH`. Lower `--overview-m` for such maps, at
  four times the cells per halving. This is the reason the overview is a
  stopgap: planning must find crossings at the map's own resolution.
- **A route the robot can drive as it is.** The route is a line through free
  space for a robot that turns on the spot. Nav2's planner makes the path that
  is driven.
- **ROS maps.** It reads GeoTIFFs only. Convert a ROS map with
  [`costmap-from-ros-map`](../costmap-geo#ros-maps).

### In place of `planner_server`

Started with `--action compute_path_to_pose --route-topic plan`, the server
answers the action Nav2's stock behaviour trees call, and `planner_server` and
the global costmap can be removed. The robot then drives the route itself.
This suits only a robot that turns on the spot, is close to circular, and
needs no costmap filters or sensed obstacles in its global plan. It was run on
the vineyard survey with a 20 m window and the stock 1 Hz replanning tree:
97.0 m in 100 s, goal reached, least distance to a lethal cell 0.36 m.

## The behaviour tree

[`behavior_trees/navigate_to_pose_on_route.xml`](behavior_trees/navigate_to_pose_on_route.xml)
is a `NavigateToPose` tree. Against Nav2's stock replanning tree it changes
one thing: where the stock tree has one `ComputePathToPose` node, this has
four.

```xml
<RateController hz="0.2">
  <ComputePathToPose goal="{goal}" path="{route}" planner_id=""
                     server_name="compute_route_to_pose" .../>
</RateController>
<RateController hz="1.0">
  <Sequence>
    <TruncatePathLocal input_path="{route}" output_path="{route_ahead}"
                       distance_forward="15.0" distance_backward="0.0"/>
    <GetPoseFromPath path="{route_ahead}" index="-1" pose="{route_goal}"/>
    <ComputePathToPose goal="{route_goal}" path="{path}"
                       planner_id="{selected_planner}" .../>
  </Sequence>
</RateController>
```

1. Every 5 s it asks `costmap_planner_server` for the route to the goal.
2. Every second it cuts the route to the 15 m ahead of the robot, takes the
   end of that stretch as a goal, and asks Nav2's planner for a path to it.
3. The controller follows that path, as in the stock tree.

A robot with its own tree adds these nodes in place of its
`ComputePathToPose`; recoveries and the rest of the tree stay as they are.

- `distance_forward` must be less than half the width of the global costmap,
  so the point planned to lies inside it.
- All nodes are ones Nav2 ships. Nothing is compiled.
- The tree was written for Nav2 1.5.2 and run only on it. Node names and ports
  differ between Nav2 releases, and `GetPoseFromPath` is absent from older ones.
- `NavigateThroughPoses` uses a separate tree. None is provided for it.

## Choosing the window sizes

Four sizes are involved: the window server's `--window-m` (`W`), the widths of
Nav2's global (`G`) and local costmaps, and the tree's `distance_forward`
(`D`). Both costmaps read the same `/map`.

1. **Local costmap.** Size it for the controller, as on any Nav2 robot: 3 to
   6 m.
2. **Planning distance `D`.** How far ahead Nav2's planner looks along the
   route. It sets how wide a detour around a sensed obstacle can be. The demo
   uses 15 m.
3. **Global costmap `G`.** It is centred on the robot and must contain the
   point `D` along the route, so `G / 2 > D`. The demo uses 40 m.
4. **Server window `W`.** The global costmap must stay inside the last window
   until the next one arrives, so the margin `(W - G) / 2` must exceed the
   distance the robot covers in that time:

   `margin > min_move_m + v_max * (1 / rate_hz + 0.2)`

   The 0.2 s is the longest time the integration test measured from a robot
   move to Nav2 holding the new window. Outside the window Nav2 sees unknown
   cells.

The demo uses `G = 40` and `W = 50`, a 5 m margin, with `--min-move-m 2`,
`--rate-hz 5` and 1 m/s: 2 + 1 x 0.4 = 2.4 m.

None of these grows with the map. A larger `W` costs memory and bandwidth:
each window is one message of `(W / resolution)^2` bytes, and each StaticLayer
keeps a copy.

| `W` | at 0.05 m | at 0.03 m |
| --- | --- | --- |
| 20 m | 0.16 MB | 0.44 MB |
| 50 m | 1.0 MB | 2.8 MB |
| 100 m | 4.0 MB | 11 MB |

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

`costmap-from-ros-map`, in `costmap-geo`, converts a ROS map to a cost GeoTIFF whose
lower-left corner is the YAML origin:

```sh
cargo run --release -p costmap-geo --bin costmap-from-ros-map -- \
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
global_costmap:              # and the same in local_costmap, at its size
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      rolling_window: true
      width: 40            # see "Choosing the window sizes"
      height: 40
      resolution: 0.05     # the map's resolution
      track_unknown_space: true
      trinary_costmap: false
      lethal_cost_threshold: 100
      plugins: ["static_layer", "inflation_layer"]  # and your sensor layers
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

Nothing here depends on a particular resolution. The fixture of the
integration test is a 0.05 m warehouse map. The vineyard in the demo is at
0.03 m because that is the pixel size of the survey's terrain model.

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

## Integration test

The test needs [pixi](https://pixi.sh) and a Rust toolchain. Run it with bash;
it is not a zsh script.

```sh
cd crates/costmap_ros/nav2
bash test/run.sh
```

The first run installs the ROS Lyrical environment from `pixi.toml` and
`pixi.lock` (Nav2 costmap_2d 1.5.2, rmw_zenoh_cpp 0.10.6, rclpy).

`test/run.sh` builds the server and the converter, converts the fixture to a
GeoTIFF, starts `rmw_zenohd`, and then runs one phase per map format (YAML,
GeoTIFF) and published window size (12 m, 100 m). Each phase starts:

1. `test/check_costmap.py`, which also publishes `map -> base_link` on `/tf`;
2. `costmap_window_server` on `tests/fixtures/warehouse.yaml`, or on its
   GeoTIFF conversion;
3. a standalone Nav2 costmap node with `test/costmap_params.yaml` (an 8 m rolling
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
E2E_MAP=/path/cost.tif E2E_POSES="99.6,30 100.1,80 53.5,30" bash test/run.sh
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

`demo/run.sh` has Nav2 drive to one goal across a cost GeoTIFF that it never
holds whole.

```sh
cd crates/costmap_ros/nav2
DRIVE_MAP=/path/cost.tif DRIVE_START=53.5,30 DRIVE_GOAL=100.1,80 bash demo/run.sh
pixi run python demo/render.py --map /path/cost.tif \
    --recording /tmp/costmap_ros_drive.XXXXXX/recording.npz --out drive.mp4
```

It starts:

- a Zenoh router;
- `sim_robot.py`, a kinematic differential-drive robot that integrates Nav2's
  `/cmd_vel` and publishes `odom -> base_link` and `/odom`;
- `costmap_window_server` on the GeoTIFF, with 50 m windows republished every 2 m;
- `costmap_planner_server` on the same GeoTIFF;
- Nav2's planner (NavFn), controller (Regulated Pure Pursuit) and behaviour
  tree navigator, configured by `demo/nav2_params.yaml`, with the tree above.
  The global costmap is a 40 m rolling window and the local costmap a 6 m one.
  Each holds a StaticLayer on `/map` and an inflation layer, and nothing else;
- `mission.py`, which sends the goal as one `NavigateToPose`.

The mission passes when:

- Nav2 reports success with the robot within 0.5 m of the goal;
- no lethal cell of the raster, read from the file with Pillow, lies under the
  0.3 m robot disc at any recorded pose.

Result on a 166 x 196 m vineyard survey (5,547 x 6,525 cells, 36 M):

| | |
| --- | --- |
| Goal | 68 m from the start in a straight line, 30 m outside Nav2's global costmap |
| Reached | yes, 0.28 m from the goal |
| Distance driven | 96.4 m in 99 s |
| Routes from `costmap_planner_server` | 20, up to 8.8 ms each |
| Paths from Nav2's planner along the route | 90 |
| Windows published | 45, of 1,667 x 1,667 cells each |
| Lethal cells under the robot disc | 0 |
| Least distance from robot centre to a lethal cell | 0.47 m |
| Chunks of the file read by the window server | 2,314 of 8,874; 1,753 held at the end |

`render.py` draws the recording as a video: the last window served and Nav2's
global costmap on the dimmed raster, with the route, Nav2's path along it, the
track, and Nav2's local costmap.

The demo does not cover:

- sensors. There is no obstacle layer; Nav2 sees only the served map;
- localisation. `map -> odom` is a fixed identity transform;
- dynamics. The robot follows the commanded velocity exactly;
- recovery behaviours. The tree has none.

An earlier form of the demo had no route server and sent seven waypoints. It
found two things worth knowing:

- with Nav2's costmaps at 0.05 m over the 0.03 m map, StaticLayer dropped a
  9 cm obstacle and Nav2 drove over it;
- with the stock 1 Hz replanning tree, a slow plan still running when a
  waypoint was reached made Nav2 start the next goal on the previous path and
  report success at once.

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

- The `tf` module is not a tf2 replacement. The robot pose uses the latest
  transform of each frame, composed with no interpolation to a common time and
  no history. That is enough to place a window, where an error of a few
  centimetres does not matter. A transform library should replace it; the
  `transforms` crate is a candidate and has not been evaluated.
- `OccupancyGrid` carries 101 levels; costs 1..=252 arrive quantised as in the
  table above.
- Every publish is a full window. `map_msgs/OccupancyGridUpdate` patches a
  fixed map, and the window origin moves, so updates are not used.
- Nav2 must run `rmw_zenoh_cpp`, and a Zenoh router must connect both sides.
- `MapInfo` has no yaw, so maps with a rotated origin are rejected.
- The route is planned on an overview of the static map. See
  [Planner server](#planner-server).
