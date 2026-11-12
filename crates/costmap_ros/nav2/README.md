# nav2

Tests, a demo and a benchmark that run `costmap_window_server` and
`costmap_planner_server` against a real ROS 2 and Nav2. This file says what
each checks, how to run it and what it measured.

Nothing here is installed on a robot. CI runs `test/colcon.sh` and none of
the others. The only file a robot integrator may want is
`demo/nav2_params.yaml`, as a starting point for their own Nav2 parameters.

All need [pixi](https://pixi.sh) and, except `test/colcon.sh`, a Rust
toolchain. The first run installs ROS Lyrical with Nav2 1.5.2 and
`rmw_zenoh_cpp` from `pixi.toml` (4.3 GB). Run the scripts with bash from this
directory; they are not zsh scripts.

| Path | Role |
| --- | --- |
| `pixi.toml`, `pixi.lock` | The ROS environment the scripts use. |
| `lib.sh` | Shell helpers the scripts source: start and stop processes, start the Zenoh router. |
| `cost_raster.py` | Reads a cost GeoTIFF or a ROS map with Pillow, independently of the Rust readers. |
| `test/colcon.sh` | **colcon test.** Builds the package with colcon in a scratch workspace and starts the node with `ros2 launch`. |
| `test/run.sh` | **Integration test.** Starts the server and one Nav2 costmap node, moves the robot, and compares every cell Nav2 holds with the source map. |
| `test/check_costmap.py` | The comparison, and the robot's `/tf`. |
| `test/costmap_params.yaml` | Parameters of the Nav2 costmap node under test. |
| `demo/run.sh` | **Demo.** Starts the server with routes, a simulated robot and Nav2's planner, controller and navigator, and drives to one goal across a cost GeoTIFF. |
| `demo/nav2_params.yaml` | Nav2 parameters for the demo. |
| `demo/sim_robot.py` | Kinematic robot: integrates `/cmd_vel`, publishes `/tf` and `/odom`. |
| `demo/mission.py` | Sends the goal, records what Nav2 published, checks the track. |
| `demo/render.py` | Draws the recording as a video. Optional. |
| `bench/run.sh` | **Benchmark.** Measures memory, CPU and planning latency with these servers and with `nav2_map_server` and a global costmap of the whole map. |
| `bench/measure.py` | Sends the planning goals and reads each process's memory and CPU time. |
| `bench/to_ros_map.py` | Writes the cost GeoTIFF as a ROS map for `nav2_map_server`. |
| `bench/nav2_params.yaml` | Nav2 parameters for the whole-map case. |

```sh
bash test/colcon.sh
bash test/run.sh
DRIVE_MAP=/path/cost.tif DRIVE_START=53.5,30 DRIVE_GOAL=100.1,80 bash demo/run.sh
BENCH_MAP=/path/cost.tif BENCH_START=53.5,30 BENCH_GOAL=100.1,80 bash bench/run.sh
```

The scripts are bash because they start, wait for and stop a set of
processes. The nodes are Python because `rclpy` is the shortest way to call
Nav2's actions and read its topics.

## colcon test

`test/colcon.sh` builds the package as a robot's workspace would and checks
what ROS then finds. CI runs it.

```sh
bash test/colcon.sh
```

It uses the `build` environment of `pixi.toml` (colcon, `ament_cmake`, the
`ros2` command, `rmw_zenoh_cpp` and a Rust toolchain; 2.2 GB, without Nav2).
It links the repository into a scratch workspace, runs
`colcon build --packages-select costmap_ros`, and checks that:

- `ros2 pkg executables costmap_ros` lists the three programs;
- each node answers `--help` under `ros2 run`;
- the behaviour tree is in the package's share directory;
- the installed binaries carry no debug information;
- `ros2 launch costmap_ros costmap.launch.py` on the warehouse ROS map starts
  the window server with routes, connected to a Zenoh router.

The build takes 1 min 37 s on the machine below and installs two 24 MB nodes
and the 3.2 MB converter.

## Integration test

The test needs [pixi](https://pixi.sh) and a Rust toolchain. Run it with bash;
it is not a zsh script.

```sh
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
Pillow, derives the cost Nav2 should hold from the table in the crate README's
"Costs and occupancy", and compares
every cell exactly, graded costs included, reporting matched/total for free,
graded, inscribed/lethal and unknown cells. To check that it can fail, serve a
different raster than the checker reads with `E2E_SERVE_MAP=/path/shifted.tif`,
for example the raster shifted one pixel with
`gdal_translate -srcwin 1 0 W-1 H -a_ullr ULX ULY LRX-res LRY in.tif shifted.tif`.

## Drive demo

`demo/run.sh` has Nav2 drive to one goal across a map that it never holds
whole. `DRIVE_MAP` is a cost GeoTIFF or a ROS map YAML.

```sh
DRIVE_MAP=/path/cost.tif DRIVE_START=53.5,30 DRIVE_GOAL=100.1,80 bash demo/run.sh
pixi run python demo/render.py --map /path/cost.tif \
    --recording /tmp/costmap_ros_drive.XXXXXX/recording.npz --out drive.mp4
```

It starts:

- a Zenoh router;
- `sim_robot.py`, a kinematic differential-drive robot that integrates Nav2's
  `/cmd_vel` and publishes `odom -> base_link` and `/odom`;
- `costmap_window_server --routes` on the map, with its defaults: 50 m
  windows, a new one every 5 m;
- Nav2's planner (NavFn), controller (Regulated Pure Pursuit) and behaviour
  tree navigator, configured by `demo/nav2_params.yaml`, with
  `behavior_trees/navigate_to_pose_on_route.xml`.
  The global costmap takes the size of each window and the local costmap is
  a 6 m rolling one. Each holds a StaticLayer on `/map` and an inflation
  layer, and nothing else;
- `mission.py`, which sends the goal as one `NavigateToPose`.

The mission passes when:

- Nav2 reports success with the robot within 0.5 m of the goal;
- no lethal cell of the raster, read from the file with Pillow, lies under the
  0.3 m robot disc at any recorded pose.

Result on a 166 x 196 m vineyard survey (5,547 x 6,525 cells, 36 M):

| | |
| --- | --- |
| Goal | 68 m from the start in a straight line, 43 m outside the first window |
| Reached | yes, 0.29 m from the goal |
| Distance driven | 96.4 m in 99 s |
| Routes | 20: the first 112 ms, the rest 0.7 to 8.0 ms |
| Paths from Nav2's planner along the route | 94 |
| Windows published | 18, of 1,667 x 1,667 cells each |
| Lethal cells under the robot disc | 0 |
| Least distance from robot centre to a lethal cell | 0.47 m |
| Chunks of the file read by the window server | 2,352 of 8,874; 1,748 held at the end |

Result on the warehouse ROS map of this repository (8,927 x 3,417 cells at
0.05 m), served from its YAML with no conversion, from (60, 20) to (130, 60):

| | |
| --- | --- |
| Reached | yes, 0.28 m from the goal |
| Distance driven | 101.6 m in 103 s |
| Routes | 21 |
| Paths from Nav2's planner along the route | 94 |
| Windows published | 19, of 1,000 x 1,000 cells each |
| Lethal cells under the robot disc | 0 |
| Least distance from robot centre to a lethal cell | 0.71 m |

```sh
DRIVE_MAP=$PWD/../../../tests/fixtures/warehouse.yaml DRIVE_START=60,20 DRIVE_GOAL=130,60 bash demo/run.sh
```

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

## Benchmark against Nav2 holding the whole map

`bench/run.sh` runs two arrangements on one map with real Nav2 nodes and
reads each process's memory and CPU time from `/proc`:

- **Nav2 alone**: `nav2_map_server` publishes the whole map, and
  `planner_server` plans with NavFn (A*) on a global costmap that covers it.
- **With this package**: `costmap_window_server --routes` publishes a 50 m
  window and plans the trip, and `planner_server` plans the 15 m ahead with
  NavFn on a global costmap that takes the size of each window.

Vineyard survey, 5,547 x 6,525 cells at 0.03 m (36 M cells), a 99 m trip
planned from 11 starts along it. Machine as in [Latency](#latency). One run.
Every measured node is started by its executable, not through `ros2 run`,
which would add a Python process to its total.

| | Nav2 alone | With this package |
| --- | --- | --- |
| Resident memory, all processes | 768 MB | 158 MB |
| of which | `map_server` 108 MB, `planner_server` 660 MB | `costmap_window_server` 52 MB, `planner_server` 106 MB |
| Peak resident memory | 1,308 MB | 158 MB |
| Trip plan, first | 349 ms | 109 ms |
| Trip plan, later, median (max) | 316 ms (807 ms) | 4.9 ms (6.2 ms) |
| 15 m plan, median (max) | 267 ms (270 ms) | 5.2 ms (33 ms) |
| CPU time for the 16 plans | 6.6 s | 0.35 s |
| CPU at rest, share of one core | 1.8 % | 3.2 % |

Plan times are the `planning_time` each server reports. The first start on a
map also builds the route graph, 5.5 s here.

CPU at rest is higher by 1.4 % of one core:

- `costmap_window_server` uses 1.25 %. All of it is the `/tf` subscription,
  which the simulated robot feeds at 50 Hz; with no `/tf` traffic it uses
  0 %. A Hiroz subscriber to `/tf` whose callback does nothing uses 1.1 %, so
  the time goes in receiving each message (about 220 µs), not in handling it.
  `map_server` needs no robot pose and uses 0.1 %.
- `planner_server` uses 1.95 % against 1.7 %.

Three arrangements were measured on the way to this one:

| | CPU at rest | Resident memory |
| --- | --- | --- |
| Two processes, 40 m rolling global costmap, window sent again every 10 s | 8.15 % | 178 MB |
| One process (`--routes`), the rest the same | 6.85 % | 154 MB |
| One process, global costmap sized by the window | 4.0 % | 178 MB |
| The same, with no window sent again while the robot is still (the defaults) | 3.2 % | 158 MB |

A rolling global costmap copies its area from the static layer and inflates
it at every update, 1 Hz here: 5.7 % of a core. One sized by the window does
that once per window.

What the comparison leaves out: a costmap of the whole map cannot follow a
map that changes on disk, and neither arrangement here has sensor layers or a
controller running. The ROS map given to `map_server` is trinary, so graded
costs in the GeoTIFF reach Nav2 only through these servers.

```sh
BENCH_MAP=/path/cost.tif BENCH_START=53.5,30 BENCH_GOAL=100.1,80 bash bench/run.sh
```

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
