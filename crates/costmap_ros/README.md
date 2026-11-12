# costmap_ros

A ROS 2 node that lets Nav2 navigate a map too large for it to hold.
`costmap_window_server` takes the place of `nav2_map_server` and does two
things:

- **It serves a window of the map.** It publishes the square of the map
  around the robot on `/map`, as a `nav_msgs/OccupancyGrid`. Nav2's costmaps
  read it with their stock `StaticLayer`.
- **It plans routes across the whole map.** A behaviour tree asks it which way
  to go and asks Nav2's own planner for the next 15 m.

Nav2 itself is unchanged: no plugin, no patch, no C++ from this package. Its
planners, controllers, costmap layers and recoveries act as before.

Measured against `nav2_map_server` with a global costmap of the whole map, on
a 166 x 196 m survey at 0.03 m (36 M cells):

| | Nav2 alone | With this node |
| --- | --- | --- |
| Resident memory | 768 MB | 158 MB |
| Plan of a 99 m trip, median | 316 ms | 4.9 ms |
| Plan of the next 15 m, median | 267 ms | 5.2 ms |
| CPU at rest, share of one core | 1.8 % | 3.2 % |

CPU at rest is higher by 1.4 % of one core: the node subscribes to `/tf` for
the robot pose, which `map_server` does not need.
[nav2/README.md](nav2/README.md) has the method and the full table.

## Is it for you

It fits a robot that:

- runs Nav2 on `rmw_zenoh_cpp`. The nodes speak Zenoh directly, through
  [Hiroz](https://github.com/ZettaScaleLabs/hiroz), and link no ROS library.
  A robot on a DDS middleware would need a bridge, which has not been tried;
- has a map that costs too much to hold whole. `map_server` and every costmap
  with a `StaticLayer` each keep a copy, and planning on a global costmap
  takes time in proportion to the map.

Stay with `map_server` if the whole map fits comfortably. The 8,927 x
3,417-cell warehouse map in this repository is near the size where the
change starts to pay.

It has run against Nav2 1.5.2 on ROS 2 Lyrical with a simulated robot. It has
not run on a physical robot, on another Nav2 release, or beside AMCL.

## Install

All three ways install the same files: `costmap_window_server`,
`costmap_planner_server` and `costmap-from-ros-map` under `lib/costmap_ros`,
and the launch file and behaviour tree under `share/costmap_ros`. Afterwards
`ros2 pkg executables costmap_ros` lists the three programs.

### A release tarball

Each release from 0.3.0 carries `costmap_ros-VERSION-x86_64.tar.gz` and
`costmap_ros-VERSION-aarch64.tar.gz`. The programs link only `libc`, `libm`
and `libgcc_s`, from glibc 2.35 or later (Ubuntu 22.04). Nothing is compiled
on the robot.

```sh
sudo tar -xzf costmap_ros-0.3.0-aarch64.tar.gz -C /opt
source /opt/ros/lyrical/setup.bash
source /opt/costmap_ros/share/costmap_ros/local_setup.bash
```

### colcon, from source

```sh
cd ~/ros2_ws/src
git clone https://github.com/MJohnson459/costmap.git
cd ..
rosdep install --from-paths src/costmap/crates/costmap_ros --ignore-src
colcon build --packages-select costmap_ros
source install/setup.bash
```

The package is `ament_cmake`; its CMake file runs `cargo build --locked`,
which downloads the Rust dependencies. It needs Rust 1.92 or later. `rosdep`
installs `cargo`, which is 1.93 on Ubuntu 26.04. On Ubuntu 24.04 it is too
old; install a toolchain with [rustup](https://rustup.rs) first. The build
takes about 1.5 minutes and installs 51 MB.

### RoboStack and pixi

`conda/recipe.yaml` builds `ros-lyrical-costmap-ros` for a RoboStack
environment:

```sh
pixi exec rattler-build build --recipe crates/costmap_ros/conda/recipe.yaml \
    -c robostack-lyrical -c conda-forge --output-dir dist/conda
```

Add `dist/conda` as a channel of the pixi workspace, then
`pixi add ros-lyrical-costmap-ros`. Each release carries the built package for
linux-64. It is not on a public channel.

### Not on the ROS build farm

There is no `apt install ros-lyrical-costmap-ros`. The build downloads about
460 crates from crates.io, and the build farm's maintainers discourage builds
that fetch dependencies. `package.xml` is format 3 and builds with stock
colcon, so a release is possible if that changes or the crates are vendored.

## Replace map_server

1. **Stop starting `map_server`.** Remove it from the launch file and from the
   `node_names` of its lifecycle manager. `costmap_window_server` is not a
   lifecycle node; it serves as soon as it starts.

2. **Start `costmap_window_server`** on the map `map_server` was given:

   ```sh
   ros2 launch costmap_ros costmap.launch.py map:=/maps/site.yaml
   ```

   or from a launch file:

   ```python
   IncludeLaunchDescription(
       PathJoinSubstitution(
           [FindPackageShare("costmap_ros"), "launch", "costmap.launch.py"]
       ),
       launch_arguments={"map": "/maps/site.yaml"}.items(),
   )
   ```

   It connects to the Zenoh router the robot already runs for
   `rmw_zenoh_cpp`. `ros2 launch costmap_ros costmap.launch.py --show-args`
   lists the arguments.

3. **Set the costmaps to follow the window**, at the map's resolution:

   ```yaml
   global_costmap:
     global_costmap:
       ros__parameters:
         rolling_window: false    # takes the size of each window served
         resolution: 0.05         # the map's resolution
         track_unknown_space: true
         trinary_costmap: false
         lethal_cost_threshold: 100
         plugins: ["static_layer", "inflation_layer"]
         static_layer:
           plugin: nav2_costmap_2d::StaticLayer
           map_topic: /map
           map_subscribe_transient_local: true

   local_costmap:
     local_costmap:
       ros__parameters:
         rolling_window: true     # as on any Nav2 robot
         width: 6
         height: 6
         resolution: 0.05         # the map's resolution
         # the same three costmap parameters and static_layer as above,
         # then your sensor layers and inflation
   ```

   The global costmap is not rolling. Its `StaticLayer` resizes it to each
   window, as it does for a map that grows under SLAM, and Nav2 recomputes it
   only then. A rolling global costmap redraws its whole area every update:
   5.7 % of a core at 1 Hz for a 40 m costmap at 0.03 m, against 1.95 % this
   way. Use a rolling one only if the global costmap carries sensor layers,
   because a resize clears them.

   Use the map's own resolution. `StaticLayer` point-samples a map of another
   resolution: a costmap at 0.05 m over a 0.03 m map dropped a 9 cm obstacle,
   and Nav2 planned and drove through it.

4. **Give `bt_navigator` the route tree.** Set `default_nav_to_pose_bt_xml` to
   the installed file, which this prints:

   ```sh
   echo "$(ros2 pkg prefix --share costmap_ros)/behavior_trees/navigate_to_pose_on_route.xml"
   ```

   Skip this step and Nav2 still works, for goals inside the served window.

5. **Keep publishing `map -> base_link` on `/tf`**, as Nav2 already needs.

A `NavigateToPose` goal anywhere on the map now works.

## How a goal is reached

[`behavior_trees/navigate_to_pose_on_route.xml`](behavior_trees/navigate_to_pose_on_route.xml)
differs from Nav2's stock replanning tree in one place: where the stock tree
has one `ComputePathToPose` node, this has four.

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

1. Every 5 s it asks `costmap_window_server` for the route to the goal.
2. Every second it cuts the route to the 15 m ahead of the robot, takes the
   end of that stretch as a goal, and asks Nav2's planner for a path to it.
3. The controller follows that path, as in the stock tree.

The route only says which way to go. The path the robot drives comes from
Nav2's planner on Nav2's costmap, so footprint checks, costmap filters and
sensed obstacles apply to it. A robot with its own tree adds these nodes in
place of its `ComputePathToPose` and keeps the rest.

- Every node is one Nav2 1.5.2 ships. Node names and ports differ between
  Nav2 releases, and `GetPoseFromPath` is absent from older ones.
- `NavigateThroughPoses` uses a separate tree. None is provided for it.

## Choosing the sizes

Three sizes are involved: the window the server publishes (`window_m`, `W`),
the tree's `distance_forward` (`D`), and the local costmap. None grows with
the map. The global costmap is the window.

1. **Local costmap.** Size it for the controller, as on any Nav2 robot: 3 to
   6 m.
2. **Planning distance `D`.** How far ahead Nav2's planner looks along the
   route. It sets how wide a detour around a sensed obstacle can be. The tree
   uses 15 m.
3. **Server window `W`.** The point `D` along the route must lie inside the
   last window until the next one arrives. A window is centred on the robot
   when it is sent, and the next is sent `min_move_m` later, so:

   `W / 2 > D + min_move_m + v_max * (1 / rate_hz + 0.2)`

   With the defaults (`W` 50, `--min-move-m 5`, `--rate-hz 2`) and 1 m/s:
   25 > 15 + 5 + 0.7. The 0.2 s is the longest time measured from a robot
   move to Nav2 holding the new window. Outside the window Nav2 sees unknown
   cells.

A smaller `min_move_m` sends windows more often, and Nav2 recomputes its
global costmap for each. With a rolling global costmap of width `G` instead,
the rule is `(W - G) / 2 > min_move_m + v_max * (1 / rate_hz + 0.2)` and
`G / 2 > D`.

A larger `W` costs memory and bandwidth: each window is one message of
`(W / resolution)^2` bytes, and each `StaticLayer` keeps a copy.

| `W` | at 0.05 m | at 0.03 m |
| --- | --- | --- |
| 20 m | 0.16 MB | 0.44 MB |
| 50 m | 1.0 MB | 2.8 MB |
| 100 m | 4.0 MB | 11 MB |

## Maps

`--map` selects the format by extension. Any other extension is a startup
error.

| Extension | Format | Memory |
| --- | --- | --- |
| `.yaml`, `.yml` | ROS map: the YAML and image `map_server` reads | the whole map, one byte per cell, in each node |
| `.tif`, `.tiff` | cost GeoTIFF: single-band 8-bit costs, 255 unknown | the chunks near the robot and the tiles a route passes |

A ROS map is held whole because a PNG or PGM cannot be read in part. That is
no worse than `map_server`, and Nav2 still holds only windows. For a map too
large for it, convert it once:

```sh
ros2 run costmap_ros costmap-from-ros-map site.yaml site.tif
```

The GeoTIFF's lower-left corner is the YAML origin. Its `map` frame starts at
that corner, so it equals the YAML map's frame only when the origin is (0, 0);
otherwise `map` coordinates shift by the origin.

[`costmap-from-ortho`](../costmap-geo) writes cost GeoTIFFs from drone
surveys. For those the `map` frame is the raster's lower-left corner, x east
and y north. A robot localised in projected coordinates (RTK GNSS in UTM,
say) needs a static transform from that frame to `map`; the window server
logs the corner's coordinates at start. It does no reprojection.

**The route graph.** Routes are planned on a graph of the map:
where each 256-cell tile's border can be crossed, and what it costs to travel
between crossings, on costs inflated for the robot. It is built at the map's
own resolution, so a doorway the robot fits through stays open. For a GeoTIFF
the first start builds the graph (5.5 s for 36 M cells on 16 cores) and
stores it beside the raster as `site.tif.tilegraph` (0.7 MB); later starts
read it. It is rebuilt when `--robot-radius-m`, `--inflation-m` or this
package's version changes, or when the raster's length, modification time or
first or last 64 KiB change. A raster edited in the middle with its length
and time kept is not noticed; delete the `.tilegraph` file to force a
rebuild. If the directory cannot be written, the
node logs a warning and builds the graph at every start. For a ROS map the graph is built at every
start: 4.4 s for the 30.5 M-cell warehouse map, with the node's memory
peaking at 150 MB.

## Node reference

The programs take command-line options, not ROS parameters, and cannot be
remapped or namespaced with `--ros-args`. The launch file passes its arguments
through as options.

### costmap_window_server

Publishes the window on `--topic` with QoS reliable, transient local, depth 1,
which is what `StaticLayer` requests with `map_subscribe_transient_local:
true`: a subscriber that starts late receives the last window. It checks the
robot pose `--rate-hz` times a second and publishes when no window has been
sent or when the robot is `--min-move-m` from the centre of the last one. The
window origin is a whole number of cells from the map origin, so Nav2 never
sees a cell resampled.

With `--routes`, which the launch file passes, the same process serves the
routes described under [costmap_planner_server](#costmap_planner_server). The
two then share one `/tf` subscription and one copy of the program: 52 MB and
1.25 % of a core at rest, against 78 MB and 2.45 % as two processes.

| Option | Default | Meaning |
| --- | --- | --- |
| `--map` | required | ROS map YAML or cost GeoTIFF |
| `--window-m` | 50 | side of the square window, metres |
| `--topic` | `map` | output topic |
| `--map-frame` | `map` | frame of the map and of the published grid |
| `--base-frame` | `base_link` | robot frame |
| `--rate-hz` | 2 | pose checks per second |
| `--min-move-m` | 5 | movement that triggers a new window, metres |
| `--keep-alive-s` | 0 | age at which the same window is sent again, seconds; 0 never sends it again |
| `--routes` | off | also plan routes |
| `--route-action`, `--route-topic`, `--robot-radius-m`, `--inflation-m` | | with `--routes`: as for `costmap_planner_server` |

### costmap_planner_server

The routes on their own, for running them apart from the window. Its options
other than `--map` are also `costmap_window_server`'s, with `--action` there
named `--route-action`.

Serves `nav2_msgs/action/ComputePathToPose` as `compute_route_to_pose` and
publishes each route on `/route`.

- A route starts at the goal's `start` pose when `use_start` is set, and at
  the latest `map -> base_link` transform otherwise. It passes through the
  goal's `viapoints` in order.
- It is returned as poses at most 0.1 m apart that face along it.
- A start or goal on an impassable cell moves to the nearest passable one
  within 1 m.
- Poses must be in the map frame. A pose in another frame aborts the goal
  with `TF_ERROR`; the node does not transform goals.
- Failures abort the goal with Nav2's error codes: `TF_ERROR`,
  `START_OCCUPIED`, `GOAL_OCCUPIED`, `NO_VALID_PATH`, and `UNKNOWN` if the
  map file cannot be read.
- A goal cancelled while its route is being planned ends as cancelled.
- There is one planner, so `planner_id` is not read.
- A route passes through the graph's crossings, at most 32 cells apart along
  a tile border, so it can cost more than the cheapest path on the map: at
  most 0.9 % more over 12 routes on the warehouse map.

| Option | Default | Meaning |
| --- | --- | --- |
| `--map` | required | ROS map YAML or cost GeoTIFF |
| `--action` | `compute_route_to_pose` | action served |
| `--route-topic` | `route` | topic each route is also published on |
| `--map-frame` | `map` | frame of the map and of the routes |
| `--base-frame` | `base_link` | robot frame |
| `--robot-radius-m` | 0.3 | robot radius, metres; cells nearer an obstacle are impassable |
| `--inflation-m` | 1.5 | distance over which routes are pushed from obstacles, metres; at least the robot radius is used |

Started with `--action compute_path_to_pose --route-topic plan`, it answers
the action Nav2's stock trees call, and `planner_server` and the global
costmap can be removed. The robot then drives the route itself. That suits
only a robot that turns on the spot, is close to circular, and needs no
costmap filters or sensed obstacles in its global plan.

### Both programs

- **Zenoh.** The session follows `rmw_zenoh_cpp`'s conventions: it connects to
  a router at `tcp/localhost:7447` and reads `ZENOH_SESSION_CONFIG_URI`,
  `ZENOH_CONFIG_OVERRIDE` and `ROS_DOMAIN_ID`.
- **Logging.** `RUST_LOG` sets the level, for example
  `RUST_LOG=costmap_window_server=debug` for one line per published window.
- **Stopping.** Ctrl-C or SIGTERM.

## Costs and occupancy

`StaticLayer` converts each occupancy value with
`StaticLayer::interpretValue`. With the costmap parameters above:

| Cost in the map | Occupancy sent | Cost Nav2 stores |
| --- | --- | --- |
| 0 (free) | 0 | 0 |
| 1..=248 | `ceil(cost * 100 / 254)`, 1..=98 | `floor(occ * 2.54)`: `cost` to `cost + 2` |
| 249..=252 | 99 | 253 |
| 253 (inscribed) | 99 | 253 |
| 254 (lethal) | 100 | 254 |
| 255 (unknown) | -1 | 255 |

An `OccupancyGrid` cell has 101 levels, so graded costs quantise in steps of
2.54. The occupancy is rounded up: Nav2 never receives a cost lower than the
one in the map, and never receives lethal for a cost below 254.
`track_unknown_space`, `trinary_costmap` and `lethal_cost_threshold` are
parameters of the costmap node, not of the layer, and
`inscribed_obstacle_cost_value` (99) and `unknown_cost_value` (255) must stay
at their defaults.

## Limits

- **Zenoh only.** Every node that talks to these runs `rmw_zenoh_cpp` and
  reaches the same router.
- **Static map only.** The node reads the file. Sensed obstacles go in Nav2's
  own layers, and Nav2's planner routes around them within its global
  costmap. The route across the map does not change: a route that stays
  blocked is not replanned around.
- **No ROS parameters, lifecycle or composition.** See
  [Node reference](#node-reference).
- **A non-rolling global costmap loses its sensed obstacles at each new
  window**, every `min_move_m`. Nav2 resets every layer of a costmap when it
  resizes. Keep sensor layers in the local costmap, or make the global one
  rolling.
- **Robot pose.** The nodes use the latest transform of each frame, with no
  interpolation to a common time and no history. That places a window, where
  a few centimetres do not matter. It is not a tf2 replacement.
- **Every publish is a full window.** `map_msgs/OccupancyGridUpdate` patches a
  fixed map, and the window origin moves.
- **Maps with a rotated origin are rejected.**

## How it was checked

[nav2/README.md](nav2/README.md) describes each of these and how to run it.

- **Unit tests**, in CI: 49 in this crate, among them a route through a 0.9 m
  doorway for a 0.3 m robot.
- **colcon build**, in CI: builds the package in a scratch workspace and
  starts the node with `ros2 launch`.
- **Integration test**, by hand: every cell a Nav2 costmap holds equals the
  source map, across five 65 m moves, for both map formats.
- **Drive demo**, by hand: Nav2 drove a simulated robot 96.4 m in 99 s across
  the survey GeoTIFF to a goal 43 m outside the first window, with 0.47 m
  least distance to a lethal cell; and 101.6 m in 103 s across the warehouse
  ROS map, served from its YAML, with 0.71 m.
- **Benchmark**, by hand: the table at the top.

## The library

The crate is also a Rust library, built with cargo alone and no ROS install.

- `convert`: `Costmap` to and from `nav_msgs/OccupancyGrid`.
- `source`: the `MapSource` trait a window is cut from, for a `Costmap` in
  memory and for `GeoTiffSource`.
- `republish`: when a new window is due.
- `route`: `RoutePlanner`. The `plan_route` example plans one route and
  prints its timings, without ROS.
- `nav2_msgs`: the `ComputePathToPose` action type, generated at build time
  from `msgs/nav2_msgs`; `hiroz-msgs` does not bundle Nav2's interfaces.
- `tf`: a transform tree fed from `/tf` and `/tf_static`. Hiroz has no tf2
  port; a transform library should replace it.
