# nav2

A test rig, a demo and a benchmark that run `costmap_window_server` and
`costmap_planner_server` against a real Nav2.
Nothing here is installed on a robot, and CI does not run it. The only file a
robot integrator may want is `demo/nav2_params.yaml`, as a starting point for
their own Nav2 parameters.

All three need [pixi](https://pixi.sh) and a Rust toolchain. The first run
installs ROS Lyrical with Nav2 1.5.2 and `rmw_zenoh_cpp` from `pixi.toml`
(4.3 GB).

| Path | Role |
| --- | --- |
| `pixi.toml`, `pixi.lock` | The ROS environment the scripts use. |
| `lib.sh` | Shell helpers the scripts source: start and stop processes, start the Zenoh router. |
| `cost_raster.py` | Reads a cost GeoTIFF with Pillow, independently of the Rust reader. |
| `test/run.sh` | **Integration test.** Starts the server and one Nav2 costmap node, moves the robot, and compares every cell Nav2 holds with the source map. |
| `test/check_costmap.py` | The comparison, and the robot's `/tf`. |
| `test/costmap_params.yaml` | Parameters of the Nav2 costmap node under test. |
| `demo/run.sh` | **Demo.** Starts both servers, a simulated robot and Nav2's planner, controller and navigator, and drives to one goal across a cost GeoTIFF. |
| `demo/nav2_params.yaml` | Nav2 parameters for the demo. |
| `demo/sim_robot.py` | Kinematic robot: integrates `/cmd_vel`, publishes `/tf` and `/odom`. |
| `demo/mission.py` | Sends the goal, records what Nav2 published, checks the track. |
| `demo/render.py` | Draws the recording as a video. Optional. |
| `bench/run.sh` | **Benchmark.** Measures memory, CPU and planning latency with these servers and with `nav2_map_server` and a global costmap of the whole map. |
| `bench/measure.py` | Sends the planning goals and reads each process's memory and CPU time. |
| `bench/to_ros_map.py` | Writes the cost GeoTIFF as a ROS map for `nav2_map_server`. |
| `bench/nav2_params.yaml` | Nav2 parameters for the whole-map case. |

```sh
bash test/run.sh
DRIVE_MAP=/path/cost.tif DRIVE_START=53.5,30 DRIVE_GOAL=100.1,80 bash demo/run.sh
BENCH_MAP=/path/cost.tif BENCH_START=53.5,30 BENCH_GOAL=100.1,80 bash bench/run.sh
```

The scripts are bash because they start, wait for and stop a set of
processes. The nodes are Python because `rclpy` is the shortest way to call
Nav2's actions and read its topics. See the [crate README](../README.md) for
what each checks and the results.
