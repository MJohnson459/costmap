#!/usr/bin/env bash
# Builds the costmap_ros package with colcon in a scratch workspace, then
# checks that ROS finds what it installs and that the launch file starts the
# window server with routes.
#
# Usage: bash test/colcon.sh
#
# Uses the `build` environment of pixi.toml: colcon, ament_cmake, the ros2
# command, rmw_zenoh_cpp and a Rust toolchain, without Nav2. It checks that:
# - `ros2 pkg executables` lists the three programs,
# - each node answers `--help` under `ros2 run`,
# - the behaviour tree is in the package's share directory,
# - the installed binaries carry no debug information,
# - `ros2 launch costmap_ros costmap.launch.py` on the warehouse ROS map
#   starts the window server with routes, connected to a Zenoh router.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$NAV2_DIR/../../.." && pwd)"
WORKSPACE="$(mktemp -d -t costmap_ros_colcon.XXXXXX)"
LOG_DIR="$WORKSPACE/log"
LOG_TAG=colcon
STAGE_TIMEOUT_S=60
source "$NAV2_DIR/lib.sh"

# The environment's activation scripts read unset variables.
set +u
eval "$(pixi shell-hook --manifest-path "$NAV2_DIR/pixi.toml" -e build --shell bash)"
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=73

# A cargo home of its own, so that a ~/.cargo/config.toml written for another
# toolchain's linker does not reach this build.
export CARGO_HOME="$WORKSPACE/cargo_home"

mkdir -p "$WORKSPACE/src" "$LOG_DIR"
ln -s "$REPO" "$WORKSPACE/src/costmap"
cd "$WORKSPACE"
log "building in $WORKSPACE"
colcon build --packages-select costmap_ros >"$LOG_DIR/colcon.log" 2>&1 || {
    tail -n 40 "$LOG_DIR/colcon.log" >&2
    fail "colcon build failed"
}
set +u
source install/setup.bash
set -u

programs="$(ros2 pkg executables costmap_ros)"
for program in costmap_window_server costmap_planner_server costmap-from-ros-map; do
    grep -q "costmap_ros $program\$" <<<"$programs" || fail "ros2 pkg does not list $program"
done
ros2 run costmap_ros costmap_window_server --help | grep -q -- "--window-m" ||
    fail "ros2 run costmap_window_server --help"
ros2 run costmap_ros costmap_planner_server --help | grep -q -- "--robot-radius-m" ||
    fail "ros2 run costmap_planner_server --help"
share="$(ros2 pkg prefix --share costmap_ros)"
[[ -f "$share/behavior_trees/navigate_to_pose_on_route.xml" ]] ||
    fail "the behaviour tree is not installed in $share"
size_kb="$(du -k "install/costmap_ros/lib/costmap_ros/costmap_window_server" | cut -f1)"
((size_kb < 60000)) || fail "costmap_window_server is ${size_kb} kB; debug information was kept"

start_router
start launch ros2 launch costmap_ros costmap.launch.py \
    "map:=$REPO/tests/fixtures/warehouse.yaml"
wait_for "costmap_window_server under ros2 launch" "$STAGE_TIMEOUT_S" \
    log_has launch "costmap_window_server.*serving"
wait_for "routes under ros2 launch" "$STAGE_TIMEOUT_S" \
    log_has launch "serving compute_route_to_pose"
require_alive launch "$launch_pid"

cleanup
du -h install/costmap_ros/lib/costmap_ros/* | sed "s/^/[colcon] /"
log "PASS"
rm -rf "$WORKSPACE"
