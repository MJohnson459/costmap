#!/usr/bin/env bash
# Builds the tiled_map_server package with colcon in a scratch workspace, then
# checks that ROS finds what it installs and that the launch file starts the
# node with routes.
#
# Usage: bash test/colcon.sh
#
# Uses the `build` environment of pixi.toml: colcon, ament_cmake, the ros2
# command, rmw_zenoh_cpp and a Rust toolchain, without Nav2. It checks that:
# - `ros2 pkg executables` lists the two programs,
# - the node answers `--help` under `ros2 run`,
# - the behaviour tree is in the package's share directory,
# - the installed binaries carry no debug information,
# - `ros2 launch tiled_map_server tiled_map_server.launch.py` on the warehouse ROS map
#   starts the node with routes, connected to a Zenoh router.
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
colcon build --packages-select tiled_map_server >"$LOG_DIR/colcon.log" 2>&1 || {
    tail -n 40 "$LOG_DIR/colcon.log" >&2
    fail "colcon build failed"
}
set +u
source install/setup.bash
set -u

programs="$(ros2 pkg executables tiled_map_server)"
for program in tiled_map_server costmap-from-ros-map; do
    grep -q "tiled_map_server $program\$" <<<"$programs" || fail "ros2 pkg does not list $program"
done
ros2 run tiled_map_server tiled_map_server --help | grep -q -- "--window-m" ||
    fail "ros2 run tiled_map_server --help"
share="$(ros2 pkg prefix --share tiled_map_server)"
[[ -f "$share/behavior_trees/navigate_to_pose_on_route.xml" ]] ||
    fail "the behaviour tree is not installed in $share"
size_kb="$(du -k "install/tiled_map_server/lib/tiled_map_server/tiled_map_server" | cut -f1)"
((size_kb < 60000)) || fail "tiled_map_server is ${size_kb} kB; debug information was kept"

start_router
start launch ros2 launch tiled_map_server tiled_map_server.launch.py \
    "map:=$REPO/tests/fixtures/warehouse.yaml"
wait_for "tiled_map_server under ros2 launch" "$STAGE_TIMEOUT_S" \
    log_has launch "tiled_map_server.*serving"
wait_for "routes under ros2 launch" "$STAGE_TIMEOUT_S" \
    log_has launch "serving compute_route_to_pose"
require_alive launch "$launch_pid"

cleanup
du -h install/tiled_map_server/lib/tiled_map_server/* | sed "s/^/[colcon] /"
log "PASS"
rm -rf "$WORKSPACE"
