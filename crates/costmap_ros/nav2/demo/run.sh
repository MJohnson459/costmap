#!/usr/bin/env bash
# Nav2 drives a simulated robot across a cost GeoTIFF it never holds whole.
#
# Usage: DRIVE_MAP=/path/cost.tif DRIVE_START=x,y DRIVE_GOAL=x,y bash demo/run.sh
#
# costmap_window_server publishes a SERVER_WINDOW_M (default 60) metre window
# of the map around the robot. Nav2's global costmap is a 50 m rolling window
# and its local costmap a 6 m one; both hold only that window plus inflation.
# route_waypoints plans the whole route with costmap's A* and the mission sends
# Nav2 one waypoint at a time, DRIVE_SPACING_M (default 15) metres apart.
#
# Starts a Zenoh router, a kinematic robot (sim_robot.py), the window server,
# Nav2's planner, controller and behaviour tree navigator, and the mission
# (mission.py), which reports PASS/FAIL and writes recording.npz to
# DRIVE_OUT (default: the log directory). render.py turns that into a
# video. Every process is started in its own process group and stopped by that
# group id; nothing else on the machine is signalled.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$NAV2_DIR/../../.." && pwd)"
[[ -n "${DRIVE_MAP:-}" && -n "${DRIVE_START:-}" && -n "${DRIVE_GOAL:-}" ]] || {
    echo "usage: DRIVE_MAP=cost.tif DRIVE_START=x,y DRIVE_GOAL=x,y bash demo/run.sh" >&2
    exit 2
}
[[ -f "$DRIVE_MAP" ]] || { echo "DRIVE_MAP $DRIVE_MAP not found" >&2; exit 2; }
SERVER_WINDOW_M="${SERVER_WINDOW_M:-60}"
DRIVE_SPACING_M="${DRIVE_SPACING_M:-15}"
STAGE_TIMEOUT_S=60
MISSION_TIMEOUT_S=900
LOG_DIR="$(mktemp -d -t costmap_ros_drive.XXXXXX)"
DRIVE_OUT="${DRIVE_OUT:-$LOG_DIR}"
LOG_TAG=drive
source "$NAV2_DIR/lib.sh"

log "building costmap_window_server and route_waypoints"
cargo build -j 6 --release --quiet --manifest-path "$REPO/Cargo.toml" -p costmap_ros \
    --bin costmap_window_server --example route_waypoints
SERVER="$REPO/target/release/costmap_window_server"

WAYPOINTS="$("$REPO/target/release/examples/route_waypoints" --map "$DRIVE_MAP" \
    --start "$DRIVE_START" --goal "$DRIVE_GOAL" --spacing-m "$DRIVE_SPACING_M")" ||
    fail "no route from $DRIVE_START to $DRIVE_GOAL"
log "waypoints: $WAYPOINTS"

enter_ros_env
RESOLUTION="$(python -u "$NAV2_DIR/test/check_costmap.py" --map "$DRIVE_MAP" --print-resolution)" ||
    fail "could not read the resolution of $DRIVE_MAP"
[[ "$RESOLUTION" == *.* ]] || RESOLUTION="$RESOLUTION.0"
log "map resolution $RESOLUTION m; server window ${SERVER_WINDOW_M} m"
start_router

start sim python -u "$HERE/sim_robot.py" --x "${DRIVE_START%,*}" --y "${DRIVE_START#*,}"
wait_for "simulated robot" "$STAGE_TIMEOUT_S" log_has sim "sim: robot at"

RUST_LOG="warn,costmap_window_server=debug,costmap_ros=debug" start server "$SERVER" \
    --map "$DRIVE_MAP" --window-m "$SERVER_WINDOW_M" --rate-hz 5 --min-move-m 2
wait_for "costmap_window_server first publish" "$STAGE_TIMEOUT_S" log_has server "published"
require_alive server "$server_pid"

# Both costmaps run at the map's resolution. StaticLayer point-samples a map of
# another resolution and drops obstacles smaller than a costmap cell.
# The tree plans once per goal: the map does not change, and a periodic replan
# still running when a waypoint is reached makes Nav2 start the next goal on
# the previous path.
BT_DIR="$CONDA_PREFIX/share/nav2_bt_navigator/behavior_trees"
start planner ros2 run nav2_planner planner_server --ros-args \
    --params-file "$HERE/nav2_params.yaml" -p "resolution:=$RESOLUTION"
start controller ros2 run nav2_controller controller_server --ros-args \
    --params-file "$HERE/nav2_params.yaml" -p "resolution:=$RESOLUTION"
start navigator ros2 run nav2_bt_navigator bt_navigator --ros-args \
    --params-file "$HERE/nav2_params.yaml" \
    -p "default_nav_to_pose_bt_xml:=$BT_DIR/navigate_w_replanning_only_if_goal_is_updated.xml"
start lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    --params-file "$HERE/nav2_params.yaml"
wait_for "Nav2 (planner, controller, navigator active)" "$STAGE_TIMEOUT_S" \
    log_has lifecycle "Managed nodes are active"

start mission python -u "$HERE/mission.py" --map "$DRIVE_MAP" \
    --waypoints "$WAYPOINTS" --out "$DRIVE_OUT"
deadline=$((SECONDS + MISSION_TIMEOUT_S))
while process_alive "$mission_pid"; do
    ((SECONDS < deadline)) || fail "mission did not finish within ${MISSION_TIMEOUT_S} s"
    sleep 0.5
done
status=0
wait "$mission_pid" || status=$?
cat "$LOG_DIR/mission.log"

# The server logs its chunk counters when it exits.
stop_group "$server_pid"
grep -E "GeoTIFF chunks" "$LOG_DIR/server.log" | sed 's/^.*GeoTIFF chunks/[drive] server chunks/' || true
log "$(grep -c "published" "$LOG_DIR/server.log") windows published by the server"
log "recording: $DRIVE_OUT/recording.npz"
((status == 0)) || fail "mission failed"
log "PASS (logs: $LOG_DIR)"
