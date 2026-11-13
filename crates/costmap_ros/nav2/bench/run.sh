#!/usr/bin/env bash
# Compares the memory, CPU and planning latency of two ways to navigate one
# map, with real Nav2 nodes from the pixi environment.
#
# Usage: BENCH_MAP=/path/cost.tif BENCH_START=x,y BENCH_GOAL=x,y bash bench/run.sh
#
# nav2:    nav2_map_server publishes the whole map and planner_server plans on
#          a global costmap covering all of it (bench/nav2_params.yaml).
# costmap: tiled_map_server --routes publishes a 50 m window and plans
#          the trip, and planner_server plans the 15 m ahead on a global
#          costmap the size of that window (demo/nav2_params.yaml).
#
# Each case plans the trip from 11 starts along it and its first 15 m five
# times, then idles for 20 s. bench/measure.py reports latencies and each
# process's resident memory and CPU time. Results are written to BENCH_OUT
# (default: a temporary directory) as nav2.json and costmap.json.
# BENCH_CASES="nav2 costmap" chooses the cases. BENCH_SERVER_ARGS adds options to
# tiled_map_server, such as "--pose-topic /pose".
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$NAV2_DIR/../../.." && pwd)"
[[ -n "${BENCH_MAP:-}" && -n "${BENCH_START:-}" && -n "${BENCH_GOAL:-}" ]] || {
    echo "usage: BENCH_MAP=cost.tif BENCH_START=x,y BENCH_GOAL=x,y bash bench/run.sh" >&2
    exit 2
}
[[ -f "$BENCH_MAP" ]] || { echo "BENCH_MAP $BENCH_MAP not found" >&2; exit 2; }
BENCH_CASES="${BENCH_CASES:-nav2 costmap}"
STAGE_TIMEOUT_S=180
LOG_DIR="$(mktemp -d -t costmap_ros_bench.XXXXXX)"
BENCH_OUT="${BENCH_OUT:-$LOG_DIR}"
LOG_TAG=bench
source "$NAV2_DIR/lib.sh"
mkdir -p "$BENCH_OUT"

log "building tiled_map_server"
cargo build -j 6 --release --quiet --manifest-path "$REPO/Cargo.toml" -p costmap_ros \
    --bin tiled_map_server

enter_ros_env
# The measured nodes are started by path: `ros2 run` would leave a Python
# process in each one's process group, and its memory in the node's total.
ROS_LIB="$CONDA_PREFIX/lib"
RESOLUTION="$(python -u "$NAV2_DIR/test/check_costmap.py" --map "$BENCH_MAP" --print-resolution)" ||
    fail "could not read the resolution of $BENCH_MAP"
[[ "$RESOLUTION" == *.* ]] || RESOLUTION="$RESOLUTION.0"

# Starts what both cases share: a router and a robot standing at the start.
start_common() {
    started_groups=()
    start_router
    start sim python -u "$NAV2_DIR/demo/sim_robot.py" \
        --x "${BENCH_START%,*}" --y "${BENCH_START#*,}"
    wait_for "simulated robot" "$STAGE_TIMEOUT_S" log_has sim "sim: robot at"
}

measure() {
    local name="$1"
    shift
    python -u "$HERE/measure.py" --start "$BENCH_START" --goal "$BENCH_GOAL" \
        --out "$BENCH_OUT/$name.json" "$@" >"$LOG_DIR/measure_$name.log" 2>&1 || {
        tail -n 20 "$LOG_DIR"/*.log >&2
        fail "measuring $name failed"
    }
    cat "$BENCH_OUT/$name.json"
}

run_nav2() {
    log "case nav2: map_server and a global costmap of the whole map"
    python -u "$HERE/to_ros_map.py" --map "$BENCH_MAP" --out "$LOG_DIR/ros_map" >/dev/null
    start_common
    start map_server "$ROS_LIB/nav2_map_server/map_server" --ros-args \
        --params-file "$HERE/nav2_params.yaml" -p "yaml_filename:=$LOG_DIR/ros_map/map.yaml"
    start planner "$ROS_LIB/nav2_planner/planner_server" --ros-args \
        --params-file "$HERE/nav2_params.yaml"
    start lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
        --params-file "$HERE/nav2_params.yaml"
    wait_for "Nav2 (map_server, planner active)" "$STAGE_TIMEOUT_S" \
        log_has lifecycle "Managed nodes are active"
    measure nav2 --trip-action compute_path_to_pose --near-action compute_path_to_pose \
        --group "map_server=$map_server_pid" --group "planner_server=$planner_pid"
}

run_costmap() {
    log "case costmap: a 50 m window and routes from one process, and a global costmap of that window"
    # The graph is built at a robot's first start on a map, not at every start.
    rm -f "$BENCH_MAP.tilegraph"
    start_common
    # The server logs each publish at debug level.
    RUST_LOG="warn,tiled_map_server=debug,costmap_ros=info" start server \
        "$REPO/target/release/tiled_map_server" \
        --map "$BENCH_MAP" --window-m 50 --routes ${BENCH_SERVER_ARGS:-}
    wait_for "routes" "$STAGE_TIMEOUT_S" log_has server "serving compute_route_to_pose"
    wait_for "tiled_map_server first publish" "$STAGE_TIMEOUT_S" log_has server "published"
    grep -o "built the tile graph in [0-9.]* s" "$LOG_DIR/server.log" | sed "s/^/[bench] /" || true
    # The demo publishes the global costmap for its recording; a robot need not.
    start planner "$ROS_LIB/nav2_planner/planner_server" --ros-args \
        --params-file "$NAV2_DIR/demo/nav2_params.yaml" -p "resolution:=$RESOLUTION" \
        -p "publish_frequency:=0.0"
    start lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
        --params-file "$NAV2_DIR/demo/nav2_params.yaml" -p 'node_names:=["planner_server"]'
    wait_for "Nav2 (planner active)" "$STAGE_TIMEOUT_S" \
        log_has lifecycle "Managed nodes are active"
    measure costmap --trip-action compute_route_to_pose --near-action compute_path_to_pose \
        --group "tiled_map_server=$server_pid" --group "planner_server=$planner_pid"
}

for case in $BENCH_CASES; do
    "run_$case"
    cleanup
done
log "results: $BENCH_OUT"
