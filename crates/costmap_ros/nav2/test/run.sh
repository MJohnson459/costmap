#!/usr/bin/env bash
# Integration test of costmap_window_server against a real Nav2 costmap.
#
# Usage: bash test/run.sh
#
# SERVER_RATE_HZ (default 10) sets the window server's --rate-hz and NAV2_HZ
# (default 10) sets Nav2's update_frequency and publish_frequency.
#
# E2E_MAP=/path/cost.tif selects the GeoTIFF mode instead: one phase per
# window size in E2E_WINDOWS_M (default "12 40", metres) on that map, with the
# robot visiting the local-frame poses in E2E_POSES ("x,y x,y ...", required).
# The checker reads the raster itself and compares every cell Nav2 holds, graded
# costs included. Nav2's resolution is taken from the map, and its window is
# NAV2_WINDOW_M (default 8, whole metres). E2E_SERVE_MAP=/path/other.tif serves
# a different raster than the checker reads (negative control).
#
# Starts a Zenoh router, then for each map format (ROS YAML, and a GeoTIFF
# converted from the same fixture) and published window size (12 m, 100 m):
# the checker (which also publishes the robot's map -> base_link tf), the
# window server on the fixture, and a standalone Nav2 costmap with a
# StaticLayer, which it configures and activates. The checker compares Nav2's
# costmap with the fixture, moves the robot, and reports PASS/FAIL and
# latency. Every process is started in its own process group and stopped by
# that group id; nothing else on the machine is signalled.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$NAV2_DIR/../../.." && pwd)"
MAP="$REPO/tests/fixtures/warehouse.yaml"
WINDOWS_M=(12 100)
MAP_KINDS=(yaml tif)
POSES_ARGS=()
CHECK_MAP="$MAP"
SERVE_MAP=""
if [[ -n "${E2E_MAP:-}" ]]; then
    [[ -n "${E2E_POSES:-}" ]] || { echo "E2E_MAP needs E2E_POSES" >&2; exit 2; }
    [[ -f "$E2E_MAP" ]] || { echo "E2E_MAP $E2E_MAP not found" >&2; exit 2; }
    read -r -a WINDOWS_M <<<"${E2E_WINDOWS_M:-12 40}"
    MAP_KINDS=(tif)
    POSES_ARGS=(--poses "$E2E_POSES")
    CHECK_MAP="$E2E_MAP"
    SERVE_MAP="${E2E_SERVE_MAP:-$E2E_MAP}"
fi
NAV2_WINDOW_M="${NAV2_WINDOW_M:-8}"
SERVER_RATE_HZ="${SERVER_RATE_HZ:-10}"
# Nav2 declares both frequencies as doubles and rejects an integer value.
NAV2_HZ="$(printf '%.3f' "${NAV2_HZ:-10}")"
STAGE_TIMEOUT_S=30
CHECK_TIMEOUT_S=180
LOG_DIR="$(mktemp -d -t costmap_ros_e2e.XXXXXX)"
TIF="$LOG_DIR/warehouse.tif"

LOG_TAG=e2e
source "$NAV2_DIR/lib.sh"

log "building costmap_window_server and the GeoTIFF converter"
cargo build -j 6 --release --quiet --manifest-path "$REPO/Cargo.toml" -p costmap_ros \
    --bin costmap_window_server
cargo build -j 6 --release --quiet --manifest-path "$REPO/Cargo.toml" -p costmap-geo \
    --bin costmap-from-ros-map
SERVER="$REPO/target/release/costmap_window_server"
if [[ -z "$SERVE_MAP" ]]; then
    "$REPO/target/release/costmap-from-ros-map" "$MAP" "$TIF" >/dev/null
    log "converted $MAP to $TIF"
fi

enter_ros_env

# Nav2's cells must coincide with the map's for an exact comparison.
RESOLUTION="$(python -u "$HERE/check_costmap.py" --map "$CHECK_MAP" --print-resolution)" ||
    fail "could not read the resolution of $CHECK_MAP"
[[ "$RESOLUTION" == *.* ]] || RESOLUTION="$RESOLUTION.0"
log "map resolution $RESOLUTION m; Nav2 window ${NAV2_WINDOW_M} m"

start_router

results=()
for map_kind in "${MAP_KINDS[@]}"; do
for window_m in "${WINDOWS_M[@]}"; do
    served_map="$MAP"
    [[ "$map_kind" == tif ]] && served_map="${SERVE_MAP:-$TIF}"
    tag="${map_kind}_${window_m}"
    log "=== $map_kind map, published window ${window_m} m ==="
    phase_start=${#started_groups[@]}

    start "check_${tag}" python -u "$HERE/check_costmap.py" --map "$CHECK_MAP" \
        --window-m "$window_m" "${POSES_ARGS[@]}"
    check_pid_var="check_${tag}_pid"
    check_pid="${!check_pid_var}"
    wait_for "tf publisher (checker)" "$STAGE_TIMEOUT_S" log_has "check_${tag}" "tf: publishing"

    RUST_LOG="warn,costmap_window_server=debug,costmap_ros=debug" start "server_${tag}" "$SERVER" \
        --map "$served_map" --window-m "$window_m" --rate-hz "$SERVER_RATE_HZ"
    server_pid_var="server_${tag}_pid"
    wait_for "costmap_window_server first publish" "$STAGE_TIMEOUT_S" \
        log_has "server_${tag}" "published"
    require_alive "server_${tag}" "${!server_pid_var}"
    if [[ "$map_kind" == tif ]] && ! log_has "server_${tag}" "GeoTIFF"; then
        fail "server did not report serving a GeoTIFF"
    fi

    start "nav2_${tag}" ros2 run nav2_costmap_2d nav2_costmap_2d \
        --ros-args --params-file "$HERE/costmap_params.yaml" \
        -p "update_frequency:=$NAV2_HZ" -p "publish_frequency:=$NAV2_HZ" \
        -p "resolution:=$RESOLUTION" -p "width:=$NAV2_WINDOW_M" -p "height:=$NAV2_WINDOW_M"
    wait_for "Nav2 costmap node" "$STAGE_TIMEOUT_S" ros2 lifecycle get --no-daemon /costmap
    for transition in configure activate; do
        # `ros2 lifecycle set` exits 0 when the node rejects a transition.
        output="$(timeout "$STAGE_TIMEOUT_S" ros2 lifecycle set --no-daemon /costmap "$transition" 2>&1)" || true
        printf '%s\n' "$output" >>"$LOG_DIR/lifecycle_${tag}.log"
        if [[ "$output" != *"Transitioning successful"* ]]; then
            grep -E "ERROR|WARN" "$LOG_DIR/nav2_${tag}.log" | grep -v zenoh_config | tail -n 10 >&2 || true
            fail "Nav2 costmap $transition did not succeed within ${STAGE_TIMEOUT_S} s"
        fi
        log "Nav2 costmap: $transition done"
    done

    deadline=$((SECONDS + CHECK_TIMEOUT_S))
    while process_alive "$check_pid"; do
        ((SECONDS < deadline)) || fail "checker did not finish within ${CHECK_TIMEOUT_S} s"
        sleep 0.5
    done
    status=0
    wait "$check_pid" || status=$?
    cat "$LOG_DIR/check_${tag}.log"
    results+=("$map_kind: $(grep -E '^(PASS|FAIL):|^latency' "$LOG_DIR/check_${tag}.log" | tr '\n' ' ')")

    for pgid in "${started_groups[@]:$phase_start}"; do
        stop_group "$pgid"
    done
    ((status == 0)) || fail "checker reported failure for the $map_kind map, ${window_m} m window"
done
done

log "=== summary (server --rate-hz $SERVER_RATE_HZ, Nav2 update/publish $NAV2_HZ Hz) ==="
for line in "${results[@]}"; do
    log "$line"
done
log "PASS (logs: $LOG_DIR)"
