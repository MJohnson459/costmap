# Process and environment helpers shared by run_e2e.sh and run_drive.sh.
# The sourcing script sets HERE, LOG_DIR, LOG_TAG and STAGE_TIMEOUT_S.

started_groups=()

log() { printf '[%s] %s\n' "$LOG_TAG" "$*"; }

fail() {
    log "FAIL: $*"
    log "logs: $LOG_DIR"
    exit 1
}

# Runs a command in a new session, so its process id is also the id of a
# process group holding everything it spawns (ros2 run starts a child).
start() {
    local name="$1"
    shift
    setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
    local pid=$!
    started_groups+=("$pid")
    log "started $name (pgid $pid), log $LOG_DIR/$name.log"
    eval "${name}_pid=$pid"
}

stop_group() {
    local pgid="$1"
    kill -0 -- "-$pgid" 2>/dev/null || return 0
    kill -INT -- "-$pgid" 2>/dev/null || true
    for _ in $(seq 50); do
        kill -0 -- "-$pgid" 2>/dev/null || return 0
        sleep 0.1
    done
    kill -KILL -- "-$pgid" 2>/dev/null || true
}

cleanup() {
    local pgid
    for pgid in "${started_groups[@]}"; do
        stop_group "$pgid"
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# Polls `$@` once every 0.2 s until it succeeds or `timeout` seconds pass.
wait_for() {
    local what="$1" timeout="$2"
    shift 2
    local deadline=$((SECONDS + timeout))
    until "$@" >/dev/null 2>&1; do
        if ((SECONDS >= deadline)); then
            fail "$what did not come up within ${timeout} s"
        fi
        sleep 0.2
    done
    log "$what is up"
}

port_in_use() { (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; }

process_alive() { kill -0 "$1" 2>/dev/null; }

log_has() { grep -q -- "$2" "$LOG_DIR/$1.log"; }

# Fails with the end of the process's log if it has exited.
require_alive() {
    local name="$1" pid="$2"
    if ! process_alive "$pid"; then
        tail -n 30 "$LOG_DIR/$name.log" >&2 || true
        fail "$name exited"
    fi
}

pick_port() {
    local port
    for port in $(seq 7447 7547); do
        if ! port_in_use "$port"; then
            echo "$port"
            return 0
        fi
    done
    return 1
}

# Puts the pixi environment on PATH and selects rmw_zenoh on a private domain.
enter_ros_env() {
    # The environment's activation scripts read unset variables.
    set +u
    eval "$(pixi shell-hook --manifest-path "$HERE/pixi.toml" --shell bash)"
    set -u
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID=73
}

# Starts a Zenoh router on a free port and points every later node at it.
start_router() {
    PORT="$(pick_port)" || fail "no free TCP port in 7447-7547 for the Zenoh router"
    [[ "$PORT" == 7447 ]] || log "port 7447 is in use; the router listens on $PORT instead"

    ZENOH_CONFIG_OVERRIDE="listen/endpoints=[\"tcp/127.0.0.1:$PORT\"]" \
        start router ros2 run rmw_zenoh_cpp rmw_zenohd
    wait_for "Zenoh router on port $PORT" "$STAGE_TIMEOUT_S" port_in_use "$PORT"
    # Every later ROS node, rclpy and the Rust server read this.
    export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/127.0.0.1:$PORT\"]"
}
