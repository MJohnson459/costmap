#!/usr/bin/env bash
# Builds the costmap_ros programs and lays them out as a ROS install prefix,
# with only a Rust toolchain: no ROS install and no colcon.
#
# Usage: bash package.sh OUT_DIR
#
# OUT_DIR/costmap_ros holds what `colcon build` installs: the programs under
# lib/costmap_ros, the launch file and behaviour tree under share/costmap_ros,
# and the ament index entry `ros2 run` and `ros2 launch` find the package by.
# OUT_DIR/costmap_ros-VERSION-ARCH.tar.gz is the same tree, for a robot:
#
#   tar -xzf costmap_ros-VERSION-ARCH.tar.gz -C /opt
#   source /opt/costmap_ros/setup.bash
set -euo pipefail

[[ $# == 1 ]] || { echo "usage: bash package.sh OUT_DIR" >&2; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
mkdir -p "$1"
OUT="$(cd "$1" && pwd)"
PREFIX="$OUT/costmap_ros"
PROGRAMS=(costmap_window_server costmap_planner_server costmap-from-ros-map)
VERSION="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$HERE/package.xml")"
ARCH="$(uname -m)"

# The workspace's release profile keeps debug information, which makes each
# binary 20 times larger.
CARGO_PROFILE_RELEASE_DEBUG=false CARGO_PROFILE_RELEASE_STRIP=true \
    cargo build --release --locked --manifest-path "$REPO/Cargo.toml" \
    --package costmap_ros --package costmap-geo "${PROGRAMS[@]/#/--bin=}"

# With a target named, as conda's Rust toolchain does, cargo adds a directory.
BUILT="$REPO/target/${CARGO_BUILD_TARGET:+$CARGO_BUILD_TARGET/}release"

rm -rf "$PREFIX"
mkdir -p "$PREFIX/lib/costmap_ros" "$PREFIX/share/costmap_ros" \
    "$PREFIX/share/ament_index/resource_index/packages"
for program in "${PROGRAMS[@]}"; do
    install -m 755 "$BUILT/$program" "$PREFIX/lib/costmap_ros/"
done
cp -r "$HERE/launch" "$HERE/behavior_trees" "$HERE/package.xml" "$PREFIX/share/costmap_ros/"
touch "$PREFIX/share/ament_index/resource_index/packages/costmap_ros"
cat >"$PREFIX/setup.bash" <<'EOF'
# Adds this prefix to the ROS environment already sourced in this shell.
_costmap_ros_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="$_costmap_ros_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
unset _costmap_ros_prefix
EOF

tar -czf "$OUT/costmap_ros-$VERSION-$ARCH.tar.gz" -C "$OUT" costmap_ros
echo "$OUT/costmap_ros-$VERSION-$ARCH.tar.gz"
