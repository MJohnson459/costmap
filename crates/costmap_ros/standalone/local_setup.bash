# Adds the prefix this file is installed in to the ROS environment already
# sourced in this shell. The file is PREFIX/share/costmap_ros/local_setup.bash.
_costmap_ros_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AMENT_PREFIX_PATH="$_costmap_ros_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
unset _costmap_ros_prefix
