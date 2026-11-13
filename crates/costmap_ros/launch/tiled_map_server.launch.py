"""Serves a map window and routes across the whole map, from one process.

    ros2 launch tiled_map_server tiled_map_server.launch.py map:=/maps/site.yaml

tiled_map_server takes command-line options, not ROS parameters, so it is
started as a process. `ros2 run tiled_map_server tiled_map_server --help`
lists every option.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

ARGUMENTS = [
    ("map", None, "Cost GeoTIFF (.tif) or ROS map (.yaml)."),
    ("window_m", "50.0", "Side of the published map window, metres."),
    ("map_topic", "map", "Topic the window is published on."),
    ("map_frame", "map", "Frame of the map."),
    ("base_frame", "base_link", "Robot frame the window follows."),
    ("pose_topic", "", "PoseWithCovarianceStamped topic to take the robot pose from instead of /tf."),
    ("robot_radius_m", "0.3", "Robot radius routes keep clear of obstacles by, metres."),
    ("inflation_m", "1.5", "Distance over which routes are pushed from obstacles, metres."),
    ("routes", "true", "Whether to plan routes across the whole map as well."),
]


def node(name, *options, condition=None):
    executable = PathJoinSubstitution(
        [FindPackagePrefix("tiled_map_server"), "lib", "tiled_map_server", name]
    )
    return ExecuteProcess(
        cmd=[executable, *options], name=name, output="screen", condition=condition
    )


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]
    value = LaunchConfiguration
    window = [
        "--map", value("map"),
        "--window-m", value("window_m"),
        "--topic", value("map_topic"),
        "--map-frame", value("map_frame"),
        "--base-frame", value("base_frame"),
        "--pose-topic", value("pose_topic"),
    ]
    routes = [
        "--routes",
        "--robot-radius-m", value("robot_radius_m"),
        "--inflation-m", value("inflation_m"),
    ]
    # One process serves both, on one /tf subscription.
    with_routes = node(
        "tiled_map_server", *window, *routes, condition=IfCondition(value("routes"))
    )
    window_only = node(
        "tiled_map_server", *window, condition=UnlessCondition(value("routes"))
    )
    return LaunchDescription([*declared, with_routes, window_only])
