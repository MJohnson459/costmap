# Agents

The goal of this repo is to create a generic implementation of a Voxel Grid that
we can load from multiple formats.

I want to support both 2D Occupancy Grid and a full 3D Voxel Grid. Both should
be generic over the application if possible, but must support the functions
required by robotics and ROS.

An Occupancy Grid is a 2D grid where each cell has a probability value. There
are different modes this can operate in, but the default is "Trinary" where a
cell is either `-1` for unknown, `0` for clear, or `100` for occupied.

## Architecture

We want to have some core structs (VoxelGrid, VoxelError) and then traits that
implement algorithms on it. We should avoid excessive use of generics but I
would weigh up each case depending on the impact on the useability of the API.

We will also want to support loading the grid from multiple formats. This
implies we will have an extensible builder and a trait so we can have an
implementation for different file formats.

A big emphasis will be on performance so we will need to properly instrument the
code.

We should plan for different types of users and use cases.

- Offline processing and editing of maps.
- Online creation of maps
- Visualisation of the maps

## Technology

- Rust will be the primary language.
- We will need to use the `image` crate to load a 2D grid from an image.
- `criterion` for benchmarking
- github actions for CI
- glam for basic maths types.

## Reference Links

- [ROS2 voxel grid impl](https://github.com/ros-navigation/navigation2/blob/main/nav2_voxel_grid/src/voxel_grid.cpp)
- [ROS2 Voxel grid header](https://github.com/ros-navigation/navigation2/blob/main/nav2_voxel_grid/include/nav2_voxel_grid/voxel_grid.hpp)
- [ROS2 Voxel utils](https://github.com/ros-navigation/navigation2/blob/main/nav2_util/include/nav2_util/occ_grid_utils.hpp)
- https://github.com/ros-navigation/navigation2/blob/088c423deb97a76f5a5f4ca133cb122338576fe1/nav2_map_server/src/map_io.cpp#L236 implemenation of 2D map loading for ROS2 Occupancy Grid (yaml metadata + image file)
- [A Fast Voxel Traversal Algorithm for Ray Tracing](http://www.cse.yorku.ca/~amana/research/grid.pdf) which describes a fast method of ray casting on a grid.
- https://theshoemaker.de/posts/ray-casting-in-2d-grids Comparison of different ray casting methods.
- https://vercidium.com/blog/optimised-voxel-raymarching/ Implementation of ray casting for 3D voxels stored ini chunks.
- [Octomap](https://octomap.github.io/) A C++ implemenation of an efficient Voxel grid.
