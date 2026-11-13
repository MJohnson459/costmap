# Changelog

Breaking changes have their own guide: [MIGRATION.md](MIGRATION.md).

## Unreleased

### Added

- **Planning.** `plan`, an 8-connected, cost-weighted A* with no corner cutting; `Path::shortcut`, `line_passable` and `Regions`.
- **`TileGraph`.** Plans across a map a tile at a time. On a 30.5 M-cell map it plans corner to corner in 28 ms where `plan` takes 2,270 ms, at most 0.9 % costlier, and reads only the tiles on the route. `write_to` and `read_from` store the graph.
- **Tiled maps.** `TiledGrid<T>` stores a map as sparse chunks; `TiledMapLayer` stamps the part under a rolling costmap.
- **Cost GeoTIFFs** (feature `geotiff`). `CostGeoTiff` reads the chunks a region needs; `write_cost_geotiff` writes them with max-pooled overviews; `GeoReference` places them.
- **`ObstacleLayer`.** Marks sensor hits, clears along beams and forgets obstacles after a set time.
- **`cost_from_occupancy`, `occupancy_from_cost`, `costmap_from_occupancy`.** Nav2's `StaticLayer` rule and its inverse.
- **`costmap_ros`.** A ROS 2 package for Nav2 on `rmw_zenoh_cpp`. `costmap_window_server` takes the place of `nav2_map_server`: it publishes a window of the map around the robot and plans routes across the whole map for Nav2's planner to follow. `costmap-from-ros-map` converts a ROS map to a cost GeoTIFF. It installs from a Debian package, with colcon, or as a conda package.

### Changed

- `Grid2d` is `DenseGrid`, with shorter constructor names. The old names are deprecated.
- World positions, resolutions and extents are `f64`.
- `LayeredCostmap::add_layer` returns a `Result`: a layer can reject a costmap it cannot work on.
- `VoxelError` is non-exhaustive.
- Grids are logged to Rerun as `GridMap` archetypes.

## 0.2.0

Raycasting, inflation, layered costmaps, ROS map loading and Rerun visualisation.
