# Changelog

Breaking changes have their own guide: [MIGRATION.md](MIGRATION.md).

## Unreleased

### Added

- **Planning.** `plan`, an 8-connected, cost-weighted A* with no corner cutting; `Path::shortcut`, `line_passable` and `Regions`.
- **`TileGraph`.** Plans across a map a tile at a time. On a 30.5 M-cell map it plans corner to corner in 28 ms where `plan` takes 2,270 ms, at most 0.9 % costlier, and reads only the tiles on the route. `write_to` and `read_from` store the graph.
- **Tiled maps.** `TiledGrid<T>` stores a map as sparse chunks.
- **Cost GeoTIFFs** (feature `geotiff`). `CostGeoTiff` reads the chunks a region needs; `write_cost_geotiff` writes them with max-pooled overviews; `GeoReference` places them.
- **`CostWindows`.** One request for every cost map: the cells of a `CellRegion` as a dense window on the map's lattice, unknown where the map holds nothing. `Costmap`, `TiledGrid<u8>` and `GeoTiffWindows`, a cost GeoTIFF that holds only the chunks near the last window, implement it. `CellRegion::centered` and `CellRegion::grown` build the regions.
- **`resample`.** Copies a window of any `CostWindows` map onto another lattice, given the pose of the map's frame in the destination's: another resolution, a sub-cell offset, or another frame such as `map` under `odom`. `Resample::WorstCost`, the default onto coarser cells, loses no obstacle and narrows free space by up to one cell per edge; `Resample::Nearest`, the default otherwise, keeps widths and can lose an obstacle smaller than a cell. Unknown loses to any known cost.
- **`StaticLayer<M>`.** The static-map layer over any `CostWindows` map, dense, tiled or on disk. It copies the cells under the master where the two share a lattice and frame, and resamples them otherwise; `set_transform` takes the map's pose in the master's frame before each update. It replaces `TiledMapLayer`.
- **`RoutePlanner`.** Plans a route between world positions across a whole map, cost GeoTIFF or ROS map, on a `TileGraph` stored beside the map and read back on the next start. An endpoint on an impassable cell snaps to the nearest passable one. `InflatedWindows` inflates each window of any `CostWindows` map with the margin it needs; `CostWindows::window_around` cuts the window around a position. All three moved here from `costmap_ros`, which keeps message conversion, pose tracking, publishing decisions and the route action server.
- **`MapFile`.** Opens a map by extension: `.yaml`/`.yml` as a ROS map held in memory, `.tif`/`.tiff` (feature `geotiff`) as `GeoTiffWindows` with the chunk size and margins a `GeoTiffReading` gives. It implements `CostWindows`. `RoutePlanner::open` and `tiled_map_server` both open maps through it.
- **`ObstacleLayer`.** Marks sensor hits, clears along beams and forgets obstacles after a set time.
- **`cost_from_occupancy`, `occupancy_from_cost`, `costmap_from_occupancy`.** Nav2's `StaticLayer` rule and its inverse.
- **`tiled_map_server`.** A ROS 2 package for Nav2 on `rmw_zenoh_cpp`, built from the `costmap_ros` crate. Its node takes the place of `nav2_map_server`: it publishes a window of the map around the robot and plans routes across the whole map for Nav2's planner to follow. `costmap-from-ros-map` converts a ROS map to a cost GeoTIFF. It installs from a Debian package, with colcon, or as a conda package.

### Changed

- `Grid2d` is `DenseGrid`, with shorter constructor names.
- World positions, resolutions and extents are `f64`.
- Cell positions are `IVec2`; sizes and counts of cells stay `UVec2`.
- `LayeredCostmap::add_layer` returns a `Result`: a layer can reject a costmap it cannot work on.
- `ProjectionLayer` resamples a source on another lattice onto the master's cells instead of rejecting it.
- `VoxelError` is non-exhaustive.
- World-to-cell conversion goes through `Lattice`, from `DenseGrid::lattice` or `TiledGrid::lattice`. `cell_at`, `cell_center`, `cell_corner`, `to_cells` and `from_cells` replace `DenseGrid::world_to_map`, `map_to_world` and their `_continuous` forms, and `TiledGrid::world_to_cell`, `cell_origin` and `cell_center`.
- A `DenseGrid` keeps its lattice and holds a rectangle of its cells, `cells()`. A grid moved by `update_origin` or `update_center`, and a window from `TiledGrid::window`, name each cell as the grid they came from does; before, their first cell was always (0, 0).
- `Layer::attach` takes the master grid, and `Layer::match_size` is removed.
- `MapInfo` and `DenseGrid::info()` are removed. `DenseGrid`'s constructors, `LayeredCostmap::new` and `ObstacleLayer::new` take a `Lattice` and the `CellRegion` of cells to hold. `CellRegion::from_size` and `CellRegion::sized` build the rectangle; `DenseGrid::world_bounds` gives the world rectangle a grid covers.
- Grids are logged to Rerun as `GridMap` archetypes.
- `TileGraph::build` and `TileCache` read tiles through `CostWindows`; `TileSource` and `DenseTiles` are removed. A graph covers any `CellRegion` of a map and keeps its `Lattice`, so it plans on a moved grid. The stored format is version 2.
- `TiledGrid::window` takes a `CellRegion`; `window_centered` is removed.
- `costmap_ros` reads every map through `CostWindows`: `MapSource`, `SourceError` and `TileStats` are removed, and `CostWindows::window_around` cuts the published window.
- `RoutePlanner`, `RouteOptions`, `Route`, `RouteError`, `window_around` and the `plan_route` example moved from `costmap_ros` to `costmap`. `RoutePlanner::origin` reports whether the graph was read or built where the node logged it.

## 0.2.0

Raycasting, inflation, layered costmaps, ROS map loading and Rerun visualisation.
