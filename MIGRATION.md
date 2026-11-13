# Migration guide

## 0.2 to 0.3

This section lists every change that breaks or deprecates 0.2 code. The [changelog](CHANGELOG.md) lists what 0.3 adds.

### `Grid2d` is now `DenseGrid`

The crate has a sparse grid, `TiledGrid`, beside the dense one, so the dense one is named for what it is. `Grid2d` remains as a deprecated alias and will be removed in 0.4. The `Costmap` and `OccupancyGrid` aliases are unchanged.

```rust
// 0.2
use costmap::Grid2d;
// 0.3
use costmap::DenseGrid;
```

The module `costmap::grid::grid2d` is now `costmap::grid::dense`.

### `DenseGrid` methods

The old names still compile, with a deprecation warning, and will be removed in 0.4.

| 0.2 | 0.3 |
| --- | --- |
| `Grid2d::new_with_value(info, value)` | `DenseGrid::filled(info, value)` |
| `Grid2d::new_with(info, f)` | `DenseGrid::from_fn(info, f)` |
| `Grid2d::init(info, data)` | `DenseGrid::from_vec(info, data)` |
| `Grid2d::init_with_value(info, data, fill)` | `DenseGrid::from_vec(info, data)?.with_fill(fill)` |
| `grid.width()` | `grid.size().x` |
| `grid.height()` | `grid.size().y` |
| `grid.iter()` | `grid.as_slice().iter()` |
| `grid.iter_mut()` | `grid.as_mut_slice().iter_mut()` |

`with_fill` sets the value that `clear`, `update_origin`, `resize_map` and `reset_map` give to new or reset cells. `filled` uses its value as the fill value; the other constructors use `Default::default`.

### World coordinates are `f64`

Positions in the world frame, map resolution and map extents are now `f64` (`glam::DVec2`). An `f32` resolves about half a metre at a UTM northing of 6,200,000 m, and a rolling grid's origin drifted by 1.8 cells after 1.4 km. Quantities local to a grid or a robot stay `f32`: ray directions, ranges, radii, yaw, costs and cell-space coordinates.

| Item | 0.2 | 0.3 |
| --- | --- | --- |
| `MapInfo::resolution` | `f32` | `f64` |
| `MapInfo::origin` | `Vec2` | `DVec2` |
| `MapInfo::world_width`, `world_height` | `f32` | `f64` |
| `MapInfo::world_center` | `Vec2` | `DVec2` |
| `Pose2::position` | `Vec2` | `DVec2` |
| `Bounds::min`, `max` | `Vec2` | `DVec2` |
| `Footprint::transform` | `Vec<Vec2>` | `Vec<DVec2>` |
| `map_to_world`, `map_to_world_continuous` | return `Vec2` | return `DVec2` |
| `world_to_map`, `world_to_map_continuous` | take `Vec2` | take `DVec2` |
| `resize_map`, `update_origin`, `update_center` | `f32`, `Vec2` | `f64`, `DVec2` |
| `raycast_dda`, `raycast_grid_step`, `line` | `origin: Vec2` | `origin: DVec2` |
| `polygon`, `polygon_value` | `&[Vec2]` | `&[DVec2]` |
| `RosMapMetadata::resolution`, `origin` | `f32`, `Vec3` | `f64`, `DVec3` |

Unchanged: `Pose2::yaw`, `Footprint::points` (robot frame), ray `dir` and `max_t`, `RayHit2D::hit_distance`, `InflationConfig`, and the cell-space `Vec2` that `world_to_map_continuous` returns and `map_to_world_continuous` takes.

Most call sites change a constructor and nothing else:

```rust
// 0.2
let pose = Pose2::new(Vec2::new(5.0, 5.0), 0.0);
// 0.3
let pose = Pose2::new(DVec2::new(5.0, 5.0), 0.0);
```

To pass an `f32` vector where a world position is wanted, call `.as_dvec2()`; to go back, `.as_vec2()`.

### Cell positions are `IVec2`

A cell position is a signed `glam::IVec2`. Sizes and counts of cells stay unsigned: `size()`, `resize_map` and `MapInfo::width`/`height` are unchanged.

| Item | 0.2 | 0.3 |
| --- | --- | --- |
| `get`, `get_mut`, `set`, `get_unchecked`, `get_unchecked_mut`, `grid[cell]` | take `UVec2` | take `IVec2` |
| `map_to_world` | takes `UVec2` | takes `IVec2` |
| `world_to_map` | returns `Option<UVec2>` | returns `Option<IVec2>` |
| `indexed_iter`, `indexed_iter_mut` | yield `(UVec2, _)` | yield `(IVec2, _)` |
| `reset_map` | `lower`, `upper: UVec2` | `lower`, `upper: IVec2` |
| `from_fn` (was `new_with`) | `f(x: u32, y: u32)` | `f(cell: IVec2)` |
| `CellRegion::min`, `max`, `new`, `contains` | `UVec2` | `IVec2` |
| `LineStep::cell` | `UVec2` | `IVec2` |
| `PolygonIterator` | yields `UVec2` | yields `IVec2` |
| `RayHit2D::cell` | `UVec2` | `IVec2` |

The deprecated `new_with` still passes `(x, y)` as `u32`.

The planning API, new in 0.3, uses `IVec2` for every cell: `plan`'s `start` and `goal`, `Path::cells`, `Path::shortcut`, `Move::cell`, `moves`, `passable_cost`, `line_passable`, `Regions::region` and `Regions::cells`. `TileGraph::plan`, `TileGraph::shortcut` and `TileCache::cost` do the same. Tile coordinates are positions too: `TileSource::tile` takes an `IVec2`, and `TileLayout::tile_of` and `tile_min` return one. `TileLayout::tile_size`, `tiles` and `size` stay `UVec2`.

A cell outside the grid, including one with a negative coordinate, gets `None` from `get`. `DenseGrid::contains(cell)` tests the bounds alone.

```rust
// 0.2
let grid = Costmap::new_with(info, |x, y| if x == 5 { COST_LETHAL } else { COST_FREE });
let cost = grid.get(UVec2::new(5, 2));
// 0.3
let grid = Costmap::from_fn(info, |cell| if cell.x == 5 { COST_LETHAL } else { COST_FREE });
let cost = grid.get(IVec2::new(5, 2));
```

### `LayeredCostmap::add_layer` returns a `Result`

`add_layer` calls the new `Layer::attach` and returns its error, so a layer that cannot work with the costmap is rejected when it is added.

```rust
// 0.2
let id = layered.add_layer(Box::new(layer));
// 0.3
let id = layered.add_layer(Box::new(layer))?;
```

Existing `Layer` implementations compile unchanged: `attach` has a default that accepts.

### `VoxelError` is non-exhaustive

A `match` on `VoxelError` needs a wildcard arm. The enum gains `InvalidArgument`, `Unsupported` and, with the `geotiff` feature, `Tiff`.

### Removed

- The `viewer` feature and its `pixels` and `winit` dependencies. No code used it.

### Dependencies

- glam 0.30 to 0.33. Types such as `Vec2` in this crate's API are glam's, so a dependent crate must use the same glam version.
- rerun 0.33 to 0.34, behind the `rerun` feature.
