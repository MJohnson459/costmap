# Migration guide

## 0.2 to 0.3

Version 0.3 is not released yet. This section lists every change on `main` that breaks or deprecates 0.2 code.

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
