# Agents

`costmap` is a costmap library in Rust. One set of concepts covers everything from a
10 m window round a robot to a map the size of a farm. The first user is a team with
a large site: `crates/costmap_ros` builds `tiled_map_server`, a replacement for
Nav2's `map_server` on Zenoh. The library is meant to outperform the C++ costmap at
large map sizes. Keep it small enough for the maintainer to understand all of it.

3D and voxel grids are not goals. Algorithms are not generic over storage.

## Concepts

- **Lattice.** An anchor (`DVec2`, metres) and a resolution. It alone converts world
  positions to cells (`cell_at`, `cell_center`).
- **Cell.** One lattice square, named by an `IVec2` from the lattice's anchor.
  Extents of cells are `UVec2`. World positions are `f64`.
- **Window.** A dense rectangle cut from a map: a `DenseGrid` on the map's lattice.
  `CostWindows` returns one from any map: `DenseGrid`, `TiledGrid` or a cost GeoTIFF.
- **Chunk.** The storage unit of `TiledGrid`.
- **Tile.** The planning unit of `TileGraph`.
- A TIFF strip or tile is a **block**, GDAL's term, so that neither word above is
  overloaded.

Use these words for these things only, in code and in prose. The error type is
`CostmapError`.

Algorithms (inflation, A*, raycasting, footprint checks) take a `DenseGrid`.
Whole-map operations are library drivers that walk tiles or chunks. `costmap_ros`
holds no algorithms: it converts messages, tracks pose, decides when to publish and
serves the route action.

## Code conventions

`src/grid/dense.rs` is the reference.

- Public methods take and return cells of the grid's lattice.
  - `cells()` is the rectangle the grid holds.
  - `as_slice()` is row-major from its first cell.
- Hot loops work in array offsets and convert once where they meet the caller:
  - `offset_of`, `cell_of`, `at`, `at_mut`, `at_held`, `to_offsets`, `offsets_in`;
  - `LineIterator::next_offset`, `planning::astar::moves_at`.
- An operation that takes or returns cells gets a case in `tests/moved_grid.rs`: it
  gives the same answer on a moved grid in shifted cells.

## Working rules

- One kind of change per PR, applied everywhere it occurs. No module-by-module splits.
- No version bumps: `cargo release` does them.
- No design documents in the repository.
- `v0.1.0` and `v0.2.0` are released. To check whether a name was released, run
  `git grep -n NAME v0.2.0 -- src`.
  - Released, and only renamed: keep the old name as a type alias or one-line
    forwarding method, `#[deprecated(since = "0.3.0", note = "...")]`, with a note
    naming the replacement. Add a use of it to `tests/deprecated.rs`.
  - Released, but would need old machinery kept alive: delete it.
  - Not released: change it freely.
- Every public change goes in `CHANGELOG.md` under "Unreleased". A change that
  breaks or deprecates released code also goes in `MIGRATION.md`, old name to new.
- Performance changes and structural changes carry criterion numbers from `main`
  and the branch in the PR. A regression blocks the PR.
  - Name every bench: `cargo bench -p costmap --features geotiff --bench NAME`.
    Save `-- --save-baseline main` on `main`, then compare with
    `-- --baseline main` on the branch.
  - Noise is about ±3 %. Confirm a smaller difference with a second run, and with
    `CARGO_PROFILE_BENCH_CODEGEN_UNITS=1`.
- Run heavy commands under `nice -n 10` with `-j 6`.

## Done when

```sh
cargo fmt --check
cargo +1.99 clippy --workspace --all-targets --features costmap/rerun,costmap/geotiff -- -D warnings
cargo test --workspace --features costmap/geotiff
RUSTDOCFLAGS="-D warnings" cargo +1.99 doc --workspace --no-deps --features costmap/rerun,costmap/geotiff
crates/costmap_ros/nav2/test/colcon.sh
crates/costmap_ros/nav2/test/run.sh
```

## References

- [Nav2 costmap_2d](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d)
- [Nav2 map_io](https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/src/map_io.cpp): loading a ROS map (YAML and image).
- [A Fast Voxel Traversal Algorithm for Ray Tracing](http://www.cse.yorku.ca/~amana/research/grid.pdf): the DDA raycast.
- [Ray casting in 2D grids](https://theshoemaker.de/posts/ray-casting-in-2d-grids): a comparison of methods.
