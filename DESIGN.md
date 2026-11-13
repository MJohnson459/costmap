# Design

One costmap library for a robot on Rust and Zenoh: the same operations on a
10 m local window and a 2 km field map.

This file states the design the code is held to. Where the code differs, the
code is wrong or this file is; fix one of them.

## Decisions

### 1. A cell names one place, whichever grid holds it

- A `Lattice` is an anchor (`DVec2`, metres) and a resolution. It converts
  between world positions and cells, and is the only thing that does.
- A cell is an `IVec2` counted from the lattice's anchor. Positions are
  `IVec2`; extents are `UVec2`.
- Every grid has a lattice. A `DenseGrid` also has the rectangle of cells it
  holds. Its public API takes cells, not array offsets.
- A window cut from a map shares the map's lattice, so a cell means the same
  place in both. A rolling grid keeps its lattice and moves by whole cells;
  only its rectangle changes.
- Grids on different lattices do not share cells: another resolution, an
  offset of part of a cell, or another frame such as `odom` against `map`.
  `resample` copies a window onto another lattice given the rigid transform
  between their frames, by one of two policies:
  - worst cost: a destination cell takes the highest cost among the source
    cells it touches. No obstacle is lost; free space narrows by up to a cell
    at each edge. The default when destination cells are coarser.
  - nearest: a destination cell takes the cost of the source cell under its
    centre. Widths are kept; an obstacle thinner than a cell can be lost. The
    default at equal resolution.
  - Under either, unknown loses to any known cost.
- The library has no transform tree. The caller supplies transforms.

Example: on a 0.05 m map, the position (120.30, -4.10) m is cell (2406, -82)
in a GeoTIFF of the whole site, in the 50 m window cut around the robot, and
in that window after it has rolled 5 m.

### 2. Two storages, one way to read them

- `DenseGrid` is bounded and contiguous. `TiledGrid` is unbounded and sparse,
  and may be filled from a file on demand.
- Both, and a raster on disk, answer one request: the cells of a rectangle as
  a dense window.

### 3. Algorithms run on dense windows; the library fetches the window

- Inflation, A*, raycasting and footprint checks take a `DenseGrid`.
- An operation over a whole large map is a library driver that walks it a
  tile at a time and reads the margin each tile needs.
- No algorithm is generic over storage in its inner loop.

### 4. One planner across sizes

- `TileGraph` plans on any map. A* is its in-tile step, and is public for
  planning on a window.
- A path is a list of cells in the map's lattice.

### 5. The live costmap is the same thing on a window

- A `LayeredCostmap`'s master is a dense window.
- Its static layer takes any map of decision 2. On the master's lattice it
  copies. Otherwise it resamples, with the transform the caller supplies at
  each update: a local costmap in `odom` can hold the static map.
- Obstacle, inflation and projection layers write to the master.

### 6. `costmap_ros` holds no algorithms

- It converts messages, tracks the robot pose, decides when to publish, and
  serves the route action.
- Anything a Rust program without ROS would also need is in the library.

### 7. One vocabulary

| Word | Meaning |
| --- | --- |
| cell | one square of a lattice, an `IVec2` |
| window | a dense rectangle of cells cut from a map |
| chunk | the unit a `TiledGrid` stores |
| tile | the unit `TileGraph` plans on |

The error type is `CostmapError`.

## Not goals

- 3D or voxel grids.
- Algorithms generic over storage.

## Later

Not designed yet, and not ruled out. Nothing above should make them harder.

- Rerouting around sensed obstacles: updating the tiles of a `TileGraph` that
  an obstacle touches.
- Planning for a non-circular footprint across a whole map.
