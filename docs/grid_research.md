# Modern alternatives to nav2-style costmaps for robot path planning

**Euclidean Distance Transforms and incremental distance fields represent the most impactful upgrade over nav2's Brushfire inflation, providing exact distances in O(N) time with O(k) incremental updates — while grid-based costmaps themselves remain the dominant production representation that no alternative has displaced.** The key insight from this research is that the biggest wins come not from abandoning grids but from (1) replacing the inflation algorithm with superior distance transforms, (2) pairing grids with modern planners like JPSW or Lazy Theta*, and (3) supporting incremental updates for dynamic environments. For a Rust library, this means building a layered architecture: a flat grid with cache-friendly memory layout, an EDT/SDF distance field, incremental update machinery, and pluggable planners.

---

## Distance transforms vastly outperform Brushfire inflation

Nav2's inflation layer uses a Brushfire algorithm — a multi-source BFS that propagates wavefronts from obstacle cells, approximating distances via 8-connected grid steps. This produces quasi-Euclidean distances with directional bias and accumulating error. The **Felzenszwalb & Huttenlocher EDT** algorithm computes *exact* squared Euclidean distances in **O(N) time** (linear in total cells) by decomposing the 2D problem into two separable 1D passes using a "marching parabolas" technique. For a 1000×1000 grid, this takes roughly **1–3ms on a single core**.

The advantages are substantial. True Euclidean distances produce **smooth, well-defined gradients** everywhere (except at equidistant points), which is critical for trajectory optimization methods like CHOMP and TrajOpt. Manhattan/Chebyshev distances from Brushfire produce piecewise-constant gradients with discontinuities. Euclidean distances are also rotationally invariant — no directional bias means the robot doesn't prefer certain heading angles due to grid artifacts.

**Signed Distance Fields (SDFs)** extend this further by encoding both interior (negative) and exterior (positive) distances. Computing an SDF is trivial with F&H: run the EDT on the original occupancy grid for exterior distances, run it again on the inverted grid for interior distances, then subtract. SDFs support set operations (union = min, intersection = max) for modular obstacle composition and provide smooth cost functions that penalize obstacle penetration — exactly what trajectory optimizers need.

For **dynamic environments**, full EDT recomputation is fast but wasteful when only a few cells change. The solution is **incremental distance field updates** based on the work of Boris Lau, Christoph Sprunk, and Wolfram Burgard (Robotics and Autonomous Systems, 2013). Their approach maintains closest-obstacle pointers per cell and uses raise/lower wavefront queues: when obstacles are added or removed, a "raise" phase invalidates affected cells and a "lower" phase propagates new distances, visiting only the **O(k) cells actually affected** rather than the full grid. FIESTA (Han et al., IROS 2019) improved on this with dual independent queues and achieved **~4× faster updates than Voxblox** with **~10× better accuracy**. The practical pattern for a Rust library is: use F&H EDT for initial full computation, then switch to Lau/FIESTA-style incremental updates for sensor-driven changes.

| Algorithm | Complexity | Distance type | Incremental? |
|-----------|-----------|---------------|-------------|
| Brushfire (nav2) | O(N·k), k = inflation radius | Quasi-Euclidean | No (recomputes layer) |
| F&H EDT | **O(N)** linear | Exact squared Euclidean | No (full recompute) |
| Lau/Dynamic Brushfire | O(N) initial, **O(k) update** | True Euclidean | Yes — raise/lower queues |
| FIESTA | O(k) update | Full Euclidean | Yes — dual queues, 4× faster than Voxblox |
| nvblox (NVIDIA) | GPU-parallel | Full Euclidean | Yes — Parallel Banding Algorithm |

For the Rust implementation, the F&H algorithm maps beautifully to cache-friendly row-major arrays and parallelizes trivially across rows with `rayon`. The horizontal pass processes consecutive memory locations; for the vertical pass, transpose the grid first (SIMD-acceleratable with 4×4 block transposes), then do another horizontal pass. Existing Rust crates include `distance-transform` (F&H port) and `edt` (Saito's algorithm), though both are minimal and a production library would want a custom implementation with SIMD acceleration and incremental update support.

---

## Planners beyond A*: JPS, Lazy Theta*, and JPSW change the game

Standard A* on an inflated costmap is the nav2 baseline, but modern planners offer dramatic speedups depending on the grid type. The most impactful algorithms to consider fall into three categories.

**Jump Point Search (JPS)** exploits path symmetry on uniform-cost (binary) grids. Instead of expanding every neighbor, JPS recursively scans in each direction until hitting obstacles or "forced neighbors" (turning points), pruning symmetric paths. This produces **10–100× speedup over A*** on typical maps — with no preprocessing and no extra memory. JPS+ adds offline preprocessing of jump point distances for **over 100× speedup**, at the cost of 8 bytes per cell. For a costmap where cells are thresholded to binary (free/occupied after applying an inflation radius), JPS is an enormous win.

**Weighted JPS (JPSW)**, published at AAAI 2023 by Carlson, Harabor, and Stuckey, is arguably the most important recent algorithm for costmap-based planning. It extends JPS to **weighted (non-uniform cost) grids** for the first time, using an "orthogonal-last" tiebreaking policy and small neighborhood Dijkstra searches. JPSW achieves **orders of magnitude speedup over A*** on weighted grids — directly applicable to costmaps with inflation gradients. This is the algorithm most worth implementing for a general-purpose costmap library.

**Lazy Theta*** provides any-angle paths (not constrained to 8 grid directions) by optimistically assuming line-of-sight between non-adjacent nodes, verifying only when a node is expanded. It's simple to implement (about 70 lines beyond A*), produces paths that are **~8% shorter** than 8-connected A*, and is already integrated into ROS2 nav2. For a library offering multiple planners, Lazy Theta* provides the best quality/complexity tradeoff for any-angle paths.

For **incremental replanning**, D* Lite remains the standard. It runs A* backward from goal to start and maintains both `g` and `rhs` values per node, enabling efficient repair when local map changes are detected near the robot. The practical caveat is that **if more than ~10–20% of the map changes, full replanning with A* is faster** — D* Lite's overhead (2× per-node memory, two-key priority queue, complex state management) only pays off for small, localized sensor updates in partially-known terrain. Most production systems (including nav2 itself) use repeated A* because modern CPUs are fast enough and costmaps change frequently enough that incremental repair rarely provides net benefit.

**Anytime algorithms** deserve consideration for time-constrained scenarios. Weighted A* (`f = g + w·h`, w > 1) is trivially implemented — just multiply the heuristic — and produces bounded-suboptimal solutions (cost ≤ w × optimal) with dramatically fewer expansions. ARA* extends this by iteratively decreasing w, reusing previous search work. For a Rust library, offering a weight parameter on A* is nearly free and provides users enormous flexibility.

| Planner | Grid type | Speed vs A* | Path quality | Dynamic maps | Impl. complexity |
|---------|-----------|-------------|-------------|-------------|-----------------|
| A* | Any weighted | 1× baseline | Grid-optimal (8-dir) | Replan from scratch | Low (~200 LOC) |
| Weighted A* | Any weighted | 2–10× faster | ≤ w × optimal | Replan from scratch | Trivial (+5 LOC) |
| JPS | Binary only | **10–100×** | Grid-optimal | Replan (no preprocess) | Medium (~400 LOC) |
| JPSW | **Weighted** | **10×+** | Grid-optimal | Replan from scratch | Hard (~800 LOC) |
| Lazy Theta* | Binary | ~1× | Any-angle, near-optimal | Replan from scratch | Low-Medium (~250 LOC) |
| D* Lite | Any weighted | 1–4× for incremental | Grid-optimal | **Incremental repair** | Medium-High (~350 LOC) |
| ARA* | Any weighted | Fast initial, improves | Bounded suboptimal → optimal | Static (AD* for dynamic) | Medium (~300 LOC) |

---

## Grid costmaps persist because nothing else matches their integration story

A key finding of this research is that continuous-space and alternative representations **do not replace grid costmaps** — they consume them. CHOMP requires a signed distance field derived from a voxelized grid. TrajOpt can work with polygon-based obstacles via GJK/EPA collision checking, making it the most "costmap-free" approach, but this only works when obstacles have clean geometric representations. **MPPI in ROS2 nav2 directly uses the 2D occupancy grid costmap** through obstacle critic plugins. The dominant hybrid architecture in production is: grid costmap for global planning → continuous optimizer (MPPI, DWA, TEB) for local trajectory execution.

Vector-based representations (visibility graphs, Voronoi diagrams, polygon obstacles) excel when obstacle count is small and geometry is well-defined — CAD models of warehouses, structured indoor environments. Their complexity scales with **obstacle count** rather than map area, which is favorable for sparse environments. But they struggle with raw sensor data: fitting polygons to LiDAR point clouds is lossy and computationally expensive. Grid costmaps win here because sensor data maps directly to cell updates with no intermediate processing.

Potential field methods have an elegant mathematical foundation. **Navigation functions** (Rimon & Koditschek, 1992) provably eliminate local minima, and **harmonic potential fields** (solving Laplace's equation) guarantee no interior extrema. But both require expensive computation (PDE solving on the grid) and in practice no major production system has replaced grid-based planning with potential fields for primary navigation. They remain useful as secondary cost layers or theoretical analysis tools.

**Neural/learned costmap representations** are the most active research area but furthest from production readiness. Neural A* (Yonetani et al., ICML 2021) uses CNNs to learn heuristic costs, reducing A* node expansions by up to 4×. Tesla's Occupancy Network replaces traditional costmaps with a learned 3D voxel representation. But these approaches require training data per environment type, generalize poorly to novel environments, and add neural network inference as a dependency. The most practical variant is **learned traversability costmaps** that output standard grid costmap formats with richer semantic information — enhancing rather than replacing the grid representation.

Grid costmaps persist because they provide natural sensor fusion (LiDAR → cells), efficient incremental updates, layer composability (nav2's static + obstacle + inflation + custom layers), a massive algorithm ecosystem, and predictable memory/compute. The most likely evolution is enriching grids — with SDFs, learned traversability, semantic layers — not replacing them.

---

## Multi-robot path finding operates on grids by design

MAPF algorithms are inherently grid-based. The standard formulation places agents on a 2D grid where they move synchronously between adjacent cells. This deliberate abstraction makes the combinatorial problem tractable — conflict detection reduces to simple vertex/edge comparisons at integer timestamps. Amazon's warehouse robots literally navigate on a grid of barcode stickers on the floor, making MAPF a natural fit.

**Conflict-Based Search (CBS)** remains the foundational optimal MAPF algorithm, using a two-level search: a high-level constraint tree branches on conflicts, while low-level A* plans individual agent paths. With symmetry reasoning (rectangle, corridor, target symmetries), CBS handles **~200 agents optimally**. **EECBS** extends this to **~1,000 agents** with bounded-suboptimal solutions provably within 2% of optimal. For scale beyond that, **LaCAM** (AAAI 2023) uses lazy successor generation via PIBT to handle **10,000+ agents**, and **MAPF-LNS2** (Large Neighborhood Search) iteratively improves solutions for thousands of agents.

The 2023 League of Robot Runners competition (sponsored by Amazon Robotics) was won by **WPPL** — Windowed PIBT + LNS — which plans in rolling time horizons using PIBT for fast initial solutions refined by LNS. More recently, **SILLM** (2024) uses imitation learning with spatially sensitive communication to outperform WPPL on 10,000-agent instances, planning in under 1 second per step on GPU.

For a Rust costmap library, the key architectural implication is that **costmap choice matters less for MAPF than for single-robot planning**. MAPF operates on a graph abstraction derived from the costmap. The costmap's main role is defining which cells are traversable and providing cost weights. What matters more is supporting **time-space representations** — reservation tables mapping (cell, timestep) pairs to agent IDs — and efficient graph extraction from the costmap. A shared static costmap for MAPF plus individual dynamic local costmaps per robot for obstacle avoidance is the standard production architecture.

| Agent count | Recommended approach | Solution quality | Planning time |
|------------|---------------------|-----------------|--------------|
| ≤50 | CBS optimal | Optimal | <1s |
| 50–200 | CBS + symmetry reasoning | Optimal | <60s |
| 200–1,000 | EECBS | ≤2% suboptimal | <60s |
| 1,000–3,000 | LaCAM* or MAPF-LNS2 | 10–40% suboptimal | <10s |
| 3,000–10,000 | PIBT+LNS (WPPL) or SILLM | Suboptimal, high throughput | <1s/step |

---

## What industry actually deploys and where Rust fits

Production robotics navigation falls into a few distinct patterns. **Boston Dynamics** uses custom MPC + trajectory optimization + RL — no standard costmaps, no ROS. **Tesla FSD v12** replaced ~300,000 lines of modular planning code (including A* with neural heuristics) with **end-to-end neural networks** trained on 10M+ video clips — no grid costmaps at all. **Waymo** maintains a modular pipeline with HD maps and learned trajectory scoring (DriveIRL), actively researching end-to-end approaches (EMMA, powered by Gemini). **Amazon Robotics** uses grid-based MAPF with centralized traffic control for 750,000+ warehouse robots. **NVIDIA Isaac** uses GPU-accelerated TSDF/ESDF (nvblox) feeding into Nav2 costmaps, plus a visibility-graph planner with LQR local control.

The pattern is clear: structured environments (warehouses, factories) use **grids + MAPF**; unstructured environments (outdoors, human spaces) use **costmaps + continuous optimization**; autonomous vehicles increasingly use **learned planners**. For general-purpose robotics — where a Rust library would have the most impact — grid-based costmaps with EDT/SDF distance fields remain the practical choice.

The Rust robotics ecosystem is growing but lacks a comprehensive costmap/navigation library. Notable projects include **OpenRR** (the most complete Rust robotics framework, with `openrr-planner` for RRT and `openrr-nav` for 2D navigation), the **`pathfinding`** crate (generic A*, Dijkstra, BFS), and **`distance-transform`** (F&H EDT port). ROS2 Rust bindings exist via `rclrs`, `r2r`, and `ros2-client` (pure Rust with RustDDS). The **`optimization-engine`** crate provides embedded nonconvex optimization suitable for MPC/NMPC. The community hub at **robotics.rs** catalogs available libraries. There is a clear gap: no Rust crate combines costmap representation, distance fields, and modern planners in a single library.

---

## Recommended architecture for a Rust costmap library

Based on this research, the highest-impact design combines proven fundamentals with modern algorithmic improvements.

**Core representation**: A flat `Vec<f32>` grid in row-major layout, cache-line aligned (`#[repr(align(64))]`). For large maps, use block-hashed sparse allocation — fixed-size blocks (e.g., 16×16 cells) indexed by `HashMap<(i32, i32), Box<[f32; 256]>>` — providing O(1) amortized access with dynamic sizing and good cache locality within blocks. Store distances as squared Euclidean values (avoiding sqrt) with a conversion API to actual distances. Support both unsigned distance fields (for inflation-style costs) and signed distance fields (for trajectory optimization).

**Distance field computation**: F&H EDT for full recomputation, parallelized across rows with `rayon`, with a transpose-based vertical pass for cache friendliness. SIMD-accelerate the marching phase via `std::arch` intrinsics or the `wide` crate. Lau/FIESTA-style incremental updates with raise/lower `VecDeque` queues for sensor-driven changes, tracking closest-obstacle pointers per cell. Expose a cost conversion function mapping SDF distances to nav2-compatible 0–254 cost values.

**Planner suite** (in priority order): (1) A*/wA* as the universal baseline, (2) JPS for binary grids with 10–100× speedup, (3) JPSW for weighted grids — the single most impactful modern algorithm for costmap planning, (4) Lazy Theta* for any-angle paths, (5) D* Lite for incremental replanning in exploration scenarios. Make the planner trait-based so users can plug in their own.

**Multi-robot support**: Provide reservation table data structures mapping `(cell_index, timestep)` pairs for time-space planning. Support graph extraction from costmaps for MAPF solvers. Individual per-robot local costmaps plus a shared static map is the standard pattern.

Rust's type system offers unique advantages here. Use newtypes (`struct SquaredEuclideanDistance(f32)`, `struct SignedDistance(f32)`, `struct CostValue(u8)`) to prevent mixing distance representations at compile time. The ownership model naturally enforces single-writer semantics for grid updates. Generic grid types (`Grid<T: Copy>`) allow the same infrastructure to serve different cell types. The `rayon` ecosystem provides work-stealing parallelism that maps perfectly to row-independent EDT passes.

The biggest wins over a nav2-style approach are: exact Euclidean distances instead of approximate Brushfire, O(k) incremental updates instead of full-layer recomputation, and modern planners (JPSW, Lazy Theta*) that exploit distance field properties for dramatically faster or higher-quality paths.