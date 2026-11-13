//! Planning across a map split into square tiles.
//!
//! [`plan`](super::plan) searches cells, so its time and memory grow with the area of
//! the map. A [`TileGraph`] is built once per map: it records where the border of each
//! tile can be crossed and what it costs to travel between those crossings inside the
//! tile. [`TileGraph::plan`] then searches the crossings, and reads only the tiles the
//! route passes through to turn it into cells.
//!
//! Crossings are found at the map's own resolution, so a gap one cell wide is kept. A
//! route exists in the graph exactly when [`plan`](super::plan) finds one on the whole
//! map with the same [`PlanOptions`]. Its cost can be higher: a route passes through
//! the chosen crossing cells, at most [`TileGraphOptions::node_spacing_cells`] apart,
//! and is cheapest between them.

use std::borrow::Borrow;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::f32::consts::SQRT_2;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex};

use glam::{IVec2, UVec2};

use crate::{
    Costmap, MapInfo,
    types::{COST_FREE, COST_UNKNOWN, VoxelError},
};

use super::astar::{COST_SCALE, Path, PlanOptions, effective_cost_weight};
use super::smooth::shortcut_cells;

/// Stored cost of a cell the planner cannot enter.
const IMPASSABLE: u8 = u8::MAX;

/// Moves to the eight neighbours, as (dx, dy, whether diagonal).
const MOVES: [(i32, i32, bool); 8] = [
    (1, 0, false),
    (-1, 0, false),
    (0, 1, false),
    (0, -1, false),
    (1, 1, true),
    (1, -1, true),
    (-1, 1, true),
    (-1, -1, true),
];

const NO_NODE: u32 = u32::MAX;

/// Largest map a stored graph may describe, in tiles.
const MAX_STORED_TILES: u64 = 1 << 24;

/// Gives each graph in this process a different [`TileGraph::id`].
static NEXT_GRAPH_ID: AtomicU64 = AtomicU64::new(0);

/// First bytes of a stored [`TileGraph`], ending in the format version.
const MAGIC: [u8; 8] = *b"CMTILEG\x01";

/// How a [`TileGraph`] is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TileGraphOptions {
    /// Tile edge in cells.
    pub tile_cells: u32,
    /// Longest stretch of open tile border served by one crossing, in cells. Smaller
    /// values give cheaper routes and a larger graph.
    pub node_spacing_cells: u32,
    /// Passability and step costs, as for [`plan`](super::plan).
    pub plan: PlanOptions,
}

impl Default for TileGraphOptions {
    fn default() -> Self {
        Self {
            tile_cells: 256,
            node_spacing_cells: 32,
            plan: PlanOptions::default(),
        }
    }
}

/// How a map of `size` cells divides into tiles of `tile_cells`.
///
/// Tile `(i, j)` starts at cell `(i, j) * tile_cells`. Tiles on the east and north
/// edges of the map are cut off at the map's edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileLayout {
    size: UVec2,
    tile_cells: u32,
}

impl TileLayout {
    /// Fails if `tile_cells` is zero or either side of `size` is.
    pub fn new(size: UVec2, tile_cells: u32) -> Result<Self, VoxelError> {
        if tile_cells == 0 || size.x == 0 || size.y == 0 {
            return Err(VoxelError::InvalidArgument(format!(
                "tile layout needs a non-empty map and tile, got {size} cells in tiles of {tile_cells}"
            )));
        }
        Ok(Self { size, tile_cells })
    }

    /// Map size in cells.
    pub fn size(&self) -> UVec2 {
        self.size
    }

    /// Tile edge in cells.
    pub fn tile_cells(&self) -> u32 {
        self.tile_cells
    }

    /// Number of tiles along each axis.
    pub fn tiles(&self) -> UVec2 {
        UVec2::new(
            self.size.x.div_ceil(self.tile_cells),
            self.size.y.div_ceil(self.tile_cells),
        )
    }

    /// The tile holding `cell`.
    pub fn tile_of(&self, cell: IVec2) -> IVec2 {
        cell.div_euclid(IVec2::splat(self.tile_cells as i32))
    }

    /// The lowest cell of `tile`.
    pub fn tile_min(&self, tile: IVec2) -> IVec2 {
        tile * self.tile_cells as i32
    }

    /// Size of `tile` in cells.
    pub fn tile_size(&self, tile: IVec2) -> UVec2 {
        (self.size.as_ivec2() - self.tile_min(tile))
            .min(IVec2::splat(self.tile_cells as i32))
            .as_uvec2()
    }

    fn contains(&self, cell: IVec2) -> bool {
        (cell.x as u32) < self.size.x && (cell.y as u32) < self.size.y
    }

    fn tile_count(&self) -> usize {
        self.tiles().x as usize * self.tiles().y as usize
    }

    fn index(&self, tile: IVec2) -> usize {
        tile.y as usize * self.tiles().x as usize + tile.x as usize
    }

    fn tile_at(&self, index: usize) -> IVec2 {
        let across = self.tiles().x as usize;
        IVec2::new((index % across) as i32, (index / across) as i32)
    }
}

/// Supplies the costs of one tile at a time.
pub trait TileSource {
    /// Costs of `tile`: a grid of [`TileLayout::tile_size`] cells whose cell `(0, 0)` is
    /// [`TileLayout::tile_min`]. The costs are planned on as given, so a source inflates
    /// them first if the robot needs clearance, using cells beyond the tile's edge.
    fn tile(&mut self, tile: IVec2) -> Result<Costmap, VoxelError>;
}

impl<S: TileSource + ?Sized> TileSource for Box<S> {
    fn tile(&mut self, tile: IVec2) -> Result<Costmap, VoxelError> {
        (**self).tile(tile)
    }
}

/// Serves the tiles of a costmap held whole in memory, owned or borrowed.
pub struct DenseTiles<G> {
    grid: G,
    layout: TileLayout,
}

impl<G: Borrow<Costmap>> DenseTiles<G> {
    pub fn new(grid: G, tile_cells: u32) -> Result<Self, VoxelError> {
        let layout = TileLayout::new(grid.borrow().size(), tile_cells)?;
        Ok(Self { grid, layout })
    }
}

impl<G: Borrow<Costmap>> TileSource for DenseTiles<G> {
    fn tile(&mut self, tile: IVec2) -> Result<Costmap, VoxelError> {
        let grid = self.grid.borrow();
        let tiles = self.layout.tiles();
        if tile.x < 0 || tile.y < 0 || tile.x as u32 >= tiles.x || tile.y as u32 >= tiles.y {
            return Err(VoxelError::OutOfBounds(format!(
                "tile {tile} is outside the {tiles} tiles of the map"
            )));
        }
        let (min, size) = (self.layout.tile_min(tile), self.layout.tile_size(tile));
        let width = grid.size().x as usize;
        let mut data = Vec::with_capacity(size.x as usize * size.y as usize);
        for y in min.y..min.y + size.y as i32 {
            let row = y as usize * width + min.x as usize;
            data.extend_from_slice(&grid.as_slice()[row..row + size.x as usize]);
        }
        let source = grid.info();
        let info = MapInfo {
            width: size.x,
            height: size.y,
            resolution: source.resolution,
            origin: source.lattice().cell_corner(min),
        };
        Costmap::from_vec(info, data)
    }
}

/// One tile's costs as the planner sees them: the cost of entering each cell, or
/// [`IMPASSABLE`].
struct TileCosts {
    size: UVec2,
    costs: Vec<u8>,
}

impl TileCosts {
    fn fetch(
        source: &mut impl TileSource,
        layout: &TileLayout,
        tile: IVec2,
        options: &PlanOptions,
    ) -> Result<Self, VoxelError> {
        let grid = source.tile(tile)?;
        let size = layout.tile_size(tile);
        if grid.size() != size {
            return Err(VoxelError::InvalidArgument(format!(
                "tile {tile} has {} cells, expected {size}",
                grid.size()
            )));
        }
        let costs = grid
            .as_slice()
            .iter()
            .map(|&cost| match cost {
                COST_UNKNOWN if options.allow_unknown => COST_FREE,
                COST_UNKNOWN => IMPASSABLE,
                cost if cost < options.lethal_threshold => cost,
                _ => IMPASSABLE,
            })
            .collect();
        Ok(Self { size, costs })
    }

    fn index(&self, local: IVec2) -> usize {
        local.y as usize * self.size.x as usize + local.x as usize
    }

    fn local(&self, index: usize) -> IVec2 {
        let width = self.size.x as usize;
        IVec2::new((index % width) as i32, (index / width) as i32)
    }

    fn column(&self, x: i32) -> Vec<u8> {
        (0..self.size.y as i32)
            .map(|y| self.costs[self.index(IVec2::new(x, y))])
            .collect()
    }

    fn row(&self, y: i32) -> Vec<u8> {
        (0..self.size.x as i32)
            .map(|x| self.costs[self.index(IVec2::new(x, y))])
            .collect()
    }
}

/// The step costs of [`plan`](super::plan), tabulated.
#[derive(Debug, Clone)]
struct Steps {
    /// Step length in metres, straight then diagonal.
    length_m: [f32; 2],
    /// Multiplier on the length for entering a cell of each cost.
    factor: [f32; 256],
}

impl Steps {
    fn new(resolution: f64, options: &PlanOptions) -> Self {
        let weight = effective_cost_weight(options.cost_weight);
        let resolution = resolution as f32;
        Self {
            length_m: [resolution, SQRT_2 * resolution],
            factor: std::array::from_fn(|cost| 1.0 + weight * cost as f32 / COST_SCALE),
        }
    }

    fn cost(&self, diagonal: bool, entered: u8) -> f32 {
        self.length_m[usize::from(diagonal)] * self.factor[usize::from(entered)]
    }

    /// Octile distance in metres: a lower bound on the cost between cells `d` apart.
    fn octile(&self, a: IVec2, b: IVec2) -> f32 {
        let d = (a - b).abs();
        let (long, short) = (d.x.max(d.y) as f32, d.x.min(d.y) as f32);
        (long - short) * self.length_m[0] + short * self.length_m[1]
    }
}

/// Open-list entry, ordered so the lowest `f` pops first and ties prefer the larger `g`.
#[derive(Clone, Copy)]
struct Open {
    f: f32,
    g: f32,
    index: u32,
}

impl PartialEq for Open {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Open {}

impl PartialOrd for Open {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Open {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| self.g.total_cmp(&other.g))
            .then_with(|| other.index.cmp(&self.index))
    }
}

/// Which way the paths of a [`Search`] run.
#[derive(Clone, Copy, PartialEq)]
enum Direction {
    /// From the origin to each cell.
    From,
    /// From each cell to the origin.
    To,
}

/// A cheapest-path search inside one tile, with buffers kept between runs.
#[derive(Default)]
struct Search {
    g: Vec<f32>,
    /// Index into [`MOVES`] of the move that reached each cell.
    parent: Vec<u8>,
    wanted: Vec<bool>,
    open: BinaryHeap<Open>,
}

impl Search {
    /// Find the cheapest paths between `origin` and the `wanted` cells, staying inside
    /// `tile`. Afterwards `g` holds the cost for each wanted cell, infinite where no
    /// path exists; other cells hold bounds. With `toward`, the search is steered at
    /// that cell, which should then be the only wanted one.
    fn run(
        &mut self,
        tile: &TileCosts,
        steps: &Steps,
        origin: usize,
        direction: Direction,
        toward: Option<usize>,
        wanted: &[usize],
    ) {
        let cells = tile.costs.len();
        let (width, height) = (tile.size.x as i32, tile.size.y as i32);
        self.g.clear();
        self.g.resize(cells, f32::INFINITY);
        self.parent.clear();
        self.parent.resize(cells, 0);
        self.wanted.clear();
        self.wanted.resize(cells, false);
        self.open.clear();

        let mut remaining = 0;
        for &cell in wanted {
            if !self.wanted[cell] {
                self.wanted[cell] = true;
                remaining += 1;
            }
        }
        let target = toward.map(|cell| tile.local(cell));
        let heuristic = |index: usize| target.map_or(0.0, |t| steps.octile(tile.local(index), t));

        self.g[origin] = 0.0;
        self.open.push(Open {
            f: heuristic(origin),
            g: 0.0,
            index: origin as u32,
        });
        while let Some(node) = self.open.pop() {
            let index = node.index as usize;
            if node.g > self.g[index] {
                continue;
            }
            if self.wanted[index] {
                self.wanted[index] = false;
                remaining -= 1;
                if remaining == 0 {
                    break;
                }
            }
            let (x, y) = (index as i32 % width, index as i32 / width);
            for (k, &(dx, dy, diagonal)) in MOVES.iter().enumerate() {
                let (nx, ny) = (x + dx, y + dy);
                if nx < 0 || ny < 0 || nx >= width || ny >= height {
                    continue;
                }
                let next = (ny * width + nx) as usize;
                let cost = tile.costs[next];
                if cost == IMPASSABLE {
                    continue;
                }
                if diagonal
                    && (tile.costs[(y * width + nx) as usize] == IMPASSABLE
                        || tile.costs[(ny * width + x) as usize] == IMPASSABLE)
                {
                    continue;
                }
                let entered = match direction {
                    Direction::From => cost,
                    Direction::To => tile.costs[index],
                };
                let g = node.g + steps.cost(diagonal, entered);
                if g < self.g[next] {
                    self.g[next] = g;
                    self.parent[next] = k as u8;
                    self.open.push(Open {
                        f: g + heuristic(next),
                        g,
                        index: next as u32,
                    });
                }
            }
        }
    }

    /// The cells of the path found between `origin` and `cell`, starting at `cell`.
    fn trace(&self, tile: &TileCosts, origin: usize, mut cell: usize) -> Vec<usize> {
        let width = tile.size.x as i32;
        let mut cells = vec![cell];
        while cell != origin {
            let (dx, dy, _) = MOVES[usize::from(self.parent[cell])];
            cell = (cell as i32 - dy * width - dx) as usize;
            cells.push(cell);
        }
        cells
    }
}

/// A crossing cell on a tile's border.
#[derive(Debug, Clone, PartialEq)]
struct Node {
    cell: IVec2,
    tile: u32,
    /// Position of this node among its tile's nodes.
    slot: u32,
    /// The node on the adjacent cell of the neighbouring tile.
    partner: u32,
    /// Cost of stepping to the partner.
    cross: f32,
}

/// The costs along the four edges of one tile.
struct Edges {
    west: Vec<u8>,
    east: Vec<u8>,
    south: Vec<u8>,
    north: Vec<u8>,
}

/// Positions along a shared tile edge to place crossings at: each run of positions
/// passable on both sides is cut into stretches of at most `spacing`, and each stretch
/// gets one crossing at its cheapest position, the most central of equals.
fn crossings(near: &[u8], far: &[u8], spacing: usize) -> Vec<usize> {
    let open = |i: usize| near[i] != IMPASSABLE && far[i] != IMPASSABLE;
    let mut found = Vec::new();
    let mut i = 0;
    while i < near.len() {
        if !open(i) {
            i += 1;
            continue;
        }
        let start = i;
        while i < near.len() && open(i) {
            i += 1;
        }
        let length = i - start;
        let stretches = length.div_ceil(spacing);
        for k in 0..stretches {
            let (lo, hi) = (
                start + k * length / stretches,
                start + (k + 1) * length / stretches,
            );
            let best = (lo..hi)
                .min_by_key(|&p| {
                    let cost = u16::from(near[p]) + u16::from(far[p]);
                    (cost, (2 * p + 1).abs_diff(lo + hi))
                })
                .expect("a stretch holds at least one position");
            found.push(best);
        }
    }
    found
}

/// The crossings of every tile border of a map and the costs between them.
#[derive(Debug)]
pub struct TileGraph {
    /// Tells this graph from every other built or read in this process: a
    /// [`TileCache`] holds parts of plans that are true of one graph only.
    id: u64,
    layout: TileLayout,
    options: TileGraphOptions,
    resolution: f64,
    steps: Steps,
    nodes: Vec<Node>,
    /// The nodes on the border of each tile.
    tile_nodes: Vec<Vec<u32>>,
    /// For each tile with `k` nodes, the `k * k` costs of travelling inside the tile
    /// from the node of one slot to the node of another, indexed `from * k + to`.
    /// Infinite where the tile has no path.
    tile_costs: Vec<Vec<f32>>,
}

impl TileGraph {
    /// Build the graph of a map of `size` cells of `resolution` metres, reading each
    /// tile from `source` twice.
    pub fn build(
        size: UVec2,
        resolution: f64,
        options: TileGraphOptions,
        source: &mut impl TileSource,
    ) -> Result<Self, VoxelError> {
        if options.node_spacing_cells == 0 {
            return Err(VoxelError::InvalidArgument(
                "node_spacing_cells must be at least 1".into(),
            ));
        }
        let layout = TileLayout::new(size, options.tile_cells)?;
        let steps = Steps::new(resolution, &options.plan);
        let count = layout.tile_count();

        let mut edges = Vec::with_capacity(count);
        for index in 0..count {
            let tile = TileCosts::fetch(source, &layout, layout.tile_at(index), &options.plan)?;
            edges.push(Edges {
                west: tile.column(0),
                east: tile.column(tile.size.x as i32 - 1),
                south: tile.row(0),
                north: tile.row(tile.size.y as i32 - 1),
            });
        }

        let mut graph = Self {
            id: NEXT_GRAPH_ID.fetch_add(1, AtomicOrdering::Relaxed),
            layout,
            options,
            resolution,
            steps,
            nodes: Vec::new(),
            tile_nodes: vec![Vec::new(); count],
            tile_costs: vec![Vec::new(); count],
        };
        let spacing = options.node_spacing_cells as usize;
        let tiles = layout.tiles();
        for index in 0..count {
            let tile = layout.tile_at(index);
            let (min, size) = (layout.tile_min(tile), layout.tile_size(tile).as_ivec2());
            if tile.x + 1 < tiles.x as i32 {
                let east = index + 1;
                let (near, far) = (&edges[index].east, &edges[east].west);
                for p in crossings(near, far, spacing) {
                    let y = min.y + p as i32;
                    graph.link(
                        (index, IVec2::new(min.x + size.x - 1, y), near[p]),
                        (east, IVec2::new(min.x + size.x, y), far[p]),
                    );
                }
            }
            if tile.y + 1 < tiles.y as i32 {
                let north = index + tiles.x as usize;
                let (near, far) = (&edges[index].north, &edges[north].south);
                for p in crossings(near, far, spacing) {
                    let x = min.x + p as i32;
                    graph.link(
                        (index, IVec2::new(x, min.y + size.y - 1), near[p]),
                        (north, IVec2::new(x, min.y + size.y), far[p]),
                    );
                }
            }
        }

        // Tiles are read one at a time and searched on every core, a batch at a time,
        // so memory holds one batch of tiles and not the map.
        let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
        let with_nodes: Vec<usize> = (0..count)
            .filter(|&index| !graph.tile_nodes[index].is_empty())
            .collect();
        for batch in with_nodes.chunks(threads * 4) {
            let mut tiles = Vec::with_capacity(batch.len());
            for &index in batch {
                let tile = layout.tile_at(index);
                tiles.push(TileCosts::fetch(source, &layout, tile, &options.plan)?);
            }
            let next = AtomicUsize::new(0);
            let done = Mutex::new(Vec::with_capacity(batch.len()));
            std::thread::scope(|scope| {
                for _ in 0..threads.min(batch.len()) {
                    scope.spawn(|| {
                        let mut search = Search::default();
                        loop {
                            let i = next.fetch_add(1, AtomicOrdering::Relaxed);
                            let Some(tile) = tiles.get(i) else {
                                break;
                            };
                            let costs = graph.costs_within(batch[i], tile, &mut search);
                            done.lock()
                                .expect("no thread panicked")
                                .push((batch[i], costs));
                        }
                    });
                }
            });
            for (index, costs) in done.into_inner().expect("no thread panicked") {
                graph.tile_costs[index] = costs;
            }
        }
        Ok(graph)
    }

    /// The costs between every pair of nodes of a tile, as stored in `tile_costs`.
    fn costs_within(&self, tile_index: usize, tile: &TileCosts, search: &mut Search) -> Vec<f32> {
        let cells = self.node_cells(tile_index, tile);
        let mut costs = Vec::with_capacity(cells.len() * cells.len());
        for &from in &cells {
            search.run(tile, &self.steps, from, Direction::From, None, &cells);
            costs.extend(cells.iter().map(|&to| search.g[to]));
        }
        costs
    }

    /// Add a node on each side of a border, given as (tile, cell, cost of the cell).
    fn link(&mut self, a: (usize, IVec2, u8), b: (usize, IVec2, u8)) {
        let first = self.nodes.len() as u32;
        for (id, partner, (tile, cell, _), entered) in
            [(first, first + 1, a, b.2), (first + 1, first, b, a.2)]
        {
            self.nodes.push(Node {
                cell,
                tile: tile as u32,
                slot: self.tile_nodes[tile].len() as u32,
                partner,
                cross: self.steps.cost(false, entered),
            });
            self.tile_nodes[tile].push(id);
        }
    }

    /// Index within `tile` of the cell of each of its nodes, by slot.
    fn node_cells(&self, tile_index: usize, tile: &TileCosts) -> Vec<usize> {
        let min = self.layout.tile_min(self.layout.tile_at(tile_index));
        self.tile_nodes[tile_index]
            .iter()
            .map(|&id| tile.index(self.nodes[id as usize].cell - min))
            .collect()
    }

    /// Write the graph to `out`, for [`read_from`](Self::read_from). Numbers are
    /// little-endian.
    pub fn write_to(&self, out: &mut impl Write) -> std::io::Result<()> {
        let plan = &self.options.plan;
        out.write_all(&MAGIC)?;
        for value in [
            self.layout.size.x,
            self.layout.size.y,
            self.options.tile_cells,
            self.options.node_spacing_cells,
            u32::from(plan.lethal_threshold),
            u32::from(plan.allow_unknown),
            plan.cost_weight.to_bits(),
            self.nodes.len() as u32,
        ] {
            out.write_all(&value.to_le_bytes())?;
        }
        out.write_all(&self.resolution.to_le_bytes())?;
        for node in &self.nodes {
            for value in [node.cell.x, node.cell.y] {
                out.write_all(&value.to_le_bytes())?;
            }
            for value in [node.tile, node.slot, node.partner, node.cross.to_bits()] {
                out.write_all(&value.to_le_bytes())?;
            }
        }
        for costs in &self.tile_costs {
            for cost in costs {
                out.write_all(&cost.to_le_bytes())?;
            }
        }
        Ok(())
    }

    /// Read a graph written by [`write_to`](Self::write_to).
    ///
    /// Fails with [`VoxelError::InvalidMetadata`] if the bytes are not a tile graph of
    /// this format version or are inconsistent. Whether the graph describes a given
    /// map is for the caller to know: nothing of the map is stored.
    pub fn read_from(input: &mut impl Read) -> Result<Self, VoxelError> {
        let invalid = |what: &str| VoxelError::InvalidMetadata(format!("tile graph: {what}"));
        let mut magic = [0; 8];
        input.read_exact(&mut magic)?;
        if magic != MAGIC {
            return Err(invalid("not a tile graph of this format version"));
        }
        let mut word = || -> Result<u32, VoxelError> {
            let mut bytes = [0; 4];
            input.read_exact(&mut bytes)?;
            Ok(u32::from_le_bytes(bytes))
        };
        let size = UVec2::new(word()?, word()?);
        let options = TileGraphOptions {
            tile_cells: word()?,
            node_spacing_cells: word()?,
            plan: PlanOptions {
                lethal_threshold: u8::try_from(word()?).map_err(|_| invalid("lethal threshold"))?,
                allow_unknown: word()? != 0,
                cost_weight: f32::from_bits(word()?),
            },
        };
        let node_count = word()? as usize;
        let mut resolution = [0; 8];
        input.read_exact(&mut resolution)?;
        let resolution = f64::from_le_bytes(resolution);
        if !(resolution.is_finite() && resolution > 0.0) {
            return Err(invalid("resolution"));
        }
        let layout = TileLayout::new(size, options.tile_cells)?;
        let tiles = layout.tiles();
        if u64::from(tiles.x) * u64::from(tiles.y) > MAX_STORED_TILES {
            return Err(invalid("more tiles than a stored graph may have"));
        }
        let count = layout.tile_count();
        // A tile has at most this many border cells, and so nodes.
        let most_nodes = 4 * options.tile_cells as usize;

        let mut word = || -> Result<u32, VoxelError> {
            let mut bytes = [0; 4];
            input.read_exact(&mut bytes)?;
            Ok(u32::from_le_bytes(bytes))
        };
        let mut nodes = Vec::new();
        let mut tile_nodes = vec![Vec::new(); count];
        for id in 0..node_count {
            let node = Node {
                cell: IVec2::new(word()? as i32, word()? as i32),
                tile: word()?,
                slot: word()?,
                partner: word()?,
                cross: f32::from_bits(word()?),
            };
            let tile = node.tile as usize;
            let consistent = layout.contains(node.cell)
                && layout.index(layout.tile_of(node.cell)) == tile
                && tile_nodes[tile].len() == node.slot as usize
                && tile_nodes[tile].len() < most_nodes
                && (node.partner as usize) < node_count
                && node.cross >= 0.0;
            if !consistent {
                return Err(invalid("inconsistent node"));
            }
            tile_nodes[tile].push(id as u32);
            nodes.push(node);
        }
        // Each node's partner is the node on the adjacent cell across a tile border.
        for (id, node) in nodes.iter().enumerate() {
            let partner = &nodes[node.partner as usize];
            let apart = (node.cell - partner.cell).abs();
            if partner.partner as usize != id || partner.tile == node.tile || apart.x + apart.y != 1
            {
                return Err(invalid("a node's partner is not across its tile border"));
            }
        }
        let mut tile_costs = Vec::with_capacity(count);
        for ids in &tile_nodes {
            let mut costs = Vec::with_capacity(ids.len() * ids.len());
            for _ in 0..ids.len() * ids.len() {
                let cost = f32::from_bits(word()?);
                if cost.is_nan() || cost < 0.0 {
                    return Err(invalid("a cost is negative or not a number"));
                }
                costs.push(cost);
            }
            tile_costs.push(costs);
        }
        Ok(Self {
            id: NEXT_GRAPH_ID.fetch_add(1, AtomicOrdering::Relaxed),
            layout,
            options,
            resolution,
            steps: Steps::new(resolution, &options.plan),
            nodes,
            tile_nodes,
            tile_costs,
        })
    }

    /// The layout the graph was built for.
    pub fn layout(&self) -> TileLayout {
        self.layout
    }

    /// Cell size in metres.
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// The options the graph was built with.
    pub fn options(&self) -> &TileGraphOptions {
        &self.options
    }

    /// Number of crossing cells. Each crossing of a border is two: one on each side.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Bytes of heap the graph holds.
    pub fn memory_bytes(&self) -> usize {
        let per_tile = size_of::<Vec<u32>>() + size_of::<Vec<f32>>();
        let ids: usize = self.tile_nodes.iter().map(|n| n.len() * 4).sum();
        let costs: usize = self.tile_costs.iter().map(|c| c.len() * 4).sum();
        self.nodes.len() * size_of::<Node>() + self.tile_nodes.len() * per_tile + ids + costs
    }

    /// Plan from `start` to `goal`, reading tiles through `tiles`.
    ///
    /// Returns `None` where [`plan`](super::plan) on the whole map would: if either cell
    /// is outside the map or impassable, or no path joins them. The path is 8-connected
    /// and its cost is the sum of its step costs. It is the cheapest path through the
    /// crossings it uses, which is not always the cheapest path on the map.
    ///
    /// Fails if the source fails, if its tiles differ from those the graph was built
    /// from, or if `tiles` was made for another graph.
    pub fn plan<S: TileSource>(
        &self,
        start: IVec2,
        goal: IVec2,
        tiles: &mut TileCache<S>,
    ) -> Result<Option<Path>, VoxelError> {
        tiles.check(self)?;
        let layout = &self.layout;
        if !layout.contains(start) || !layout.contains(goal) {
            return Ok(None);
        }
        let start_tile = layout.index(layout.tile_of(start));
        let goal_tile = layout.index(layout.tile_of(goal));
        let start_costs = tiles.get(start_tile)?;
        let goal_costs = tiles.get(goal_tile)?;
        let start_cell = start_costs.index(start - layout.tile_min(layout.tile_of(start)));
        let goal_cell = goal_costs.index(goal - layout.tile_min(layout.tile_of(goal)));
        if start_costs.costs[start_cell] == IMPASSABLE || goal_costs.costs[goal_cell] == IMPASSABLE
        {
            return Ok(None);
        }
        if start == goal {
            return Ok(Some(Path {
                cells: vec![start],
                cost: 0.0,
            }));
        }

        let mut search = Search::default();
        let start_nodes = self.node_cells(start_tile, &start_costs);
        let mut wanted = start_nodes.clone();
        if start_tile == goal_tile {
            wanted.push(goal_cell);
        }
        search.run(
            &start_costs,
            &self.steps,
            start_cell,
            Direction::From,
            None,
            &wanted,
        );
        let from_start: Vec<f32> = start_nodes.iter().map(|&cell| search.g[cell]).collect();
        let direct = if start_tile == goal_tile {
            search.g[goal_cell]
        } else {
            f32::INFINITY
        };

        let to_goal = match &tiles.to_goal {
            Some((cell, costs)) if *cell == goal => costs.clone(),
            _ => {
                let goal_nodes = self.node_cells(goal_tile, &goal_costs);
                search.run(
                    &goal_costs,
                    &self.steps,
                    goal_cell,
                    Direction::To,
                    None,
                    &goal_nodes,
                );
                let costs: Arc<[f32]> = goal_nodes.iter().map(|&cell| search.g[cell]).collect();
                tiles.to_goal = Some((goal, costs.clone()));
                costs
            }
        };

        let Some(chain) =
            self.search_nodes(start_tile, &from_start, goal_tile, &to_goal, direct, goal)
        else {
            return Ok(None);
        };

        let mut path = Path {
            cells: vec![start],
            cost: 0.0,
        };
        let mut append = |stretch: &Stretch| {
            path.cells.extend_from_slice(&stretch.cells);
            path.cost += stretch.cost;
        };
        let (mut at, mut at_tile, mut at_node) = (start, start_tile, None);
        for &id in &chain {
            let node = &self.nodes[id as usize];
            if node.tile as usize != at_tile {
                let crossed = &self.nodes[node.partner as usize];
                append(&Stretch {
                    cells: vec![node.cell],
                    cost: crossed.cross,
                });
            } else if let Some(from) = at_node {
                // The stretch between two crossings is the same in every plan.
                let stretch = match tiles.stretches.get(&(from, id)) {
                    Some(stretch) => stretch.clone(),
                    None => {
                        let stretch =
                            Arc::new(self.refine(tiles, &mut search, at_tile, at, node.cell)?);
                        if tiles.stretches.len() >= STRETCHES_HELD {
                            tiles.stretches.clear();
                        }
                        tiles.stretches.insert((from, id), stretch.clone());
                        stretch
                    }
                };
                append(&stretch);
            } else {
                append(&self.refine(tiles, &mut search, at_tile, at, node.cell)?);
            }
            (at, at_tile, at_node) = (node.cell, node.tile as usize, Some(id));
        }
        append(&self.refine(tiles, &mut search, at_tile, at, goal)?);
        Ok(Some(path))
    }

    /// Search the nodes for the cheapest way from the start to the goal. Returns the
    /// nodes passed in order, which is empty when the direct path inside the shared
    /// tile is cheapest, or `None` if there is no way.
    fn search_nodes(
        &self,
        start_tile: usize,
        from_start: &[f32],
        goal_tile: usize,
        to_goal: &[f32],
        direct: f32,
        goal: IVec2,
    ) -> Option<Vec<u32>> {
        let goal_id = self.nodes.len();
        let mut best = vec![f32::INFINITY; goal_id + 1];
        let mut parent = vec![NO_NODE; goal_id + 1];
        let mut open = BinaryHeap::new();
        let heuristic = |id: usize| {
            if id == goal_id {
                0.0
            } else {
                self.steps.octile(self.nodes[id].cell, goal)
            }
        };
        let mut relax = |open: &mut BinaryHeap<Open>, id: usize, g: f32, from: u32| {
            if g < best[id] {
                best[id] = g;
                parent[id] = from;
                open.push(Open {
                    f: g + heuristic(id),
                    g,
                    index: id as u32,
                });
            }
            best[id]
        };
        for (slot, &id) in self.tile_nodes[start_tile].iter().enumerate() {
            relax(&mut open, id as usize, from_start[slot], NO_NODE);
        }
        relax(&mut open, goal_id, direct, NO_NODE);

        let mut found = false;
        while let Some(entry) = open.pop() {
            let id = entry.index as usize;
            if entry.g > relax(&mut open, id, f32::INFINITY, NO_NODE) {
                continue;
            }
            if id == goal_id {
                found = true;
                break;
            }
            let node = &self.nodes[id];
            let tile = node.tile as usize;
            let slot = node.slot as usize;
            relax(
                &mut open,
                node.partner as usize,
                entry.g + node.cross,
                id as u32,
            );
            let mates = &self.tile_nodes[tile];
            let row = &self.tile_costs[tile][slot * mates.len()..(slot + 1) * mates.len()];
            for (&mate, &cost) in mates.iter().zip(row) {
                relax(&mut open, mate as usize, entry.g + cost, id as u32);
            }
            if tile == goal_tile {
                relax(&mut open, goal_id, entry.g + to_goal[slot], id as u32);
            }
        }
        if !found {
            return None;
        }
        let mut chain = Vec::new();
        let mut id = parent[goal_id];
        while id != NO_NODE {
            chain.push(id);
            id = parent[id as usize];
        }
        chain.reverse();
        Some(chain)
    }

    /// The cheapest path inside a tile from `from` to `to`.
    fn refine<S: TileSource>(
        &self,
        tiles: &mut TileCache<S>,
        search: &mut Search,
        tile_index: usize,
        from: IVec2,
        to: IVec2,
    ) -> Result<Stretch, VoxelError> {
        if from == to {
            return Ok(Stretch {
                cells: Vec::new(),
                cost: 0.0,
            });
        }
        let tile = tiles.get(tile_index)?;
        let min = self.layout.tile_min(self.layout.tile_at(tile_index));
        let (origin, target) = (tile.index(from - min), tile.index(to - min));
        search.run(
            &tile,
            &self.steps,
            origin,
            Direction::From,
            Some(target),
            &[target],
        );
        if search.g[target].is_infinite() {
            return Err(VoxelError::InvalidArgument(format!(
                "no path from {from} to {to} inside their tile: the tile graph was built from a different map"
            )));
        }
        let cells = search.trace(&tile, origin, target);
        Ok(Stretch {
            cells: cells
                .iter()
                .rev()
                .skip(1)
                .map(|&cell| min + tile.local(cell))
                .collect(),
            cost: search.g[target],
        })
    }

    /// Waypoints along `path` joined by straight segments, as [`Path::shortcut`] gives
    /// on the whole map.
    pub fn shortcut<S: TileSource>(
        &self,
        path: &Path,
        tiles: &mut TileCache<S>,
    ) -> Result<Vec<IVec2>, VoxelError> {
        tiles.check(self)?;
        let mut failure = None;
        let waypoints = shortcut_cells(&path.cells, |cell| match tiles.cost(cell) {
            Ok(cost) => cost,
            Err(error) => {
                failure.get_or_insert(error);
                None
            }
        });
        match failure {
            Some(error) => Err(error),
            None => Ok(waypoints),
        }
    }
}

/// Part of a path: its cells after the first, and the cost of entering them.
struct Stretch {
    cells: Vec<IVec2>,
    cost: f32,
}

/// Stretches between crossings kept by a [`TileCache`]. A stretch crosses one tile, so
/// it holds a few hundred cells.
const STRETCHES_HELD: usize = 1024;

/// Tiles read from a [`TileSource`], kept for later plans, with the parts of earlier
/// plans that later ones repeat.
pub struct TileCache<S> {
    source: S,
    /// The [`TileGraph::id`] of the graph this cache was made for.
    graph: u64,
    layout: TileLayout,
    plan: PlanOptions,
    capacity: usize,
    clock: u64,
    /// Each held tile with the clock value of its last use.
    tiles: HashMap<usize, (u64, Arc<TileCosts>)>,
    last: Option<(usize, Arc<TileCosts>)>,
    /// Paths found between two nodes of one tile, by their ids.
    stretches: HashMap<(u32, u32), Arc<Stretch>>,
    /// The last goal and the cost to it from each node of its tile, by slot.
    to_goal: Option<(IVec2, Arc<[f32]>)>,
    fetched: usize,
}

impl<S: TileSource> TileCache<S> {
    /// Hold up to `capacity` tiles of `source` for plans on `graph`. A plan reads the
    /// tiles its route passes through; with fewer held, tiles are read again.
    pub fn new(source: S, graph: &TileGraph, capacity: usize) -> Self {
        Self {
            source,
            graph: graph.id,
            layout: graph.layout,
            plan: graph.options.plan,
            capacity: capacity.max(1),
            clock: 0,
            tiles: HashMap::new(),
            last: None,
            stretches: HashMap::new(),
            to_goal: None,
            fetched: 0,
        }
    }

    /// Number of tiles read from the source so far.
    pub fn fetched(&self) -> usize {
        self.fetched
    }

    /// Number of tiles held now.
    pub fn held(&self) -> usize {
        self.tiles.len()
    }

    fn check(&self, graph: &TileGraph) -> Result<(), VoxelError> {
        if self.graph != graph.id {
            return Err(VoxelError::InvalidArgument(
                "tile cache was made for a different tile graph".into(),
            ));
        }
        Ok(())
    }

    fn get(&mut self, index: usize) -> Result<Arc<TileCosts>, VoxelError> {
        self.clock += 1;
        if let Some((stamp, tile)) = self.tiles.get_mut(&index) {
            *stamp = self.clock;
            return Ok(tile.clone());
        }
        let tile = Arc::new(TileCosts::fetch(
            &mut self.source,
            &self.layout,
            self.layout.tile_at(index),
            &self.plan,
        )?);
        self.fetched += 1;
        if self.tiles.len() >= self.capacity {
            let oldest = self.tiles.iter().min_by_key(|(_, (stamp, _))| *stamp);
            if let Some((&oldest, _)) = oldest {
                self.tiles.remove(&oldest);
            }
        }
        self.tiles.insert(index, (self.clock, tile.clone()));
        Ok(tile)
    }

    /// Cost of entering `cell`, or `None` if it is impassable or outside the map.
    pub fn cost(&mut self, cell: IVec2) -> Result<Option<u8>, VoxelError> {
        if !self.layout.contains(cell) {
            return Ok(None);
        }
        let tile = self.layout.tile_of(cell);
        let index = self.layout.index(tile);
        let costs = match &self.last {
            Some((held, costs)) if *held == index => costs.clone(),
            _ => {
                let costs = self.get(index)?;
                self.last = Some((index, costs.clone()));
                costs
            }
        };
        let cost = costs.costs[costs.index(cell - self.layout.tile_min(tile))];
        Ok((cost != IMPASSABLE).then_some(cost))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        plan,
        types::{COST_LETHAL, MapInfo},
    };

    const OPTIONS: TileGraphOptions = TileGraphOptions {
        tile_cells: 8,
        node_spacing_cells: 3,
        plan: PlanOptions {
            lethal_threshold: crate::types::COST_INSCRIBED,
            allow_unknown: false,
            cost_weight: 3.0,
        },
    };

    fn info(width: u32, height: u32) -> MapInfo {
        MapInfo {
            width,
            height,
            resolution: 0.5,
            ..Default::default()
        }
    }

    fn cell(x: i32, y: i32) -> IVec2 {
        IVec2::new(x, y)
    }

    fn graph_of(grid: &Costmap, options: TileGraphOptions) -> TileGraph {
        let mut source = DenseTiles::new(grid, options.tile_cells).unwrap();
        TileGraph::build(grid.size(), grid.info().resolution, options, &mut source).unwrap()
    }

    fn plan_on(
        grid: &Costmap,
        graph: &TileGraph,
        start: IVec2,
        goal: IVec2,
    ) -> (Option<Path>, usize) {
        let source = DenseTiles::new(grid, graph.options().tile_cells).unwrap();
        let mut tiles = TileCache::new(source, graph, 64);
        let path = graph.plan(start, goal, &mut tiles).unwrap();
        (path, tiles.fetched())
    }

    /// Asserts `path` is a valid plan on `grid` and returns the sum of its step costs.
    fn check_path(grid: &Costmap, path: &Path, start: IVec2, goal: IVec2) -> f32 {
        let options = OPTIONS.plan;
        assert_eq!(path.cells.first(), Some(&start));
        assert_eq!(path.cells.last(), Some(&goal));
        let mut cost = 0.0;
        for pair in path.cells.windows(2) {
            let step = super::super::astar::moves(grid, pair[0], &options)
                .find(|step| step.cell == pair[1])
                .unwrap_or_else(|| panic!("{} -> {} is not a planner move", pair[0], pair[1]));
            let length_m = step.length_cells * grid.info().resolution as f32;
            cost += length_m * (1.0 + options.cost_weight * f32::from(step.cost) / 252.0);
        }
        cost
    }

    struct Lcg(u64);

    impl Lcg {
        fn below(&mut self, n: u32) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 33) % u64::from(n)) as u32
        }
    }

    #[test]
    fn finds_a_path_exactly_when_the_whole_map_search_does() {
        let (width, height) = (37, 29);
        let mut rng = Lcg(0x711E);
        let (mut joined, mut apart) = (0, 0);
        for _ in 0..60 {
            let mut grid = Costmap::filled(info(width, height), COST_FREE);
            for _ in 0..rng.below(14) {
                // Walls with gaps, graded cells and specks.
                let (x, y) = (rng.below(width) as i32, rng.below(height) as i32);
                let length = rng.below(20) as i32;
                let across = rng.below(2) == 0;
                let value = if rng.below(4) == 0 {
                    60 + rng.below(150) as u8
                } else {
                    COST_LETHAL
                };
                for i in 0..length {
                    let c = if across {
                        cell(x + i, y)
                    } else {
                        cell(x, y + i)
                    };
                    if c.x < width as i32 && c.y < height as i32 && rng.below(9) != 0 {
                        grid.set(c, value).unwrap();
                    }
                }
            }
            let graph = graph_of(&grid, OPTIONS);
            for _ in 0..40 {
                let start = cell(rng.below(width) as i32, rng.below(height) as i32);
                let goal = cell(rng.below(width) as i32, rng.below(height) as i32);
                let whole = plan(&grid, start, goal, &OPTIONS.plan);
                let (tiled, _) = plan_on(&grid, &graph, start, goal);
                assert_eq!(whole.is_some(), tiled.is_some(), "{start} -> {goal}");
                let (Some(whole), Some(tiled)) = (whole, tiled) else {
                    apart += 1;
                    continue;
                };
                joined += 1;
                let cost = check_path(&grid, &tiled, start, goal);
                assert!((cost - tiled.cost).abs() < 1e-3, "{cost} vs {}", tiled.cost);
                assert!(tiled.cost >= whole.cost - 1e-3, "{start} -> {goal}");
            }
        }
        assert!(
            joined > 500 && apart > 100,
            "{joined} joined, {apart} apart"
        );
    }

    #[test]
    fn a_gap_one_cell_wide_on_a_tile_border_is_kept() {
        // A wall along the border between the tiles at x = 7 and x = 8, two cells
        // thick, open at one cell.
        let grid = Costmap::from_fn(info(24, 24), |p| {
            if (p.x == 7 || p.x == 8) && p.y != 13 {
                COST_LETHAL
            } else {
                COST_FREE
            }
        });
        let graph = graph_of(&grid, OPTIONS);
        let (start, goal) = (cell(2, 2), cell(20, 2));
        let (path, _) = plan_on(&grid, &graph, start, goal);
        let path = path.expect("the gap joins the two sides");
        check_path(&grid, &path, start, goal);
        assert!(path.cells.contains(&cell(7, 13)) && path.cells.contains(&cell(8, 13)));
    }

    #[test]
    fn a_path_may_leave_the_tile_its_ends_share() {
        // A wall splits tile (0, 0) from top to bottom; the way round is through the
        // tile to the north.
        let grid = Costmap::from_fn(info(16, 16), |p| {
            if p.x == 4 && p.y < 8 {
                COST_LETHAL
            } else {
                COST_FREE
            }
        });
        let graph = graph_of(&grid, OPTIONS);
        let (start, goal) = (cell(1, 1), cell(6, 1));
        let (path, _) = plan_on(&grid, &graph, start, goal);
        let path = path.expect("the tile to the north joins the two halves");
        check_path(&grid, &path, start, goal);
        assert!(path.cells.iter().any(|c| c.y >= 8));
    }

    #[test]
    fn endpoints_outside_the_map_or_impassable_give_no_path() {
        let mut grid = Costmap::filled(info(20, 20), COST_FREE);
        grid.set(cell(15, 15), COST_LETHAL).unwrap();
        let graph = graph_of(&grid, OPTIONS);
        assert!(plan_on(&grid, &graph, cell(20, 0), cell(1, 1)).0.is_none());
        assert!(plan_on(&grid, &graph, cell(1, 1), cell(0, 20)).0.is_none());
        assert!(plan_on(&grid, &graph, cell(1, 1), cell(15, 15)).0.is_none());
        assert!(plan_on(&grid, &graph, cell(15, 15), cell(1, 1)).0.is_none());
        let (same, _) = plan_on(&grid, &graph, cell(3, 3), cell(3, 3));
        assert_eq!(same.unwrap().cells, [cell(3, 3)]);
    }

    #[test]
    fn open_ground_costs_little_more_than_the_whole_map_search() {
        let grid = Costmap::filled(info(200, 150), COST_FREE);
        let options = TileGraphOptions {
            tile_cells: 32,
            node_spacing_cells: 8,
            ..OPTIONS
        };
        let graph = graph_of(&grid, options);
        let (start, goal) = (cell(3, 5), cell(190, 77));
        let whole = plan(&grid, start, goal, &options.plan).unwrap();
        let (tiled, fetched) = plan_on(&grid, &graph, start, goal);
        let tiled = tiled.unwrap();
        assert!(
            tiled.cost < whole.cost * 1.05,
            "{} vs {}",
            tiled.cost,
            whole.cost
        );
        // 35 tiles hold the map; the route passes through a strip of them.
        assert!(fetched <= 12, "{fetched} tiles read");
    }

    #[test]
    fn shortcut_straightens_a_path_across_tiles() {
        let grid = Costmap::filled(info(64, 64), COST_FREE);
        let graph = graph_of(&grid, OPTIONS);
        let source = DenseTiles::new(&grid, OPTIONS.tile_cells).unwrap();
        let mut tiles = TileCache::new(source, &graph, 64);
        let (start, goal) = (cell(2, 3), cell(60, 41));
        let path = graph.plan(start, goal, &mut tiles).unwrap().unwrap();
        assert_eq!(graph.shortcut(&path, &mut tiles).unwrap(), [start, goal]);
    }

    #[test]
    fn a_small_cache_reads_tiles_again_and_plans_the_same_path() {
        let grid = Costmap::from_fn(info(64, 64), |p| {
            if p.x % 16 == 5 && p.y % 11 != 0 {
                COST_LETHAL
            } else {
                COST_FREE
            }
        });
        let graph = graph_of(&grid, OPTIONS);
        let (start, goal) = (cell(1, 1), cell(62, 60));
        let (expected, _) = plan_on(&grid, &graph, start, goal);
        let source = DenseTiles::new(&grid, OPTIONS.tile_cells).unwrap();
        let mut tiles = TileCache::new(source, &graph, 1);
        let path = graph.plan(start, goal, &mut tiles).unwrap();
        assert_eq!(path, expected);
        assert_eq!(tiles.held(), 1);
    }

    #[test]
    fn a_repeated_plan_is_the_same_path_and_reads_no_more_tiles() {
        let grid = Costmap::from_fn(info(64, 64), |p| {
            if p.x % 16 == 5 && p.y % 11 != 0 {
                COST_LETHAL
            } else {
                ((p.x * 3 + p.y) % 50) as u8
            }
        });
        let graph = graph_of(&grid, OPTIONS);
        let source = DenseTiles::new(&grid, OPTIONS.tile_cells).unwrap();
        let mut tiles = TileCache::new(source, &graph, 64);
        let (start, goal) = (cell(1, 1), cell(62, 60));
        let first = graph.plan(start, goal, &mut tiles).unwrap().unwrap();
        let fetched = tiles.fetched();
        assert_eq!(graph.plan(start, goal, &mut tiles).unwrap().unwrap(), first);
        assert_eq!(tiles.fetched(), fetched);
        // A start further along the same route shares its later stretches.
        let later = first.cells[first.cells.len() / 3];
        let rest = graph.plan(later, goal, &mut tiles).unwrap().unwrap();
        check_path(&grid, &rest, later, goal);
        let (other, _) = plan_on(&grid, &graph, later, goal);
        assert_eq!(Some(rest), other);
    }

    #[test]
    fn a_stored_graph_reads_back_and_plans_the_same_path() {
        let grid = Costmap::from_fn(info(40, 30), |p| {
            if p.x % 9 == 4 && p.y % 7 != 0 {
                COST_LETHAL
            } else {
                (p.x + p.y) as u8
            }
        });
        let graph = graph_of(&grid, OPTIONS);
        let mut bytes = Vec::new();
        graph.write_to(&mut bytes).unwrap();
        let read = TileGraph::read_from(&mut bytes.as_slice()).unwrap();
        assert_eq!(read.nodes, graph.nodes);
        assert_eq!(read.tile_costs, graph.tile_costs);
        assert_eq!(read.resolution(), graph.resolution());
        let (start, goal) = (cell(0, 0), cell(39, 29));
        let expected = plan_on(&grid, &graph, start, goal).0;
        assert!(expected.is_some());
        assert_eq!(plan_on(&grid, &read, start, goal).0, expected);

        assert!(TileGraph::read_from(&mut &bytes[..bytes.len() - 1]).is_err());
        // The partner of the first node is the fifth word after the 48-byte header.
        let mut wrong_partner = bytes.clone();
        wrong_partner[48 + 16..48 + 20].copy_from_slice(&0u32.to_le_bytes());
        assert!(matches!(
            TileGraph::read_from(&mut wrong_partner.as_slice()),
            Err(VoxelError::InvalidMetadata(_))
        ));
        bytes[7] ^= 0xff;
        assert!(matches!(
            TileGraph::read_from(&mut bytes.as_slice()),
            Err(VoxelError::InvalidMetadata(_))
        ));
    }

    #[test]
    fn a_cache_serves_only_the_graph_it_was_made_for() {
        let grid = Costmap::filled(info(40, 30), COST_FREE);
        let graph = graph_of(&grid, OPTIONS);
        let other = graph_of(&grid, OPTIONS);
        let source = DenseTiles::new(&grid, OPTIONS.tile_cells).unwrap();
        let mut tiles = TileCache::new(source, &graph, 64);
        assert!(graph.plan(cell(0, 0), cell(39, 29), &mut tiles).is_ok());
        assert!(matches!(
            other.plan(cell(0, 0), cell(39, 29), &mut tiles),
            Err(VoxelError::InvalidArgument(_))
        ));
    }

    #[test]
    fn crossings_split_long_runs_and_prefer_cheap_central_cells() {
        let free = [0u8; 10];
        assert_eq!(crossings(&free, &free, 10), [4]);
        assert_eq!(crossings(&free, &free, 5), [2, 7]);
        let mut near = [0u8; 10];
        near[3] = IMPASSABLE;
        // Runs 0..3 and 4..10; the second is split in two.
        assert_eq!(crossings(&near, &free, 3), [1, 5, 8]);
        let graded = [9, 9, 9, 9, 2, 9, 9, 9, 9, 9];
        assert_eq!(crossings(&graded, &free, 10), [4]);
        let far = [IMPASSABLE; 10];
        assert!(crossings(&free, &far, 3).is_empty());
    }
}
