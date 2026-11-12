//! Routes across a whole map: a cost GeoTIFF or a ROS map.
//!
//! Nav2 holds a window of the map, so its own planner cannot reach a goal
//! outside it. [`RoutePlanner`] plans on a [`TileGraph`] of the whole raster:
//! the places each tile's border can be crossed and the costs between them.
//! The graph is built at the raster's own resolution, so a doorway the robot
//! fits through stays open. Planning reads only the tiles a route passes
//! through, so memory use does not grow with the raster.

use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Instant, UNIX_EPOCH};

use costmap::types::{COST_UNKNOWN, VoxelError};
use costmap::{
    CostGeoTiff, Costmap, MapInfo, RosMapLoader, TileCache, TileGraph, TileGraphOptions,
    TileLayout, TileSource, TiledGrid, costmap_from_occupancy,
};
use glam::{DVec2, IVec2, UVec2};
use thiserror::Error;
use tracing::{info, warn};

/// Chunk edge of the grid raster pixels are decoded into.
const CHUNK_CELLS: u32 = 256;
const COST_SCALING: f32 = 2.0;
/// Inflated tiles kept between plans: 16 MB at the default tile size.
const TILES_HELD: usize = 256;
/// First bytes of a stored graph, ending in the format version.
const GRAPH_MAGIC: [u8; 8] = *b"CMROUTE\x01";

/// How a [`RoutePlanner`] plans.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteOptions {
    /// Robot radius in metres. Cells nearer than this to an obstacle are
    /// impassable.
    pub robot_radius_m: f32,
    /// Distance from obstacles over which cost falls off, in metres. Routes
    /// keep away from obstacles by up to this much where there is room.
    pub inflation_m: f32,
    /// Farthest a start or goal is moved to reach a passable cell, in metres.
    pub snap_m: f64,
    /// Largest gap between consecutive route points, in metres.
    pub point_spacing_m: f64,
    /// Tile size and crossing spacing of the graph, and the planner's costs.
    pub graph: TileGraphOptions,
}

impl Default for RouteOptions {
    fn default() -> Self {
        Self {
            robot_radius_m: 0.3,
            inflation_m: 1.5,
            snap_m: 1.0,
            point_spacing_m: 0.1,
            graph: TileGraphOptions::default(),
        }
    }
}

/// A route could not be planned.
#[derive(Debug, Error, PartialEq)]
pub enum RouteError {
    #[error("{name} {point} has no passable cell within {snap_m} m")]
    Blocked {
        name: &'static str,
        point: DVec2,
        snap_m: f64,
    },
    #[error("no route from {start} to {goal}")]
    NoRoute { start: DVec2, goal: DVec2 },
    #[error("reading the map failed: {0}")]
    Map(String),
}

impl From<VoxelError> for RouteError {
    fn from(error: VoxelError) -> Self {
        Self::Map(error.to_string())
    }
}

/// A route in the map frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// Points from start to goal, at most `point_spacing_m` apart.
    pub points: Vec<DVec2>,
    /// Length along the points, in metres.
    pub length_m: f64,
}

/// Cells of a map before inflation, read a rectangle at a time.
trait RawCosts {
    /// The cells `[low, low + size)`, unknown where the map has none.
    fn window(&mut self, low: IVec2, size: UVec2) -> Result<Costmap, VoxelError>;
}

/// A cost GeoTIFF, of which the pixels around the last tile stay decoded.
struct GeoTiffCosts {
    map: CostGeoTiff,
    decoded: TiledGrid<u8>,
}

impl RawCosts for GeoTiffCosts {
    fn window(&mut self, low: IVec2, size: UVec2) -> Result<Costmap, VoxelError> {
        // Neighbouring tiles are asked for next and share this one's margin.
        let reach = size.max_element();
        self.decoded
            .evict_outside(low + (size / 2).as_ivec2(), reach);
        self.map
            .load_region(&mut self.decoded, low, low + size.as_ivec2())?;
        Ok(self.decoded.window(low, size))
    }
}

impl RawCosts for Costmap {
    fn window(&mut self, low: IVec2, size: UVec2) -> Result<Costmap, VoxelError> {
        let info = MapInfo {
            width: size.x,
            height: size.y,
            resolution: self.info().resolution,
            origin: self.info().origin + low.as_dvec2() * self.info().resolution,
        };
        let mut window = Costmap::filled(info, COST_UNKNOWN);
        // The rows and columns of the window that lie inside the map.
        let first = low.max(IVec2::ZERO);
        let last = (low + size.as_ivec2()).min(self.size().as_ivec2());
        if first.cmplt(last).all() {
            let (width, out_width) = (self.size().x as usize, size.x as usize);
            let columns = (last.x - first.x) as usize;
            for y in first.y..last.y {
                let from = y as usize * width + first.x as usize;
                let to = (y - low.y) as usize * out_width + (first.x - low.x) as usize;
                window.as_mut_slice()[to..to + columns]
                    .copy_from_slice(&self.as_slice()[from..from + columns]);
            }
        }
        Ok(window)
    }
}

/// Inflated tiles of a map, each read with a margin as wide as inflation
/// reaches, so every obstacle that raises a cost in the tile is in the window.
struct InflatedTiles<R> {
    raw: R,
    layout: TileLayout,
    /// Cells read beyond each edge of a tile.
    margin: u32,
    /// Distance from an obstacle that inflation reaches: `inflation_m`, or the
    /// robot radius if that is larger, so cells the robot cannot occupy are
    /// impassable whatever `inflation_m` is.
    reach_m: f32,
    options: RouteOptions,
}

impl<R: RawCosts> InflatedTiles<R> {
    fn new(
        raw: R,
        size: UVec2,
        resolution: f64,
        options: &RouteOptions,
    ) -> Result<Self, VoxelError> {
        let reach_m = options.inflation_m.max(options.robot_radius_m);
        Ok(Self {
            raw,
            layout: TileLayout::new(size, options.graph.tile_cells)?,
            margin: (f64::from(reach_m) / resolution).ceil() as u32 + 1,
            reach_m,
            options: *options,
        })
    }
}

impl<R: RawCosts> TileSource for InflatedTiles<R> {
    fn tile(&mut self, tile: UVec2) -> Result<Costmap, VoxelError> {
        let (min, size) = (self.layout.tile_min(tile), self.layout.tile_size(tile));
        let low = min.as_ivec2() - IVec2::splat(self.margin as i32);
        let mut costs = self.raw.window(low, size + 2 * self.margin)?;
        costs.inflate(self.reach_m, self.options.robot_radius_m, COST_SCALING);
        crop(&costs, UVec2::splat(self.margin), size)
    }
}

/// The `size` cells of `grid` from `min`.
fn crop(grid: &Costmap, min: UVec2, size: UVec2) -> Result<Costmap, VoxelError> {
    let width = grid.size().x as usize;
    let mut data = Vec::with_capacity(size.x as usize * size.y as usize);
    for y in min.y..min.y + size.y {
        let row = y as usize * width + min.x as usize;
        data.extend_from_slice(&grid.as_slice()[row..row + size.x as usize]);
    }
    let info = MapInfo {
        width: size.x,
        height: size.y,
        resolution: grid.info().resolution,
        origin: grid.info().origin + min.as_dvec2() * grid.info().resolution,
    };
    Costmap::from_vec(info, data)
}

/// What a stored graph was built from: the raster's length and modification
/// time, the options that shape its costs, and this crate's version. A raster
/// replaced by one of the same length and time is not told apart.
fn fingerprint(map: &Path, options: &RouteOptions) -> Result<Vec<u8>, VoxelError> {
    let metadata = fs::metadata(map)?;
    let modified = metadata
        .modified()?
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let mut bytes = GRAPH_MAGIC.to_vec();
    bytes.extend(metadata.len().to_le_bytes());
    bytes.extend(modified.as_nanos().to_le_bytes());
    bytes.extend(options.robot_radius_m.to_le_bytes());
    bytes.extend(options.inflation_m.to_le_bytes());
    bytes.extend(COST_SCALING.to_le_bytes());
    // A later version may inflate or place crossings differently.
    bytes.extend(env!("CARGO_PKG_VERSION").as_bytes());
    Ok(bytes)
}

/// Where the graph of the raster at `map` is stored: beside it, with
/// `.tilegraph` added to its name.
pub fn graph_path(map: &Path) -> PathBuf {
    let mut name = map.as_os_str().to_owned();
    name.push(".tilegraph");
    PathBuf::from(name)
}

/// The graph stored at `path`, if it was built from this raster with these
/// options.
fn read_graph(path: &Path, fingerprint: &[u8], options: &RouteOptions) -> Option<TileGraph> {
    let mut input = BufReader::new(File::open(path).ok()?);
    let mut stored = vec![0; fingerprint.len()];
    input.read_exact(&mut stored).ok()?;
    if stored != fingerprint {
        return None;
    }
    let graph = TileGraph::read_from(&mut input).ok()?;
    (*graph.options() == options.graph).then_some(graph)
}

/// Store `graph` at `path` through a temporary file, so a reader never sees
/// part of one.
fn write_graph(path: &Path, fingerprint: &[u8], graph: &TileGraph) -> std::io::Result<()> {
    let mut partial = path.as_os_str().to_owned();
    partial.push(".part");
    let partial = PathBuf::from(partial);
    let mut out = BufWriter::new(File::create(&partial)?);
    out.write_all(fingerprint)?;
    graph.write_to(&mut out)?;
    out.flush()?;
    drop(out);
    fs::rename(&partial, path)
}

/// Plans routes across a whole cost map.
pub struct RoutePlanner {
    graph: TileGraph,
    tiles: Mutex<TileCache<Box<dyn TileSource + Send>>>,
    /// Map-frame position of the corner of cell `(0, 0)`.
    origin: DVec2,
    options: RouteOptions,
}

impl RoutePlanner {
    /// Plan on the map at `path`, chosen by extension.
    ///
    /// `.yaml` and `.yml` are a ROS map. It is held whole in memory, as
    /// `nav2_map_server` holds it, and its graph is built at every start.
    ///
    /// `.tif` and `.tiff` are a cost GeoTIFF, of which only the tiles a route
    /// passes are held. Any other extension is an error. Its graph is read from [`graph_path`] if the
    /// file there was built from this raster with these options. Otherwise it
    /// is built, which reads the whole raster twice, a few tiles at a time,
    /// and stored there for the next start. If it cannot be stored, a warning
    /// is logged and it is built again next time.
    pub fn open(path: impl AsRef<Path>, options: RouteOptions) -> Result<Self, VoxelError> {
        let path = path.as_ref();
        let extension = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase);
        match extension.as_deref() {
            Some("yaml" | "yml") => {
                let occupancy = RosMapLoader::load_from_yaml(path)?;
                return Self::from_costmap(costmap_from_occupancy(&occupancy)?, options);
            }
            Some("tif" | "tiff") => {}
            _ => {
                return Err(VoxelError::InvalidArgument(format!(
                    "{}: unsupported extension; accepted: .yaml, .yml (ROS map), .tif, .tiff (cost GeoTIFF)",
                    path.display()
                )));
            }
        }
        let map = CostGeoTiff::open(path)?;
        let (size, resolution) = (map.size(), map.resolution());
        let raw = GeoTiffCosts {
            decoded: TiledGrid::new(resolution, CHUNK_CELLS, COST_UNKNOWN)?,
            map,
        };
        let mut source = InflatedTiles::new(raw, size, resolution, &options)?;
        let stored = graph_path(path);
        let fingerprint = fingerprint(path, &options)?;
        let graph = match read_graph(&stored, &fingerprint, &options) {
            Some(graph) => {
                info!("read the tile graph from {}", stored.display());
                graph
            }
            None => {
                let started = Instant::now();
                let graph = TileGraph::build(size, resolution, options.graph, &mut source)?;
                info!(
                    "built the tile graph in {:.1} s",
                    started.elapsed().as_secs_f64()
                );
                if let Err(error) = write_graph(&stored, &fingerprint, &graph) {
                    warn!(
                        "could not store the tile graph at {}: {error}",
                        stored.display()
                    );
                }
                graph
            }
        };
        Ok(Self::new(graph, Box::new(source), DVec2::ZERO, options))
    }

    /// Plan on `costs`, held in memory and inflated a tile at a time. The map
    /// frame is the frame `costs` is placed in by its origin.
    pub fn from_costmap(costs: Costmap, options: RouteOptions) -> Result<Self, VoxelError> {
        let info = *costs.info();
        let size = UVec2::new(info.width, info.height);
        let mut source = InflatedTiles::new(costs, size, info.resolution, &options)?;
        let graph = TileGraph::build(size, info.resolution, options.graph, &mut source)?;
        Ok(Self::new(graph, Box::new(source), info.origin, options))
    }

    fn new(
        graph: TileGraph,
        source: Box<dyn TileSource + Send>,
        origin: DVec2,
        options: RouteOptions,
    ) -> Self {
        Self {
            tiles: Mutex::new(TileCache::new(source, &graph, TILES_HELD)),
            graph,
            origin,
            options,
        }
    }

    /// The graph routes are planned on.
    pub fn graph(&self) -> &TileGraph {
        &self.graph
    }

    /// The route from `start` to `goal`, both in the map frame.
    ///
    /// An endpoint on an impassable cell moves to the nearest passable one
    /// within `snap_m`. The route starts and ends at the given points; between
    /// them it runs through cell centres, shortcut into straight segments
    /// where the map allows.
    pub fn plan(&self, start: DVec2, goal: DVec2) -> Result<Route, RouteError> {
        self.plan_through(start, &[], goal)
    }

    /// The route from `start` to `goal` that passes through each of `via` in
    /// order. Each leg is planned as [`plan`](Self::plan) plans a route.
    pub fn plan_through(
        &self,
        start: DVec2,
        via: &[DVec2],
        goal: DVec2,
    ) -> Result<Route, RouteError> {
        let mut stops = vec![(start, "start")];
        stops.extend(via.iter().map(|&point| (point, "viapoint")));
        stops.push((goal, "goal"));

        let mut tiles = self
            .tiles
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mut route = Route {
            points: vec![start],
            length_m: 0.0,
        };
        for pair in stops.windows(2) {
            self.extend(&mut tiles, &mut route, pair[0], pair[1])?;
        }
        Ok(route)
    }

    /// Appends the leg between two named stops to `route`, which ends at `from`.
    fn extend(
        &self,
        tiles: &mut TileCache<Box<dyn TileSource + Send>>,
        route: &mut Route,
        (from, from_name): (DVec2, &'static str),
        (to, to_name): (DVec2, &'static str),
    ) -> Result<(), RouteError> {
        let from_cell = self.snap(tiles, from, from_name)?;
        let to_cell = self.snap(tiles, to, to_name)?;
        let path = self
            .graph
            .plan(from_cell, to_cell, tiles)?
            .ok_or(RouteError::NoRoute {
                start: from,
                goal: to,
            })?;

        let resolution = self.graph.resolution();
        let mut corners = vec![from];
        corners.extend(
            self.graph
                .shortcut(&path, tiles)?
                .into_iter()
                .map(|cell| self.origin + (cell.as_dvec2() + 0.5) * resolution),
        );
        corners.push(to);

        for pair in corners.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let distance = a.distance(b);
            if distance == 0.0 {
                continue;
            }
            route.length_m += distance;
            let steps = (distance / self.options.point_spacing_m).ceil().max(1.0) as u32;
            route
                .points
                .extend((1..=steps).map(|i| a.lerp(b, f64::from(i) / f64::from(steps))));
        }
        Ok(())
    }

    /// The passable cell nearest `point`, within `snap_m`.
    fn snap(
        &self,
        tiles: &mut TileCache<Box<dyn TileSource + Send>>,
        point: DVec2,
        name: &'static str,
    ) -> Result<UVec2, RouteError> {
        let blocked = RouteError::Blocked {
            name,
            point,
            snap_m: self.options.snap_m,
        };
        let resolution = self.graph.resolution();
        let local = (point - self.origin) / resolution;
        let size = self.graph.layout().size();
        let reach = (self.options.snap_m / resolution).ceil();
        if !local.is_finite()
            || local.cmplt(DVec2::splat(-reach)).any()
            || local.cmpgt(size.as_dvec2() + reach).any()
        {
            return Err(blocked);
        }
        let inside = |cell: IVec2| {
            UVec2::try_from(cell)
                .ok()
                .filter(|c| c.x < size.x && c.y < size.y)
        };
        let centre = local.floor().as_ivec2();
        if let Some(cell) = inside(centre)
            && tiles.cost(cell)?.is_some()
        {
            return Ok(cell);
        }
        // The passable cell whose centre is nearest the point, within `snap_m`.
        let reach = reach as i32;
        let mut best: Option<(f64, UVec2)> = None;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let Some(cell) = inside(centre + IVec2::new(dx, dy)) else {
                    continue;
                };
                let distance = (cell.as_dvec2() + 0.5).distance(local) * resolution;
                if distance > self.options.snap_m || best.is_some_and(|(d, _)| distance >= d) {
                    continue;
                }
                if tiles.cost(cell)?.is_some() {
                    best = Some((distance, cell));
                }
            }
        }
        best.map(|(_, cell)| cell).ok_or(blocked)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::types::{COST_FREE, COST_LETHAL};

    /// Small tiles, so the 20 m test maps span several.
    const OPTIONS: RouteOptions = RouteOptions {
        robot_radius_m: 0.3,
        inflation_m: 1.0,
        snap_m: 1.0,
        point_spacing_m: 0.1,
        graph: TileGraphOptions {
            tile_cells: 32,
            node_spacing_cells: 8,
            plan: costmap::PlanOptions {
                lethal_threshold: costmap::types::COST_INSCRIBED,
                allow_unknown: false,
                cost_weight: 3.0,
            },
        },
    };

    fn planner(costs: Costmap) -> RoutePlanner {
        RoutePlanner::from_costmap(costs, OPTIONS).unwrap()
    }

    /// 20 x 20 m at 0.25 m with a wall along x = 10 m, open only for y in 15..17 m.
    fn walled_planner() -> RoutePlanner {
        let info = MapInfo::square(80, 0.25);
        planner(Costmap::from_fn(info, |x, y| {
            if x == 40 && !(60..68).contains(&y) {
                COST_LETHAL
            } else {
                COST_FREE
            }
        }))
    }

    #[test]
    fn route_goes_through_the_gap_and_ends_at_the_given_points() {
        let planner = walled_planner();
        let (start, goal) = (DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0));
        let route = planner.plan(start, goal).unwrap();
        assert_eq!(route.points.first(), Some(&start));
        assert_eq!(route.points.last(), Some(&goal));
        let crossing = route
            .points
            .iter()
            .find(|p| p.x >= 10.0)
            .expect("the route crosses the wall line");
        assert!((15.0..17.0).contains(&crossing.y), "crossed at {crossing}");
        // Up to the gap and back down is far longer than the 16 m straight line.
        assert!(route.length_m > 30.0, "length {}", route.length_m);
    }

    #[test]
    fn points_are_no_farther_apart_than_the_spacing() {
        let route = walled_planner()
            .plan(DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0))
            .unwrap();
        let widest = route
            .points
            .windows(2)
            .map(|p| p[0].distance(p[1]))
            .fold(0.0, f64::max);
        assert!(widest <= OPTIONS.point_spacing_m + 1e-9, "gap {widest}");
        let summed: f64 = route.points.windows(2).map(|p| p[0].distance(p[1])).sum();
        assert!((summed - route.length_m).abs() < 1e-6);
    }

    #[test]
    fn an_endpoint_beside_the_wall_snaps_to_a_passable_cell() {
        let planner = walled_planner();
        // 0.2 m from the wall: inside the robot radius, so its own cell is impassable.
        let start = DVec2::new(9.8, 5.0);
        let route = planner.plan(start, DVec2::new(2.0, 2.0)).unwrap();
        assert_eq!(route.points[0], start);
    }

    #[test]
    fn an_endpoint_with_no_passable_cell_nearby_is_an_error() {
        let info = MapInfo::square(80, 0.25);
        let costs = Costmap::from_fn(info, |x, _| if x < 40 { COST_LETHAL } else { COST_FREE });
        let planner = planner(costs);
        let start = DVec2::new(2.0, 2.0);
        let err = planner.plan(start, DVec2::new(18.0, 2.0)).unwrap_err();
        assert_eq!(
            err,
            RouteError::Blocked {
                name: "start",
                point: start,
                snap_m: 1.0
            }
        );
        let outside = planner.plan(DVec2::new(-5.0, 2.0), DVec2::new(18.0, 2.0));
        assert!(matches!(
            outside,
            Err(RouteError::Blocked { name: "start", .. })
        ));
    }

    #[test]
    fn a_route_passes_through_its_viapoints_in_order() {
        let planner = walled_planner();
        let (start, goal) = (DVec2::new(2.0, 2.0), DVec2::new(4.0, 2.0));
        let via = [DVec2::new(2.0, 18.0), DVec2::new(8.0, 10.0)];
        let direct = planner.plan(start, goal).unwrap();
        let route = planner.plan_through(start, &via, goal).unwrap();
        let first = route.points.iter().position(|p| *p == via[0]);
        let second = route.points.iter().position(|p| *p == via[1]);
        assert!(first.is_some() && first < second, "{first:?} {second:?}");
        assert_eq!(route.points.last(), Some(&goal));
        assert!(route.length_m > direct.length_m + 20.0);

        let blocked = planner.plan_through(start, &[DVec2::new(-3.0, 5.0)], goal);
        assert!(matches!(
            blocked,
            Err(RouteError::Blocked {
                name: "viapoint",
                ..
            })
        ));
    }

    #[test]
    fn a_closed_wall_gives_no_route() {
        let info = MapInfo::square(80, 0.25);
        let costs = Costmap::from_fn(info, |x, _| if x == 40 { COST_LETHAL } else { COST_FREE });
        let planner = planner(costs);
        let (start, goal) = (DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0));
        assert_eq!(
            planner.plan(start, goal),
            Err(RouteError::NoRoute { start, goal })
        );
    }

    #[test]
    fn a_doorway_the_robot_fits_through_stays_open() {
        // 20 x 20 m at 0.05 m. The wall along x = 10 m has one opening 0.9 m
        // wide, which leaves the 0.3 m robot 0.15 m on each side.
        let info = MapInfo::square(400, 0.05);
        let doorway = 200..218;
        let planner = planner(Costmap::from_fn(info, |x, y| {
            if (200..203).contains(&x) && !doorway.contains(&y) {
                COST_LETHAL
            } else {
                COST_FREE
            }
        }));
        let route = planner
            .plan(DVec2::new(2.0, 2.0), DVec2::new(18.0, 2.0))
            .unwrap();
        let crossing = route.points.iter().find(|p| p.x >= 10.0).unwrap();
        assert!((10.0..10.9).contains(&crossing.y), "crossed at {crossing}");
    }

    #[test]
    fn a_start_just_outside_the_map_snaps_onto_it() {
        let planner = walled_planner();
        let start = DVec2::new(-0.4, 5.0);
        let route = planner.plan(start, DVec2::new(2.0, 2.0)).unwrap();
        assert_eq!(route.points[0], start);
        let far = planner.plan(DVec2::new(-1.5, 5.0), DVec2::new(2.0, 2.0));
        assert!(matches!(
            far,
            Err(RouteError::Blocked { name: "start", .. })
        ));
    }

    #[test]
    fn cells_inside_the_robot_radius_are_impassable_with_no_inflation() {
        // 0.25 m cells; the wall along x = 10 m has a gap 0.5 m wide, which a
        // 0.3 m robot does not fit through.
        let info = MapInfo::square(80, 0.25);
        let costs = Costmap::from_fn(info, |x, y| {
            if x == 40 && !(40..42).contains(&y) {
                COST_LETHAL
            } else {
                COST_FREE
            }
        });
        let options = RouteOptions {
            inflation_m: 0.0,
            ..OPTIONS
        };
        let planner = RoutePlanner::from_costmap(costs, options).unwrap();
        let (start, goal) = (DVec2::new(2.0, 10.0), DVec2::new(18.0, 10.0));
        assert_eq!(
            planner.plan(start, goal),
            Err(RouteError::NoRoute { start, goal })
        );
    }

    #[test]
    fn an_unknown_map_extension_is_an_error() {
        let opened = RoutePlanner::open("site.png", OPTIONS);
        assert!(matches!(opened, Err(VoxelError::InvalidArgument(_))));
    }

    #[test]
    fn a_ros_map_is_planned_on_in_its_own_frame() {
        let dir = std::env::temp_dir().join(format!("costmap_route_yaml_{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        // 10 x 10 m at 0.1 m with its lower-left corner at (-5, -5). A wall
        // along x = 0 is open above y = 3.
        let mut pixels = Vec::new();
        for row in 0..100u32 {
            for x in 0..100u32 {
                let y = 99 - row;
                pixels.push(if x == 50 && y < 80 { 0u8 } else { 254 });
            }
        }
        let mut pgm = b"P5\n100 100\n255\n".to_vec();
        pgm.extend(pixels);
        fs::write(dir.join("map.pgm"), pgm).unwrap();
        fs::write(
            dir.join("map.yaml"),
            "image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\nnegate: 0\n\
             occupied_thresh: 0.65\nfree_thresh: 0.25\n",
        )
        .unwrap();

        let planner = RoutePlanner::open(dir.join("map.yaml"), OPTIONS).unwrap();
        let (start, goal) = (DVec2::new(-3.0, -3.0), DVec2::new(3.0, -3.0));
        let route = planner.plan(start, goal).unwrap();
        assert_eq!(route.points.first(), Some(&start));
        assert_eq!(route.points.last(), Some(&goal));
        let crossing = route.points.iter().find(|p| p.x >= 0.0).unwrap();
        assert!(crossing.y > 3.0, "crossed at {crossing}");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_stored_graph_is_used_until_the_options_change() {
        let dir = std::env::temp_dir().join(format!("costmap_route_{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let map = dir.join("cost.tif");
        let info = MapInfo::square(100, 0.25);
        let costs = Costmap::from_fn(info, |x, y| {
            if x == 50 && y > 10 {
                COST_LETHAL
            } else {
                COST_FREE
            }
        });
        let mut tiles = TiledGrid::new(0.25, 64, COST_UNKNOWN).unwrap();
        tiles.write_window(&costs).unwrap();
        let georef = costmap::GeoReference {
            epsg: Some(32633),
            anchor: DVec2::new(500_000.0, 5_000_000.0),
        };
        costmap::write_cost_geotiff_region(&map, &tiles, &georef, IVec2::ZERO, IVec2::splat(100))
            .unwrap();

        let (start, goal) = (DVec2::new(2.0, 20.0), DVec2::new(22.0, 20.0));
        let built = RoutePlanner::open(&map, OPTIONS).unwrap();
        let expected = built.plan(start, goal).unwrap();
        let fingerprint = fingerprint(&map, &OPTIONS).unwrap();
        assert!(read_graph(&graph_path(&map), &fingerprint, &OPTIONS).is_some());
        let reopened = RoutePlanner::open(&map, OPTIONS).unwrap();
        assert_eq!(reopened.plan(start, goal).unwrap(), expected);

        let wider = RouteOptions {
            robot_radius_m: 0.5,
            ..OPTIONS
        };
        let other = fingerprint_of(&map, &wider);
        assert!(read_graph(&graph_path(&map), &other, &wider).is_none());
        fs::remove_dir_all(&dir).unwrap();
    }

    fn fingerprint_of(map: &Path, options: &RouteOptions) -> Vec<u8> {
        fingerprint(map, options).unwrap()
    }
}
