//! Routes across a whole cost GeoTIFF.
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
    CostGeoTiff, Costmap, DenseTiles, MapInfo, TileCache, TileGraph, TileGraphOptions, TileLayout,
    TileSource, TiledGrid,
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

/// Inflated tiles of a cost GeoTIFF, each read with enough of its surroundings
/// that inflation gives what it would on the whole raster.
struct GeoTiffTiles {
    map: CostGeoTiff,
    raw: TiledGrid<u8>,
    layout: TileLayout,
    /// Cells read beyond each edge of a tile.
    margin: u32,
    options: RouteOptions,
}

impl GeoTiffTiles {
    fn new(map: CostGeoTiff, options: &RouteOptions) -> Result<Self, VoxelError> {
        let resolution = map.resolution();
        Ok(Self {
            raw: TiledGrid::new(resolution, CHUNK_CELLS, COST_UNKNOWN)?,
            layout: TileLayout::new(map.size(), options.graph.tile_cells)?,
            margin: (f64::from(options.inflation_m) / resolution).ceil() as u32 + 1,
            options: *options,
            map,
        })
    }
}

impl TileSource for GeoTiffTiles {
    fn tile(&mut self, tile: UVec2) -> Result<Costmap, VoxelError> {
        let (min, size) = (self.layout.tile_min(tile), self.layout.tile_size(tile));
        let margin = IVec2::splat(self.margin as i32);
        let low = min.as_ivec2() - margin;
        let padded = size + 2 * self.margin;
        self.map
            .load_region(&mut self.raw, low, low + padded.as_ivec2())?;
        let mut costs = self.raw.window(low, padded);
        // Neighbouring tiles are asked for next, so their pixels stay decoded.
        let reach = self.layout.tile_cells() + 2 * self.margin;
        self.raw
            .evict_outside(min.as_ivec2() + (size / 2).as_ivec2(), reach);
        inflate(&mut costs, &self.options);
        crop(&costs, UVec2::splat(self.margin), size)
    }
}

fn inflate(costs: &mut Costmap, options: &RouteOptions) {
    costs.inflate(options.inflation_m, options.robot_radius_m, COST_SCALING);
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
/// time, and the options that shape its costs.
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
    options: RouteOptions,
}

impl RoutePlanner {
    /// Plan on the raster at `path`.
    ///
    /// The graph is read from [`graph_path`] if the file there was built from
    /// this raster with these options. Otherwise it is built, which reads the
    /// whole raster twice, a few tiles at a time, and stored there for the
    /// next start. If it cannot be stored, a warning is logged and it is
    /// built again next time.
    pub fn open(path: impl AsRef<Path>, options: RouteOptions) -> Result<Self, VoxelError> {
        let path = path.as_ref();
        let map = CostGeoTiff::open(path)?;
        let (size, resolution) = (map.size(), map.resolution());
        let mut source = GeoTiffTiles::new(map, &options)?;
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
        Ok(Self::new(graph, Box::new(source), options))
    }

    /// Plan on `costs`, held in memory and inflated here.
    pub fn from_costmap(mut costs: Costmap, options: RouteOptions) -> Result<Self, VoxelError> {
        inflate(&mut costs, &options);
        let (size, resolution) = (costs.size(), costs.info().resolution);
        let mut source = DenseTiles::new(costs, options.graph.tile_cells)?;
        let graph = TileGraph::build(size, resolution, options.graph, &mut source)?;
        Ok(Self::new(graph, Box::new(source), options))
    }

    fn new(graph: TileGraph, source: Box<dyn TileSource + Send>, options: RouteOptions) -> Self {
        Self {
            tiles: Mutex::new(TileCache::new(source, &graph, TILES_HELD)),
            graph,
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
                .map(|cell| (cell.as_dvec2() + 0.5) * resolution),
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
        let centre = (point / resolution).floor();
        let size = self.graph.layout().size().as_dvec2();
        if centre.cmplt(DVec2::ZERO).any() || centre.cmpge(size).any() {
            return Err(blocked);
        }
        let centre = centre.as_ivec2();
        let reach = (self.options.snap_m / resolution).ceil() as i32;
        let mut best: Option<(i64, UVec2)> = None;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let distance_sq = i64::from(dx) * i64::from(dx) + i64::from(dy) * i64::from(dy);
                if distance_sq > i64::from(reach) * i64::from(reach)
                    || best.is_some_and(|(d, _)| distance_sq >= d)
                {
                    continue;
                }
                let Ok(cell) = UVec2::try_from(centre + IVec2::new(dx, dy)) else {
                    continue;
                };
                if tiles.cost(cell)?.is_some() {
                    best = Some((distance_sq, cell));
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
