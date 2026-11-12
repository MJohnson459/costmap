use std::path::Path;
use std::time::Instant;

use criterion::{Criterion, criterion_group, criterion_main};
use glam::UVec2;

use costmap::planning::{DenseTiles, Regions, TileCache, TileGraph, TileGraphOptions};
use costmap::{Costmap, PlanOptions, RosMapLoader, costmap_from_occupancy, plan};

/// The warehouse map, inflated for a robot of 0.3 m radius.
fn warehouse_costmap() -> Costmap {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let grid = RosMapLoader::load_from_yaml(manifest_dir.join("tests/fixtures/warehouse.yaml"))
        .expect("warehouse map should load");
    let mut costs = costmap_from_occupancy(&grid).expect("the map converts to costs");
    costs.inflate(1.0, 0.3, 2.0);
    costs
}

/// Start and goal pairs spread over the largest region, the first of them its
/// row-major first and last cells.
fn queries(grid: &Costmap, options: &PlanOptions) -> Vec<(UVec2, UVec2)> {
    let regions = Regions::new(grid, options);
    let largest = regions.largest().expect("the map has passable cells");
    let cells: Vec<UVec2> = regions.cells(largest).collect();
    let pick = |i: usize| cells[i * (cells.len() - 1) / 16];
    let mut pairs = vec![(cells[0], cells[cells.len() - 1])];
    pairs.extend((1..8).map(|i| (pick(i), pick(16 - i))));
    pairs.extend((0..4).map(|i| (pick(2 * i + 1), pick(2 * i + 6))));
    pairs
}

fn bench_tile_graph(c: &mut Criterion) {
    let grid = warehouse_costmap();
    let options = TileGraphOptions::default();
    let pairs = queries(&grid, &options.plan);

    let built = Instant::now();
    let mut source = DenseTiles::new(&grid, options.tile_cells).unwrap();
    let graph = TileGraph::build(grid.size(), grid.info().resolution, options, &mut source)
        .expect("the graph builds");
    eprintln!(
        "{} x {} cells: built in {:.2} s, {} nodes, {:.1} MB",
        grid.size().x,
        grid.size().y,
        built.elapsed().as_secs_f64(),
        graph.node_count(),
        graph.memory_bytes() as f64 / 1e6,
    );

    let mut worst: f32 = 1.0;
    for &(start, goal) in &pairs {
        let whole_started = Instant::now();
        let whole = plan(&grid, start, goal, &options.plan).expect("the pair is joined");
        let whole_s = whole_started.elapsed().as_secs_f64();

        let source = DenseTiles::new(&grid, options.tile_cells).unwrap();
        let mut tiles = TileCache::new(source, &graph, 4096);
        let tiled_started = Instant::now();
        let tiled = graph.plan(start, goal, &mut tiles).unwrap().unwrap();
        let tiled_s = tiled_started.elapsed().as_secs_f64();
        let shortcut_started = Instant::now();
        let waypoints = graph.shortcut(&tiled, &mut tiles).unwrap();
        let shortcut_s = shortcut_started.elapsed().as_secs_f64();

        let ratio = tiled.cost / whole.cost;
        worst = worst.max(ratio);
        eprintln!(
            "{start} -> {goal}: whole map {:.0} ms, tile graph {:.2} ms + shortcut {:.2} ms, \
             {} tiles read, cost {:.1} vs {:.1} ({:+.1} %), {} waypoints",
            whole_s * 1e3,
            tiled_s * 1e3,
            shortcut_s * 1e3,
            tiles.fetched(),
            tiled.cost,
            whole.cost,
            (ratio - 1.0) * 100.0,
            waypoints.len(),
        );
    }
    eprintln!("worst cost ratio {worst:.4}");

    let (start, goal) = pairs[0];
    c.bench_function("tile_graph_warehouse_corner_to_corner", |b| {
        b.iter(|| {
            let source = DenseTiles::new(&grid, options.tile_cells).unwrap();
            let mut tiles = TileCache::new(source, &graph, 4096);
            graph
                .plan(std::hint::black_box(start), goal, &mut tiles)
                .unwrap()
        });
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().sample_size(10);
    targets = bench_tile_graph
);
criterion_main!(benches);
