use std::path::Path;

use criterion::{Criterion, criterion_group, criterion_main};

use costmap::planning::Regions;
use costmap::{Costmap, PlanOptions, RosMapLoader, costmap_from_occupancy, plan};

fn warehouse_costmap() -> Costmap {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let grid = RosMapLoader::load_from_yaml(manifest_dir.join("tests/fixtures/warehouse.yaml"))
        .expect("warehouse map should load");
    costmap_from_occupancy(&grid).expect("the map converts to costs")
}

fn bench_astar(c: &mut Criterion) {
    let grid = warehouse_costmap();
    let options = PlanOptions::default();
    // The first and last cells, in row-major order, of the largest region.
    let regions = Regions::new(&grid, &options);
    let largest = regions
        .largest()
        .expect("warehouse map should contain free cells");
    let start = regions.cells(largest).next().expect("a region has a cell");
    let goal = regions.cells(largest).last().expect("a region has a cell");
    assert!(plan(&grid, start, goal, &options).is_some());

    c.bench_function("astar_warehouse_corner_to_corner", |b| {
        b.iter(|| plan(std::hint::black_box(&grid), start, goal, &options));
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().sample_size(10);
    targets = bench_astar
);
criterion_main!(benches);
