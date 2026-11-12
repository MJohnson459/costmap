use std::path::Path;

use criterion::{Criterion, criterion_group, criterion_main};
use glam::UVec2;

use costmap::types::{COST_FREE, COST_LETHAL, COST_UNKNOWN, FREE, UNKNOWN};
use costmap::{Costmap, PlanOptions, RosMapLoader, plan};

fn warehouse_costmap() -> Costmap {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let grid = RosMapLoader::load_from_yaml(manifest_dir.join("tests/fixtures/warehouse.yaml"))
        .expect("warehouse map should load");
    Costmap::from_fn(*grid.info(), |x, y| match grid.get(UVec2::new(x, y)) {
        Some(&FREE) => COST_FREE,
        Some(&UNKNOWN) | None => COST_UNKNOWN,
        Some(_) => COST_LETHAL,
    })
}

/// Cells of the largest 4-connected free region, in row-major order.
fn largest_free_region(grid: &Costmap) -> Vec<UVec2> {
    let width = grid.size().x;
    let index = |c: UVec2| (c.y * width + c.x) as usize;
    let mut seen = vec![false; width as usize * grid.size().y as usize];
    let mut best = Vec::new();
    for (seed, &cost) in grid.indexed_iter() {
        if cost != COST_FREE || seen[index(seed)] {
            continue;
        }
        let mut region = Vec::new();
        let mut stack = vec![seed];
        seen[index(seed)] = true;
        while let Some(cell) = stack.pop() {
            region.push(cell);
            let neighbours = [
                cell.checked_sub(UVec2::X),
                cell.checked_sub(UVec2::Y),
                Some(cell + UVec2::X),
                Some(cell + UVec2::Y),
            ];
            for next in neighbours.into_iter().flatten() {
                if grid.get(next) == Some(&COST_FREE) && !seen[index(next)] {
                    seen[index(next)] = true;
                    stack.push(next);
                }
            }
        }
        if region.len() > best.len() {
            best = region;
        }
    }
    best.sort_by_key(|c| (c.y, c.x));
    best
}

fn bench_astar(c: &mut Criterion) {
    let grid = warehouse_costmap();
    let region = largest_free_region(&grid);
    let (start, goal) = match (region.first(), region.last()) {
        (Some(&start), Some(&goal)) => (start, goal),
        _ => panic!("warehouse map should contain free cells"),
    };
    let options = PlanOptions::default();
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
