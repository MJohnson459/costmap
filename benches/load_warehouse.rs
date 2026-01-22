use std::path::Path;

use criterion::{criterion_group, criterion_main, Criterion};

fn bench_load_warehouse(c: &mut Criterion) {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let yaml_path = manifest_dir.join("tests/fixtures/warehouse.yaml");

    c.bench_function("load_ros2_warehouse_map", |b| {
        b.iter(|| {
            let _grid = voxel_grid::load_occupancy_grid(&yaml_path)
                .expect("warehouse map should load");
        });
    });
}

criterion_group!(benches, bench_load_warehouse);
criterion_main!(benches);
