use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use glam::Vec2;

use costmap::Grid2d;
use costmap::types::MapInfo;

fn bench_update_origin(c: &mut Criterion) {
    let info = MapInfo {
        width: 256,
        height: 256,
        ..Default::default()
    };
    let grid = Grid2d::<u8>::new(info);

    c.bench_function("grid2d_update_origin_shift", |b| {
        b.iter_batched(
            || grid.clone(),
            |mut grid| {
                grid.update_origin(Vec2::new(0.5, 0.0));
                black_box(grid);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_update_origin);
criterion_main!(benches);
