use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use glam::{UVec2, Vec3};

use costmap::Grid2d;
use costmap::types::MapInfo;

fn bench_resize_map(c: &mut Criterion) {
    let info = MapInfo {
        width: 256,
        height: 256,
        ..Default::default()
    };
    let grid = Grid2d::<u8>::empty(info);

    c.bench_function("grid2d_resize_map_overlap", |b| {
        b.iter_batched(
            || grid.clone(),
            |mut grid| {
                grid.resize_map(UVec2::new(320, 320), 0.05, &Vec3::new(0.5, 0.0, 0.0));
                black_box(grid);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_resize_map);
criterion_main!(benches);
