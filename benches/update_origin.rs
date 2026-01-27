use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use glam::Vec3;

use voxel_grid::Grid2d;
use voxel_grid::types::MapInfo;

fn bench_update_origin(c: &mut Criterion) {
    let info = MapInfo {
        width: 256,
        height: 256,
        depth: 1,
        resolution: 0.05,
        origin: glam::Vec3::new(0.0, 0.0, 0.0),
    };
    let grid = Grid2d::<u8>::empty(info);

    c.bench_function("grid2d_update_origin_shift", |b| {
        b.iter_batched(
            || grid.clone(),
            |mut grid| {
                grid.update_origin(&Vec3::new(0.5, 0.0, 0.0));
                black_box(grid);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_update_origin);
criterion_main!(benches);
