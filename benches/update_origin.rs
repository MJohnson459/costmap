use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use glam::{DVec2, UVec2};

use costmap::DenseGrid;
use costmap::types::{CellRegion, Lattice};

fn bench_update_origin(c: &mut Criterion) {
    let grid = DenseGrid::<u8>::new(
        Lattice::new(DVec2::ZERO, 0.05),
        CellRegion::from_size(UVec2::splat(256)),
    );

    c.bench_function("dense_grid_update_origin_shift", |b| {
        b.iter_batched(
            || grid.clone(),
            |mut grid| {
                grid.update_origin(DVec2::new(0.5, 0.0));
                black_box(grid);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_update_origin);
criterion_main!(benches);
