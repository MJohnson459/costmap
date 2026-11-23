use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use glam::{DVec2, UVec2};

use costmap::DenseGrid;
use costmap::iterators::polygon::PolygonIterator;
use costmap::types::{CellRegion, Lattice};

fn bench_polygon_iter(c: &mut Criterion) {
    let grid = DenseGrid::<u8>::new(
        Lattice::new(DVec2::ZERO, 1.0),
        CellRegion::from_size(UVec2::splat(256)),
    );

    let points = vec![
        DVec2::new(10.0, 10.0),
        DVec2::new(110.0, 10.0),
        DVec2::new(110.0, 110.0),
        DVec2::new(10.0, 110.0),
    ];

    c.bench_function("polygon_iter_rectangle_100x100", |b| {
        b.iter_batched(
            || points.clone(),
            |pts| {
                let count = PolygonIterator::new(&grid, &pts).unwrap().count();
                black_box(count);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_polygon_iter);
criterion_main!(benches);
