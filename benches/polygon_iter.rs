use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use glam::Vec2;

use costmap::Grid2d;
use costmap::iterators::polygon::PolygonIterator;
use costmap::types::MapInfo;

fn bench_polygon_iter(c: &mut Criterion) {
    let info = MapInfo {
        width: 256,
        height: 256,
        resolution: 1.0,
        ..Default::default()
    };
    let grid = Grid2d::<u8>::new(info);

    let points = vec![
        Vec2::new(10.0, 10.0),
        Vec2::new(110.0, 10.0),
        Vec2::new(110.0, 110.0),
        Vec2::new(10.0, 110.0),
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
