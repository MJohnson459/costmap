use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use glam::Vec2;

use costmap::iterators::line::{LineIterator, LineValueIterator, LineValueMutIterator};
use costmap::types::{FREE, MapInfo};

fn bench_line_iterator(c: &mut Criterion) {
    let mut grid = build_empty_grid(256, 256, 0.05);
    let rays = build_rays();

    c.bench_function("line_iterator_steps_only", |b| {
        b.iter(|| {
            let mut steps = 0usize;
            for (origin, dir, max_t) in &rays {
                if let Some(iter) = LineIterator::new(&mut grid, origin, dir, *max_t) {
                    steps += iter.count();
                }
            }
            black_box(steps);
        });
    });

    c.bench_function("line_value_iterator_read", |b| {
        b.iter(|| {
            let mut sum = 0i32;
            for (origin, dir, max_t) in &rays {
                if let Some(iter) = LineValueIterator::new(&grid, origin, dir, *max_t) {
                    for value in iter {
                        sum += *value as i32;
                    }
                }
            }
            black_box(sum);
        });
    });

    c.bench_function("line_value_mut_iterator_write", |b| {
        b.iter_batched(
            || build_empty_grid(256, 256, 0.05),
            |mut grid| {
                for (origin, dir, max_t) in &rays {
                    if let Some(iter) = LineValueMutIterator::new(&mut grid, origin, dir, *max_t) {
                        for value in iter {
                            *value = FREE;
                        }
                    }
                }
                black_box(grid);
            },
            BatchSize::SmallInput,
        );
    });
}

fn build_empty_grid(width: usize, height: usize, resolution: f32) -> costmap::OccupancyGrid {
    let data = vec![FREE; width * height];
    let info = MapInfo {
        width: width as u32,
        height: height as u32,
        resolution,
        ..Default::default()
    };
    costmap::Grid2d::new(info, data).expect("grid should build")
}

fn build_rays() -> Vec<(Vec2, Vec2, f32)> {
    let mut rays = Vec::new();
    for i in 0..64 {
        let origin = Vec2::new(0.1, 0.1 + i as f32 * 0.02);
        let dir = Vec2::new(1.0, (i as f32 * 0.01) - 0.3).normalize();
        rays.push((origin, dir, 20.0));
    }
    rays.push((Vec2::new(2.0, 2.0), Vec2::new(-1.0, 0.2).normalize(), 15.0));
    rays.push((Vec2::new(6.0, 1.0), Vec2::new(0.2, 1.0).normalize(), 15.0));
    rays
}

criterion_group!(benches, bench_line_iterator);
criterion_main!(benches);
