use criterion::{Criterion, black_box, criterion_group, criterion_main};
use glam::Vec2;

use voxel_grid::types::{FREE, MapInfo, OCCUPIED};

fn bench_raycast(c: &mut Criterion) {
    let grid = build_grid(256, 256, 0.05);
    let empty_grid = build_empty_grid(256, 256, 0.05);
    let early_hit_grid = build_grid_with_occupied_column(256, 256, 0.05, 1);
    let late_hit_grid = build_grid_with_occupied_column(256, 256, 0.05, 254);
    let rays = build_rays();
    let rays_positive_x = build_rays_positive_x();

    c.bench_function("raycast_dda_hits", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays {
                if grid.raycast_dda(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_grid_step_hits", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays {
                if grid.raycast_grid_step(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_dda_miss", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays {
                if empty_grid.raycast_dda(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_grid_step_miss", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays {
                if empty_grid.raycast_grid_step(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_dda_hits_early", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays_positive_x {
                if early_hit_grid.raycast_dda(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_grid_step_hits_early", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays_positive_x {
                if early_hit_grid.raycast_grid_step(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_dda_hits_late", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays_positive_x {
                if late_hit_grid.raycast_dda(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });

    c.bench_function("raycast_grid_step_hits_late", |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for (origin, dir, max_t) in &rays_positive_x {
                if late_hit_grid.raycast_grid_step(origin, dir, *max_t).is_some() {
                    hits += 1;
                }
            }
            black_box(hits);
        });
    });
}

fn build_grid(width: usize, height: usize, resolution: f32) -> voxel_grid::OccupancyGrid {
    let mut data = vec![FREE; width * height];
    for y in (0..height).step_by(16) {
        for x in (0..width).step_by(16) {
            let idx = y * width + x;
            data[idx] = OCCUPIED;
        }
    }

    let info = MapInfo {
        width: width as u32,
        height: height as u32,
        depth: 1,
        resolution,
        origin: glam::Vec3::new(0.0, 0.0, 0.0),
    };

    voxel_grid::Grid2d::new(info, data).expect("grid should build")
}

fn build_empty_grid(width: usize, height: usize, resolution: f32) -> voxel_grid::OccupancyGrid {
    let data = vec![FREE; width * height];
    let info = MapInfo {
        width: width as u32,
        height: height as u32,
        depth: 1,
        resolution,
        origin: glam::Vec3::new(0.0, 0.0, 0.0),
    };
    voxel_grid::Grid2d::new(info, data).expect("grid should build")
}

fn build_grid_with_occupied_column(
    width: usize,
    height: usize,
    resolution: f32,
    column: usize,
) -> voxel_grid::OccupancyGrid {
    let mut data = vec![FREE; width * height];
    let col = column.min(width.saturating_sub(1));
    for y in 0..height {
        let idx = y * width + col;
        data[idx] = OCCUPIED;
    }
    let info = MapInfo {
        width: width as u32,
        height: height as u32,
        depth: 1,
        resolution,
        origin: glam::Vec3::new(0.0, 0.0, 0.0),
    };
    voxel_grid::Grid2d::new(info, data).expect("grid should build")
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

fn build_rays_positive_x() -> Vec<(Vec2, Vec2, f32)> {
    let mut rays = Vec::new();
    for i in 0..64 {
        let origin = Vec2::new(0.1, 0.1 + i as f32 * 0.02);
        let dir = Vec2::new(1.0, 0.02).normalize();
        rays.push((origin, dir, 20.0));
    }
    rays
}

criterion_group!(benches, bench_raycast);
criterion_main!(benches);
