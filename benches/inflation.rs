use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use glam::{UVec2, Vec2};

use costmap::types::{COST_FREE, COST_LETHAL, MapInfo};
use costmap::{Grid2d, WavefrontInflationLayer};
use costmap::{
    InflationConfig,
    grid::{Bounds, CellRegion, Layer, LayeredGrid2d, Pose2},
};

#[derive(Clone, Copy)]
enum LethalPattern {
    Empty,
    SingleCenter,
    Sparse(u32),
    Dense(u32),
}

fn grid_with_lethals(
    width: u32,
    height: u32,
    resolution: f32,
    pattern: LethalPattern,
) -> Grid2d<u8> {
    let info = MapInfo {
        width,
        height,
        resolution,
        ..Default::default()
    };
    let mut grid = Grid2d::<u8>::new_with_value(info, COST_FREE);

    match pattern {
        LethalPattern::Empty => {}
        LethalPattern::SingleCenter => {
            let cx = width / 2;
            let cy = height / 2;
            let _ = grid.set(UVec2::new(cx, cy), COST_LETHAL);
        }
        LethalPattern::Sparse(step) => {
            let step = step.max(1);
            for y in (0..height).step_by(step as usize) {
                for x in (0..width).step_by(step as usize) {
                    let _ = grid.set(UVec2::new(x, y), COST_LETHAL);
                }
            }
        }
        LethalPattern::Dense(step) => {
            let step = step.max(1);
            for y in (0..height).step_by(step as usize) {
                for x in (0..width).step_by(step as usize) {
                    let _ = grid.set(UVec2::new(x, y), COST_LETHAL);
                }
            }
        }
    }

    grid
}

/// Layer that sets a fixed set of cells to COST_LETHAL (for benchmarking inflation).
struct StaticLethalsLayer {
    positions: Vec<UVec2>,
    info: MapInfo,
}

impl Layer for StaticLethalsLayer {
    fn reset(&mut self) {}

    fn is_clearable(&self) -> bool {
        false
    }

    fn update_bounds(&mut self, _robot: Pose2, bounds: &mut Bounds) {
        bounds.expand_to_include(self.info.origin);
        bounds.expand_to_include(
            self.info.origin + Vec2::new(self.info.world_width(), self.info.world_height()),
        );
    }

    fn update_costs(&mut self, master: &mut Grid2d<u8>, region: CellRegion) {
        for &pos in &self.positions {
            if pos.x >= region.min.x
                && pos.x < region.max.x
                && pos.y >= region.min.y
                && pos.y < region.max.y
            {
                let _ = master.set(pos, COST_LETHAL);
            }
        }
    }
}

fn bench_inflation(c: &mut Criterion) {
    let res_005 = 0.05;
    let res_002 = 0.02;

    const INSCRIBED_RADIUS_M: f32 = 0.1; // 2 cells at 0.05 m/cell
    const COST_SCALING: f32 = 3.0;

    // Groups: empty_256, single_center_64, sparse_256, dense_256, sparse_512, sparse_4000
    // Each group has wavefront + edt with matching names

    let mut group = c.benchmark_group("empty_256");
    group.bench_function("wavefront", |b| {
        let grid = grid_with_lethals(256, 256, res_005, LethalPattern::Empty);
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.5, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();

    let mut group = c.benchmark_group("single_center_64");
    group.bench_function("wavefront", |b| {
        let grid = grid_with_lethals(64, 64, res_005, LethalPattern::SingleCenter);
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.5, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();

    let mut group = c.benchmark_group("sparse_256");
    group.bench_function("wavefront", |b| {
        let grid = grid_with_lethals(256, 256, res_005, LethalPattern::Sparse(32));
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.5, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();

    let mut group = c.benchmark_group("dense_256");
    group.bench_function("wavefront", |b| {
        let grid = grid_with_lethals(256, 256, res_005, LethalPattern::Dense(4));
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.5, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();

    let mut group = c.benchmark_group("sparse_512");
    group.bench_function("wavefront", |b| {
        let grid = grid_with_lethals(512, 512, res_005, LethalPattern::Sparse(64));
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.5, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();

    // Large grid — 4000×4000 (16M cells) for scaling comparison
    let mut group = c.benchmark_group("sparse_4096");
    group.sample_size(20); // Fewer samples — each iteration is slow
    group.bench_function("wavefront", |b| {
        let grid = grid_with_lethals(4096, 4096, res_005, LethalPattern::Sparse(64));
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.5, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();

    // Wavefront-only scenarios (EDT has no equivalent)
    let mut group = c.benchmark_group("wavefront_only");
    group.bench_function("zero_radius", |b| {
        let grid = grid_with_lethals(64, 64, res_005, LethalPattern::SingleCenter);
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.0, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.bench_function("minimal_radius", |b| {
        let grid = grid_with_lethals(64, 64, res_005, LethalPattern::SingleCenter);
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(0.05, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.bench_function("large_radius", |b| {
        let grid = grid_with_lethals(256, 256, res_002, LethalPattern::Sparse(12));
        b.iter_batched(
            || grid.clone(),
            |mut g| {
                g.inflate(1.0, INSCRIBED_RADIUS_M, COST_SCALING);
                black_box(&g);
            },
            BatchSize::SmallInput,
        );
    });
    group.bench_function("inflation_layer_typical", |b| {
        let info = MapInfo {
            width: 256,
            height: 256,
            resolution: res_005,
            ..Default::default()
        };
        let positions: Vec<UVec2> = (0..256)
            .step_by(32)
            .flat_map(|y| (0..256).step_by(32).map(move |x| UVec2::new(x, y)))
            .collect();
        let static_layer = StaticLethalsLayer { positions, info };
        let inflation_layer = WavefrontInflationLayer::new(InflationConfig {
            inflation_radius_m: 0.5,
            inscribed_radius_m: 0.1,
            cost_scaling_factor: 1.0,
            ..Default::default()
        });
        let mut layered = LayeredGrid2d::new(info, COST_FREE, false);
        layered.add_layer(Box::new(static_layer));
        layered.add_layer(Box::new(inflation_layer));
        let robot = Pose2 {
            position: Vec2::new(6.4, 6.4),
            yaw: 0.0,
        };
        b.iter(|| {
            layered.update_map(robot);
            black_box(layered.master());
        });
    });
    group.finish();
}

criterion_group!(benches, bench_inflation);
criterion_main!(benches);
