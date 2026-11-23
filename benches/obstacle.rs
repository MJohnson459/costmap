use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use glam::{DVec2, UVec2};

use costmap::types::{COST_FREE, CellRegion, Lattice, Pose2};
use costmap::{LayeredCostmap, ObstacleConfig, ObstacleLayer};

const BEAMS: usize = 720;

/// A 10 m costmap at 0.05 m with the sensor at its centre.
fn lattice() -> Lattice {
    Lattice::new(DVec2::ZERO, 0.05)
}

fn cells() -> CellRegion {
    CellRegion::from_size(UVec2::splat(200))
}

fn sensor() -> Pose2 {
    Pose2::new(DVec2::new(5.0, 5.0), 0.3)
}

/// One revolution of returns between 1.0 m and 2.2 m, inside the default
/// obstacle range, so every beam clears and marks.
fn ranges() -> Vec<f32> {
    (0..BEAMS).map(|i| 1.0 + (i % 7) as f32 * 0.2).collect()
}

fn bench_obstacle(c: &mut Criterion) {
    let ranges = ranges();
    let increment = std::f32::consts::TAU / BEAMS as f32;

    c.bench_function("obstacle_insert_scan_720", |b| {
        let mut layer = ObstacleLayer::new(lattice(), cells(), ObstacleConfig::default());
        b.iter(|| {
            layer.insert_laser_scan(sensor(), 0.0, increment, black_box(&ranges), 0.0);
        });
    });

    c.bench_function("obstacle_insert_scan_720_and_update", |b| {
        let mut layered = LayeredCostmap::new(lattice(), cells(), COST_FREE, false);
        let id = layered
            .add_layer(Box::new(ObstacleLayer::new(
                lattice(),
                cells(),
                ObstacleConfig::default(),
            )))
            .expect("the layer attaches");
        b.iter(|| {
            layered
                .layer_mut::<ObstacleLayer>(id)
                .expect("the layer is an obstacle layer")
                .insert_laser_scan(sensor(), 0.0, increment, black_box(&ranges), 0.0);
            layered.update_map(sensor());
            black_box(layered.master());
        });
    });
}

criterion_group!(benches, bench_obstacle);
criterion_main!(benches);
