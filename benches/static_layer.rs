use std::path::Path;

use criterion::{Criterion, criterion_group, criterion_main};
use glam::{DVec2, UVec2};

use costmap::types::{COST_FREE, CellRegion, Lattice, Pose2};
use costmap::{
    Costmap, LayeredCostmap, MergePolicy, Resample, RosMapLoader, StaticLayer,
    costmap_from_occupancy,
};

fn warehouse() -> Costmap {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let grid = RosMapLoader::load_from_yaml(manifest_dir.join("tests/fixtures/warehouse.yaml"))
        .expect("warehouse map should load");
    costmap_from_occupancy(&grid).expect("the map converts to costs")
}

/// A fixed 50 m master at `resolution`, its lower-left corner `offset` from the
/// warehouse's lattice, holding the warehouse through a static layer.
fn layered(
    map: &Costmap,
    resolution: f64,
    offset: f64,
    transform: Pose2,
    resample: Option<Resample>,
) -> LayeredCostmap {
    let side = (50.0 / resolution).round() as u32;
    let lattice = Lattice::new(DVec2::new(100.0 + offset, 50.0 + offset), resolution);
    let mut layered = LayeredCostmap::new(
        lattice,
        CellRegion::from_size(UVec2::splat(side)),
        COST_FREE,
        false,
    );
    let mut layer = StaticLayer::new(map.clone(), MergePolicy::Overwrite);
    layer.set_transform(transform);
    if let Some(resample) = resample {
        layer = layer.with_resample(resample);
    }
    layered.add_layer(Box::new(layer)).unwrap();
    layered
}

fn bench_static_layer(c: &mut Criterion) {
    let map = warehouse();
    let drift = Pose2::new(DVec2::new(0.4, -0.3), 0.012);
    let cases = [
        ("static_layer/copy_1000", 0.05, 0.0, Pose2::default(), None),
        ("static_layer/odom_nearest_1000", 0.05, 0.0, drift, None),
        (
            "static_layer/odom_worst_1000",
            0.05,
            0.0,
            drift,
            Some(Resample::WorstCost),
        ),
        (
            "static_layer/coarse_worst_500",
            0.1,
            0.013,
            Pose2::default(),
            None,
        ),
        ("static_layer/coarse_odom_worst_500", 0.1, 0.0, drift, None),
    ];
    for (name, resolution, offset, transform, resample) in cases {
        let mut layered = layered(&map, resolution, offset, transform, resample);
        c.bench_function(name, |b| b.iter(|| layered.update_map(Pose2::default())));
    }
}

criterion_group!(benches, bench_static_layer);
criterion_main!(benches);
