use std::path::Path;

use voxel_grid::load_occupancy_grid;
use voxel_grid::types::{FREE, OCCUPIED, UNKNOWN};

#[test]
fn loads_trinary_ros2_map() {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let yaml_path = manifest_dir.join("tests/fixtures/simple.yaml");

    let grid = load_occupancy_grid(&yaml_path).expect("grid should load");

    assert_eq!(grid.width(), 2);
    assert_eq!(grid.height(), 2);

    assert_eq!(grid.get(0, 0), Some(OCCUPIED));
    assert_eq!(grid.get(1, 0), Some(FREE));
    assert_eq!(grid.get(0, 1), Some(FREE));
    assert_eq!(grid.get(1, 1), Some(UNKNOWN));
}
