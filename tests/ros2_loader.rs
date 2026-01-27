use glam::UVec2;
use image::GenericImageView;
use std::path::Path;
use voxel_grid::types::{FREE, OCCUPIED, UNKNOWN};
use voxel_grid::{RosMapLoader, loaders::ros2::RosMapMetadata};

#[test]
fn loads_trinary_ros2_map() {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let yaml_path = manifest_dir.join("tests/fixtures/simple.yaml");

    let grid = RosMapLoader::load_from_yaml(&yaml_path).expect("grid should load");

    assert_eq!(grid.width(), 2);
    assert_eq!(grid.height(), 2);

    assert_eq!(grid.get(&UVec2::new(0, 0)), Some(&FREE));
    assert_eq!(grid.get(&UVec2::new(1, 0)), Some(&OCCUPIED));
    assert_eq!(grid.get(&UVec2::new(0, 1)), Some(&OCCUPIED));
    assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&UNKNOWN));
}

#[test]
fn loads_real_map_fixture() {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let yaml_path = manifest_dir.join("tests/fixtures/warehouse.yaml");
    let image_path = manifest_dir.join("tests/fixtures/warehouse.png");

    let grid = RosMapLoader::load_from_yaml(&yaml_path).expect("grid should load");
    let image = image::open(&image_path).expect("image should load");
    let (width, height) = image.dimensions();

    assert_eq!(grid.width(), width);
    assert_eq!(grid.height(), height);
}

#[test]
fn loads_image_direct() {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let image_path = manifest_dir.join("tests/fixtures/simple.pgm");
    let metadata = RosMapMetadata {
        image: image_path.to_path_buf(),
        ..Default::default()
    };
    let grid = RosMapLoader::load_from_image(&metadata).expect("grid should load");

    assert_eq!(grid.width(), 2);
    assert_eq!(grid.height(), 2);

    assert_eq!(grid.get(&UVec2::new(0, 0)), Some(&FREE));
    assert_eq!(grid.get(&UVec2::new(1, 0)), Some(&OCCUPIED));
    assert_eq!(grid.get(&UVec2::new(0, 1)), Some(&OCCUPIED));
    assert_eq!(grid.get(&UVec2::new(1, 1)), Some(&UNKNOWN));
}
