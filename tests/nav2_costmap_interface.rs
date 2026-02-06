#![cfg(feature = "nav2_compat")]

use std::time::Duration;

use costmap::nav2_compat::*;

struct DummyLayer;

impl Layer for DummyLayer {
    fn reset(&mut self) {
        todo!("layer reset not implemented");
    }

    fn is_clearable(&self) -> bool {
        todo!("layer is_clearable not implemented");
    }

    fn update_bounds(&mut self, _robot: Pose2D, _bounds: &mut Bounds) {
        todo!("layer update_bounds not implemented");
    }

    fn update_costs(
        &mut self,
        _master: &mut Costmap2D,
        _min_i: u32,
        _min_j: u32,
        _max_i: u32,
        _max_j: u32,
    ) {
        todo!("layer update_costs not implemented");
    }
}

#[test]
fn costmap_basic_access() {
    let mut map = Costmap2D::new(10, 10, 0.1, 0.0, 0.0);
    map.set_cost(2, 3, 100);
    assert_eq!(map.get_cost(2, 3), 100);
}

#[test]
fn costmap_world_map_conversions() {
    let map = Costmap2D::new(10, 10, 0.5, -1.0, -2.0);
    let (wx, wy) = map.map_to_world(2, 4);
    let (mx, my) = map.world_to_map(wx, wy).expect("in bounds");
    assert_eq!((mx, my), (2, 4));
}

#[test]
fn costmap_continuous_world_to_map() {
    let map = Costmap2D::new(10, 10, 1.0, 0.0, 0.0);
    let (mx, my) = map.world_to_map_continuous(2.2, 3.7).expect("in bounds");
    assert!((mx - 2.2).abs() < 1e-3);
    assert!((my - 3.7).abs() < 1e-3);
}

#[test]
fn costmap_resize_and_origin_shift() {
    let mut map = Costmap2D::new(10, 10, 0.1, 0.0, 0.0);
    map.update_origin(1.0, 2.0);
    map.resize_map(20, 20, 0.05, -1.0, -1.0);
    assert_eq!(map.get_size_in_cells_x(), 20);
    assert_eq!(map.get_size_in_cells_y(), 20);
}

#[test]
fn layered_costmap_plugin_update_flow() {
    let base = Costmap2D::new(5, 5, 0.2, 0.0, 0.0);
    let mut layered = LayeredCostmap::new(base, true);
    layered.add_plugin(Box::new(DummyLayer));
    layered.update_map(Pose2D {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    });
    assert!(layered.is_current());
}

#[test]
fn layered_costmap_footprint_and_bounds() {
    let base = Costmap2D::new(5, 5, 0.2, 0.0, 0.0);
    let mut layered = LayeredCostmap::new(base, false);
    layered.set_footprint(Footprint {
        points: vec![(0.2, 0.2), (-0.2, 0.2), (-0.2, -0.2), (0.2, -0.2)],
    });
    let bounds = layered.get_updated_bounds();
    assert!(bounds.max_x >= bounds.min_x);
    assert!(bounds.max_y >= bounds.min_y);
}

#[test]
fn costmap_ros_update_and_queries() {
    let base = Costmap2D::new(5, 5, 0.2, 0.0, 0.0);
    let layered = LayeredCostmap::new(base, true);
    let mut ros = Costmap2DROS::new(layered);
    ros.update_map();

    let pose = ros.get_robot_pose().expect("pose");
    let _ = ros
        .transform_pose_to_global_frame(&pose)
        .expect("transform");
    ros.wait_until_current(Duration::from_millis(100))
        .expect("current");
}

#[test]
fn footprint_collision_cost() {
    let map = Costmap2D::new(10, 10, 0.1, 0.0, 0.0);
    let footprint = Footprint {
        points: vec![(0.2, 0.2), (-0.2, 0.2), (-0.2, -0.2), (0.2, -0.2)],
    };
    let cost = map.footprint_cost(
        Pose2D {
            x: 1.0,
            y: 2.0,
            yaw: 0.0,
        },
        &footprint,
    );
    assert!(cost <= 254);
}

#[test]
fn polygon_cost_setting() {
    let mut map = Costmap2D::new(10, 10, 0.1, 0.0, 0.0);
    // With resolution 0.1 and origin (0.0, 0.0), world coordinates (0.1, 0.1) to (0.2, 0.2)
    // should cover map cells (1,1) to (2,2)
    let polygon = vec![(0.1, 0.1), (0.2, 0.1), (0.2, 0.2), (0.1, 0.2)];
    assert!(map.set_convex_polygon_cost(&polygon, 200));

    assert_eq!(map.get_cost(1, 1), 200);
    assert_eq!(map.get_cost(2, 1), 200);
    assert_eq!(map.get_cost(2, 2), 200);
    assert_eq!(map.get_cost(1, 2), 200);
}

#[test]
fn raytrace_clearing_paths() {
    let cells = raytrace_line_2d((1, 1), (8, 4), 50, 0);
    assert!(!cells.is_empty());
    assert_eq!(cells.first().copied(), Some((1, 1)));
    assert_eq!(cells.last().copied(), Some((8, 4)));
}

#[test]
fn inflation_cost_falloff() {
    let inscribed = 0.2;
    let scaling = 10.0;
    let cost_near = inflate_cost(0.0, inscribed, scaling);
    let cost_far = inflate_cost(0.5, inscribed, scaling);
    assert!(cost_near >= cost_far);
}

#[test]
fn inflation_radius_to_cell_distance() {
    let radius_cells = inflation_radius_to_cells(0.55, 0.05);
    assert!(radius_cells > 0);
}
