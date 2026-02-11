//! Nav2-style API translation only. No business logic; wraps core types and
//! converts between Nav2-style signatures (e.g. `get_cost(mx, my)`, `update_map(x, y, yaw)`)
//! and core types (`Grid2d<u8>`, `Pose2`, `LayeredGrid2d`).

use std::time::Duration;

use glam::{UVec2, Vec2};

use crate::{Grid2d, MapInfo, Pose2, types::VoxelError};
use crate::{
    grid::{Layer as CoreLayer, LayeredGrid2d},
    types::{COST_FREE, COST_UNKNOWN},
};

/// Nav2-style robot pose (separate x, y, yaw).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2D {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PoseStamped {
    pub frame_id: String,
    pub pose: Pose2D,
}

/// Nav2-style axis-aligned bounds (separate min_x, min_y, max_x, max_y in meters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

/// Nav2-style footprint: points as (x, y) tuples in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    pub points: Vec<(f32, f32)>,
}

/// Raytrace range constraints in meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaytraceRange {
    pub min: f32,
    pub max: f32,
}

fn pose2d_to_core(p: Pose2D) -> Pose2 {
    Pose2 {
        position: Vec2::new(p.x, p.y),
        yaw: p.yaw,
    }
}

fn bounds_core_to_nav2(min: Vec2, max: Vec2) -> Bounds {
    Bounds {
        min_x: min.x,
        min_y: min.y,
        max_x: max.x,
        max_y: max.y,
    }
}

/// Nav2-style costmap: thin wrapper around `Grid2d<u8>`. All methods translate to core.
#[derive(Debug)]
pub struct Costmap2D {
    grid: Grid2d<u8>,
}

/// Nav2-style view over a reference to the master grid (e.g. from LayeredCostmap).
#[derive(Debug)]
pub struct Costmap2DRef<'a> {
    grid: &'a Grid2d<u8>,
}

impl<'a> Costmap2DRef<'a> {
    pub fn get_cost(&self, mx: u32, my: u32) -> u8 {
        self.grid.get(UVec2::new(mx, my)).copied().unwrap_or(0)
    }

    pub fn map_to_world(&self, mx: u32, my: u32) -> (f32, f32) {
        let pos = self.grid.map_to_world(UVec2::new(mx, my));
        (pos.x, pos.y)
    }

    pub fn world_to_map(&self, wx: f32, wy: f32) -> Option<(u32, u32)> {
        let cell = self.grid.world_to_map(Vec2::new(wx, wy))?;
        Some((cell.x, cell.y))
    }

    pub fn get_size_in_cells_x(&self) -> u32 {
        self.grid.width()
    }

    pub fn get_size_in_cells_y(&self) -> u32 {
        self.grid.height()
    }

    pub fn get_resolution(&self) -> f32 {
        self.grid.info().resolution
    }

    pub fn get_origin(&self) -> (f32, f32) {
        let o = self.grid.info().origin;
        (o.x, o.y)
    }
}

impl Costmap2D {
    /// Construct a new costmap with size in cells, resolution in meters, and origin in meters.
    ///
    /// Expectations:
    /// - `(origin_x, origin_y)` is the world coordinate of cell (0, 0).
    /// - Resolution is meters per cell and must be > 0.
    ///
    /// C++: `Costmap2D(unsigned int size_x, unsigned int size_y, double resolution,`
    /// `double origin_x, double origin_y, unsigned char default_value = 0);`
    pub fn new(size_x: u32, size_y: u32, resolution: f32, origin_x: f32, origin_y: f32) -> Self {
        Self {
            grid: Grid2d::empty(MapInfo {
                width: size_x,
                height: size_y,
                resolution,
                origin: Vec2::new(origin_x, origin_y),
            }),
        }
    }

    /// Return the cost at a cell coordinate.
    ///
    /// Expectations:
    /// - Out-of-bounds access should return a lethal or unknown cost via caller checks.
    ///
    /// C++: `unsigned char getCost(unsigned int mx, unsigned int my) const;`
    pub fn get_cost(&self, mx: u32, my: u32) -> u8 {
        self.grid.get(UVec2::new(mx, my)).copied().unwrap_or(0)
    }

    /// Set the cost at a cell coordinate.
    ///
    /// Expectations:
    /// - Out-of-bounds writes should be ignored or error via caller checks.
    ///
    /// C++: `void setCost(unsigned int mx, unsigned int my, unsigned char cost);`
    pub fn set_cost(&mut self, mx: u32, my: u32, cost: u8) {
        // Explicitly (and silently) ignore any out of bound errors
        let _ = self.grid.set(UVec2::new(mx, my), cost);
    }

    /// Convert map coordinates to world coordinates at cell center.
    ///
    /// C++: `void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;`
    pub fn map_to_world(&self, mx: u32, my: u32) -> (f32, f32) {
        let pos = self.grid.map_to_world(UVec2::new(mx, my));
        (pos.x, pos.y)
    }

    /// Convert world coordinates to map coordinates with bounds checking.
    ///
    /// Expectations:
    /// - Returns `None` if the world coordinate lies outside the map bounds.
    ///
    /// C++: `bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;`
    pub fn world_to_map(&self, wx: f32, wy: f32) -> Option<(u32, u32)> {
        let pos = Vec2::new(wx, wy);
        let cell = self.grid.world_to_map(pos)?;
        Some((cell.x, cell.y))
    }

    /// Convert world coordinates to continuous map coordinates without snapping.
    ///
    /// Expectations:
    /// - Values are in cell coordinates (0..size) and can be fractional.
    ///
    /// C++: `bool worldToMapContinuous(double wx, double wy, float & mx, float & my) const;`
    pub fn world_to_map_continuous(&self, wx: f32, wy: f32) -> Option<(f32, f32)> {
        let pos = Vec2::new(wx, wy);
        let map_pos = self.grid.world_to_map_continuous(pos)?;
        Some((map_pos.x, map_pos.y))
    }

    /// Return the map size in cells along X.
    /// C++: `unsigned int getSizeInCellsX() const;`
    pub fn get_size_in_cells_x(&self) -> u32 {
        self.grid.width()
    }

    /// Return the map size in cells along Y.
    /// C++: `unsigned int getSizeInCellsY() const;`
    pub fn get_size_in_cells_y(&self) -> u32 {
        self.grid.height()
    }

    /// Return the map resolution in meters per cell.
    /// C++: `double getResolution() const;`
    pub fn get_resolution(&self) -> f32 {
        self.grid.info().resolution
    }

    /// Return the map origin (x, y) in meters.
    /// C++: `double getOriginX() const;` and `double getOriginY() const;`
    pub fn get_origin(&self) -> (f32, f32) {
        let origin = self.grid.info().origin;
        (origin.x, origin.y)
    }

    /// Update the map origin in world coordinates.
    ///
    /// Expectations:
    /// - When rolling, attempt to preserve overlapping regions of the previous map.
    ///
    /// C++: `virtual void updateOrigin(double new_origin_x, double new_origin_y);`
    pub fn update_origin(&mut self, origin_x: f32, origin_y: f32) {
        self.grid.update_origin(Vec2::new(origin_x, origin_y));
    }

    /// Resize the map and update resolution/origin.
    ///
    /// Expectations:
    /// - Reallocation clears the map to default value.
    ///
    /// C++: `void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,`
    /// `double origin_x, double origin_y);`
    pub fn resize_map(
        &mut self,
        size_x: u32,
        size_y: u32,
        resolution: f32,
        origin_x: f32,
        origin_y: f32,
    ) {
        self.grid.resize_map(
            UVec2::new(size_x, size_y),
            resolution,
            Vec2::new(origin_x, origin_y),
        );
    }

    /// Reset a rectangular region to default value.
    ///
    /// Expectations:
    /// - Bounds are inclusive on start and exclusive on end indices.
    ///
    /// C++: `void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);`
    pub fn reset_map(&mut self, x0: u32, y0: u32, xn: u32, yn: u32) {
        self.grid.reset_map(UVec2::new(x0, y0), UVec2::new(xn, yn));
    }

    /// Compute footprint collision cost at a pose (LETHAL if collision).
    ///
    /// Expectations:
    /// - Returns the max cost over all cells within the footprint.
    /// - Should treat unknown as lethal if configured to track unknown space.
    ///
    /// C++ (used by Costmap2DROS via FootprintCollisionChecker):
    /// `double footprintCostAtPose(double x, double y, double theta, const Footprint & footprint);`
    pub fn footprint_cost(&self, _pose: Pose2D, _footprint: &Footprint) -> u8 {
        let polygon: Vec<Vec2> = _footprint
            .points
            .iter()
            .map(|(x, y)| Vec2::new(*x, *y))
            .collect();

        let Some(iter) = self.grid.polygon_value(&polygon) else {
            return 0;
        };
        iter.into_iter()
            .fold(0, |max_cost, cell| max_cost.max(*cell))
    }

    /// Rasterize a convex polygon and set costs; returns false on failure.
    ///
    /// Expectations:
    /// - Polygon points are in world coordinates (meters), matching Nav2's API.
    /// - The polygon is converted to map coordinates internally, similar to Nav2's worldToMap.
    ///
    /// C++: `bool setConvexPolygonCost(const std::vector<geometry_msgs::msg::Point> & polygon,`
    /// `unsigned char cost_value);`
    pub fn set_convex_polygon_cost(&mut self, _polygon: &[(f32, f32)], cost: u8) -> bool {
        let polygon: Vec<Vec2> = _polygon.iter().map(|(x, y)| Vec2::new(*x, *y)).collect();

        let Some(iter) = self.grid.polygon_value_mut(&polygon) else {
            return false;
        };
        for cell in iter {
            *cell = cost;
        }
        true
    }
}

/// Nav2-style layered costmap: thin wrapper around core `LayeredGrid2d`.
/// All logic is in the core; this only converts pose/bounds API.
pub struct LayeredCostmap {
    layered: LayeredGrid2d,
}

impl LayeredCostmap {
    /// Create from a Nav2-style costmap (takes ownership of its grid).
    pub fn new(info: MapInfo, rolling_window: bool, track_unknown: bool) -> Self {
        let fill = if track_unknown {
            COST_UNKNOWN
        } else {
            COST_FREE
        };
        Self {
            layered: LayeredGrid2d::new(info, fill, rolling_window),
        }
    }

    /// Add a core layer. Order is preserved.
    pub fn add_plugin(&mut self, plugin: Box<dyn CoreLayer>) {
        self.layered.add_layer(plugin);
    }

    /// Run the update loop; converts Nav2 pose to core and delegates.
    pub fn update_map(&mut self, robot: Pose2D) {
        self.layered.update_map(pose2d_to_core(robot));
    }

    /// Returns true if configured as rolling window.
    pub fn is_rolling(&self) -> bool {
        self.layered.is_rolling_window()
    }

    /// Reference to the master grid with Nav2-style API.
    pub fn get_costmap(&self) -> Costmap2DRef<'_> {
        Costmap2DRef {
            grid: self.layered.master(),
        }
    }

    /// Mutable reference to the core layered grid (for direct access if needed).
    pub fn layered_mut(&mut self) -> &mut LayeredGrid2d {
        &mut self.layered
    }

    /// Bounds from the last update (Nav2-style min_x, min_y, max_x, max_y).
    pub fn get_updated_bounds(&self) -> Bounds {
        let b = self.layered.updated_bounds();
        bounds_core_to_nav2(b.min, b.max)
    }

    /// Whether all layers are current.
    /// C++: `bool isCurrent();`
    pub fn is_current(&self) -> bool {
        todo!("is_current not implemented")
    }

    /// Set the robot footprint and notify layers.
    /// C++: `void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec);`
    pub fn set_footprint(&mut self, _footprint: Footprint) {
        todo!("set_footprint not implemented")
    }
}

impl Costmap2D {
    /// Access the underlying grid (for use when building LayeredCostmap).
    pub fn into_grid(self) -> Grid2d<u8> {
        self.grid
    }
}

/// ROS wrapper expectations for a Costmap2D-like API.
///
/// Expectations:
/// - Provides lifecycle-like update scheduling and TF usage in the real system.
pub struct Costmap2DROS {
    layered: LayeredCostmap,
}

impl Costmap2DROS {
    /// Create a wrapper around a layered costmap.
    /// C++: `Costmap2DROS(const std::string & name, const std::string & parent_namespace,`
    /// `const bool & use_sim_time, const rclcpp::NodeOptions & options);`
    pub fn new(layered: LayeredCostmap) -> Self {
        Self { layered }
    }

    /// Update the costmap using the latest robot pose and layers.
    /// C++: `void updateMap();`
    pub fn update_map(&mut self) {
        todo!("update_map not implemented")
    }

    /// Reset all layers to default state.
    /// C++: `void resetLayers();`
    pub fn reset_layers(&mut self) {
        todo!("reset_layers not implemented")
    }

    /// Block until the map is current or timeout is reached.
    /// C++: `void waitUntilCurrent(const rclcpp::Duration & timeout);`
    pub fn wait_until_current(&self, _timeout: Duration) -> Result<(), VoxelError> {
        todo!("wait_until_current not implemented")
    }

    /// Return the robot pose in the global frame.
    /// C++: `bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);`
    pub fn get_robot_pose(&self) -> Result<PoseStamped, VoxelError> {
        todo!("get_robot_pose not implemented")
    }

    /// Transform a pose into the global frame.
    /// C++: `bool transformPoseToGlobalFrame(...)`
    pub fn transform_pose_to_global_frame(
        &self,
        _pose: &PoseStamped,
    ) -> Result<PoseStamped, VoxelError> {
        todo!("transform_pose_to_global_frame not implemented")
    }

    /// Return a Nav2-style view over the master costmap.
    pub fn get_costmap(&self) -> Costmap2DRef<'_> {
        self.layered.get_costmap()
    }

    /// Return the layered costmap for plugin access.
    /// C++: `LayeredCostmap * getLayeredCostmap();`
    pub fn get_layered_costmap(&self) -> &LayeredCostmap {
        &self.layered
    }
}

/// Raytrace a line in grid coordinates, returning all visited cells.
///
/// Expectations:
/// - Uses Bresenham or DDA to include both start and end cells.
/// - Applies min/max range constraints in cell units.
pub fn raytrace_line_2d(
    _start: (u32, u32),
    _end: (u32, u32),
    _max_range: u32,
    _min_range: u32,
) -> Vec<(u32, u32)> {
    todo!("raytrace_line_2d not implemented")
}

/// Compute inflation cost for a given distance.
///
/// Expectations:
/// - Returns LETHAL at distance 0, then decays with scaling factor.
/// - Matches Nav2 inflation curve to be drop-in compatible.
///
/// C++ (nav2_costmap_2d/costmap_math.hpp):
/// `unsigned char computeCost(double distance, double inscribed_radius, double cost_scaling_factor);`
pub fn inflate_cost(distance: f32, inscribed_radius: f32, cost_scaling_factor: f32) -> u8 {
    crate::layers::inflation::inscribed_inflation_cost(
        distance,
        inscribed_radius,
        cost_scaling_factor,
    )
}

/// Convert inflation radius in meters to a cell count.
///
/// Expectations:
/// - Typically `ceil(radius / resolution)`.
///
/// C++: `unsigned int cellDistance(double world_dist);`
pub fn inflation_radius_to_cells(inflation_radius: f32, resolution: f32) -> u32 {
    crate::layers::inflation::inflation_radius_to_cells(inflation_radius, resolution)
}
