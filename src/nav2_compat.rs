//! Minimal compatibility surface for a Costmap2D-like API.
//!
//! This module intentionally contains unimplemented stubs to document the
//! interface expected by Nav2-style costmaps. Tests are written against these
//! stubs to guide future implementation work.

use std::time::Duration;

use glam::{UVec2, Vec2, Vec3};

use crate::{Grid2d, MapInfo, types::VoxelError};

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

/// Axis-aligned bounds in world coordinates (meters).
///
/// Expectations:
/// - Bounds are inclusive on min and exclusive on max for grid index conversions.
/// - Bounds should be expanded by layers when they affect space outside their immediate update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

/// Footprint polygon in world coordinates (meters).
///
/// Expectations:
/// - Points are ordered and form a convex polygon (as in costmap2d).
/// - Units are meters in the global costmap frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    pub points: Vec<(f32, f32)>,
}

/// Raytrace range constraints in meters.
///
/// Expectations:
/// - `min` is the minimum ray length; hits closer than this should be ignored.
/// - `max` is the maximum ray length; rays should be clamped to this distance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaytraceRange {
    pub min: f32,
    pub max: f32,
}

/// Minimal layer interface expected by a layered costmap update loop.
///
/// Expectations:
/// - `update_bounds` expands bounds in world coordinates that `update_costs` will mutate.
/// - `update_costs` only writes within `[min_i, max_i)` and `[min_j, max_j)` indices.
/// - `reset` should restore the layer to default (typically unknown or free).
/// - `is_clearable` indicates whether global clearing actions should reset the layer.
pub trait Layer {
    /// Reset the layer to its initial state.
    /// C++: `virtual void reset() = 0;`
    fn reset(&mut self);
    /// Whether this layer should be cleared by global clear requests.
    /// C++: `virtual bool isClearable() = 0;`
    fn is_clearable(&self) -> bool;
    /// Expand update bounds for this layer given the robot pose.
    /// C++: `virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,`
    /// `double * min_x, double * min_y, double * max_x, double * max_y) = 0;`
    fn update_bounds(&mut self, robot: Pose2D, bounds: &mut Bounds);
    /// Update costs for this layer within the given map window.
    /// C++: `virtual void updateCosts(Costmap2D & master_grid, int min_i, int min_j,`
    /// `int max_i, int max_j) = 0;`
    fn update_costs(
        &mut self,
        master: &mut Costmap2D,
        min_i: u32,
        min_j: u32,
        max_i: u32,
        max_j: u32,
    );
    /// Callback for footprint updates. Layers can invalidate caches here.
    /// C++: `virtual void onFootprintChanged();`
    fn on_footprint_changed(&mut self, footprint: &Footprint) {
        let _ = footprint;
    }
}

/// A Costmap2D-like grid with cost values in [0, 254].
///
/// Expectations:
/// - Costs are unsigned 8-bit values where 0 is FREE and 254 is LETHAL.
/// - Unknown should be encoded by a configurable sentinel (often 255 or 253 in ROS2).
/// - World coordinates are in meters (f32), map coordinates are integer cell indices.
#[derive(Debug)]
pub struct Costmap2D {
    grid: Grid2d<u8>,
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
                depth: 1,
                resolution: resolution,
                origin: Vec2::new(origin_x, origin_y).extend(0.0),
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
        self.grid.get(&UVec2::new(mx, my)).copied().unwrap_or(0)
    }

    /// Set the cost at a cell coordinate.
    ///
    /// Expectations:
    /// - Out-of-bounds writes should be ignored or error via caller checks.
    ///
    /// C++: `void setCost(unsigned int mx, unsigned int my, unsigned char cost);`
    pub fn set_cost(&mut self, mx: u32, my: u32, cost: u8) {
        // Explicitly (and silently) ignore any out of bound errors
        let _ = self.grid.set(&UVec2::new(mx, my), cost);
    }

    /// Convert map coordinates to world coordinates at cell center.
    ///
    /// Expectations:
    /// - Returns the center of the specified cell, not the lower-left corner.
    ///
    /// C++: `void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;`
    pub fn map_to_world(&self, mx: u32, my: u32) -> (f32, f32) {
        let pos = self.grid.map_to_world(&Vec2::new(mx as f32, my as f32))
            + 0.5 * self.grid.info().resolution;
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
        let map_pos = self.grid.world_to_map(&pos)?;
        Some((map_pos.x as u32, map_pos.y as u32))
    }

    /// Convert world coordinates to continuous map coordinates without snapping.
    ///
    /// Expectations:
    /// - Values are in cell coordinates (0..size) and can be fractional.
    ///
    /// C++: `bool worldToMapContinuous(double wx, double wy, float & mx, float & my) const;`
    pub fn world_to_map_continuous(&self, wx: f32, wy: f32) -> Option<(f32, f32)> {
        let pos = Vec2::new(wx, wy);
        let map_pos = self.grid.world_to_map(&pos)?;
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
        self.grid
            .update_origin(&Vec3::new(origin_x, origin_y, self.grid.info().origin.z));
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
            &Vec3::new(origin_x, origin_y, self.grid.info().origin.z),
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

/// Aggregates multiple layers into a single master costmap.
///
/// Expectations:
/// - `update_map` runs `update_bounds` then `update_costs` across all layers.
/// - Bounds accumulation uses world coordinates and then converts to map indices.
pub struct LayeredCostmap {
    plugins: Vec<Box<dyn Layer>>,
    costmap: Costmap2D,
    rolling_window: bool,
}

impl LayeredCostmap {
    /// Create a layered costmap around a base grid.
    /// C++: `LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);`
    pub fn new(costmap: Costmap2D, rolling_window: bool) -> Self {
        Self {
            plugins: Vec::new(),
            costmap,
            rolling_window,
        }
    }

    /// Add a layer plugin to the update loop.
    ///
    /// Expectations:
    /// - Layers should be stored in order and processed in that order.
    ///
    /// C++: `void addPlugin(std::shared_ptr<Layer> plugin);`
    pub fn add_plugin(&mut self, plugin: Box<dyn Layer>) {
        let _ = plugin;
        todo!("add_plugin not implemented");
    }

    /// Run update bounds/costs across layers for a robot pose.
    ///
    /// Expectations:
    /// - Uses a common bounds window for all layers (after expansion).
    ///
    /// C++: `void updateMap(double robot_x, double robot_y, double robot_yaw);`
    pub fn update_map(&mut self, _robot: Pose2D) {
        todo!("update_map not implemented");
    }

    /// Return whether all layers are current.
    /// C++: `bool isCurrent();`
    pub fn is_current(&self) -> bool {
        todo!("is_current not implemented");
    }

    /// Returns true if the map is configured as rolling.
    /// C++: `bool isRolling();`
    pub fn is_rolling(&self) -> bool {
        self.rolling_window
    }

    /// Resize the master map and propagate to layers.
    ///
    /// Expectations:
    /// - Calls `matchSize` / equivalent on each layer.
    ///
    /// C++: `void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,`
    /// `double origin_x, double origin_y, bool size_locked = false);`
    pub fn resize_map(
        &mut self,
        _size_x: u32,
        _size_y: u32,
        _resolution: f32,
        _origin_x: f32,
        _origin_y: f32,
    ) {
        todo!("resize_map not implemented");
    }

    /// Return an immutable reference to the master costmap.
    /// C++: `Costmap2D * getCostmap();`
    pub fn get_costmap(&self) -> &Costmap2D {
        &self.costmap
    }

    /// Return a mutable reference to the master costmap.
    /// C++: `Costmap2D * getCostmap();`
    pub fn get_costmap_mut(&mut self) -> &mut Costmap2D {
        &mut self.costmap
    }

    /// Set the robot footprint and notify layers.
    ///
    /// Expectations:
    /// - Updates inscribed/circumscribed radii if tracked.
    ///
    /// C++: `void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec);`
    pub fn set_footprint(&mut self, _footprint: Footprint) {
        todo!("set_footprint not implemented");
    }

    /// Get the bounds updated by the last update cycle.
    ///
    /// Expectations:
    /// - Returns world-coordinate bounds last computed during update.
    ///
    /// C++: `void getUpdatedBounds(double & minx, double & miny, double & maxx, double & maxy);`
    pub fn get_updated_bounds(&self) -> Bounds {
        todo!("get_updated_bounds not implemented");
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
    ///
    /// Expectations:
    /// - Equivalent to `LayeredCostmap::update_map` plus footprint publication.
    ///
    /// C++: `void updateMap();`
    pub fn update_map(&mut self) {
        todo!("update_map not implemented");
    }

    /// Reset all layers to default state.
    /// C++: `void resetLayers();`
    pub fn reset_layers(&mut self) {
        todo!("reset_layers not implemented");
    }

    /// Block until the map is current or timeout is reached.
    ///
    /// Expectations:
    /// - Throws/errs on timeout.
    ///
    /// C++: `void waitUntilCurrent(const rclcpp::Duration & timeout);`
    pub fn wait_until_current(&self, _timeout: Duration) -> Result<(), VoxelError> {
        todo!("wait_until_current not implemented");
    }

    /// Return the robot pose in the global frame.
    ///
    /// Expectations:
    /// - Uses TF buffer in a real implementation.
    ///
    /// C++: `bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);`
    pub fn get_robot_pose(&self) -> Result<PoseStamped, VoxelError> {
        todo!("get_robot_pose not implemented");
    }

    /// Transform a pose into the global frame.
    ///
    /// C++: `bool transformPoseToGlobalFrame(const geometry_msgs::msg::PoseStamped & input_pose,`
    /// `geometry_msgs::msg::PoseStamped & transformed_pose);`
    pub fn transform_pose_to_global_frame(
        &self,
        _pose: &PoseStamped,
    ) -> Result<PoseStamped, VoxelError> {
        todo!("transform_pose_to_global_frame not implemented");
    }

    /// Return the master costmap for queries.
    /// C++: `Costmap2D * getCostmap();`
    pub fn get_costmap(&self) -> &Costmap2D {
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
    todo!("raytrace_line_2d not implemented");
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
    crate::inflation::inscribed_inflation_cost(distance, inscribed_radius, cost_scaling_factor)
}

/// Convert inflation radius in meters to a cell count.
///
/// Expectations:
/// - Typically `ceil(radius / resolution)`.
///
/// C++: `unsigned int cellDistance(double world_dist);`
pub fn inflation_radius_to_cells(inflation_radius: f32, resolution: f32) -> u32 {
    crate::inflation::inflation_radius_to_cells(inflation_radius, resolution)
}
