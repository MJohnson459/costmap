//! Bridge from `costmap` grids to ROS 2 / Nav2.
//!
//! The crate publishes windows of a large costmap as `nav_msgs/OccupancyGrid`
//! for Nav2's `StaticLayer` to consume. Middleware is Hiroz (ROS 2 on Zenoh),
//! which interoperates with ROS 2 nodes running `rmw_zenoh_cpp`.
//!
//! - [`convert`]: `Costmap` ↔ `nav_msgs/OccupancyGrid`.
//! - [`source`]: the [`MapSource`] trait that supplies windows of a map, for
//!   dense maps in memory and tiled GeoTIFFs read on demand.
//! - [`republish`]: when the window server publishes a new window.
//! - [`route`]: routes across a whole cost GeoTIFF, planned on an overview.
//! - [`tf`]: a latest-value transform tree and its `/tf` listener.

pub mod convert;
pub mod nav2_msgs;
pub mod republish;
pub mod route;
pub mod source;
pub mod tf;

pub use convert::{costmap_from_occupancy_grid, occupancy_grid_from_costmap};
#[doc(hidden)]
pub use hiroz_msgs::size_estimation;
pub use republish::RepublishPolicy;
pub use route::{Route, RouteError, RouteOptions, RoutePlanner};
pub use source::{GeoTiffSource, MapSource, SourceError, TileStats};
