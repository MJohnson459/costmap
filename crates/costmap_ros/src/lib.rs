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
//! - [`route`]: routes across a whole cost GeoTIFF, planned on a graph of its tiles.
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

#[cfg(test)]
mod tests {
    /// `package.xml` and the conda recipe each state the version again.
    #[test]
    fn packaging_files_carry_the_crate_version() {
        let version = env!("CARGO_PKG_VERSION");
        let package_xml = include_str!("../package.xml");
        assert!(
            package_xml.contains(&format!("<version>{version}</version>")),
            "package.xml does not carry version {version}"
        );
        let recipe = include_str!("../conda/recipe.yaml");
        assert!(
            recipe.contains(&format!("version: \"{version}\"")),
            "conda/recipe.yaml does not carry version {version}"
        );
    }
}
