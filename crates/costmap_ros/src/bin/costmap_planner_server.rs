//! Plans routes across a whole map for Nav2.
//!
//! Nav2's own planner searches its global costmap, so it cannot reach a goal
//! outside the window it holds. This node serves an action of the same type,
//! `nav2_msgs/action/ComputePathToPose`, under the name
//! `compute_route_to_pose`, and answers with a path across the whole map,
//! planned on a graph of the map's tiles. A behaviour tree asks it for the
//! route and Nav2's planner for the stretch ahead; see
//! `behavior_trees/navigate_to_pose_on_route.xml`.
//!
//! `costmap_window_server --routes` serves the same action from the process
//! that publishes the map window. This program is for running the two apart.
//!
//! Started with `--action compute_path_to_pose` it takes the place of
//! `planner_server` outright, for a robot that needs none of Nav2's planners.

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, anyhow};
use clap::Parser;
use costmap_ros::RouteOptions;
use costmap_ros::pose::RobotPose;
use costmap_ros::route_server::{RouteServerOptions, open_routes, serve_routes};
use hiroz::Builder;
use hiroz::context::ZContextBuilder;
use tracing::info;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(about = "Plan routes for Nav2 across a whole map")]
struct Args {
    /// Map file, chosen by extension: .yaml/.yml is a ROS map held in memory;
    /// .tif/.tiff is a single-band u8 cost GeoTIFF read a tile at a time, with
    /// the map frame at its lower-left corner.
    #[arg(long)]
    map: PathBuf,
    /// Action to serve.
    #[arg(long, default_value = "compute_route_to_pose")]
    action: String,
    /// Topic each route is also published on.
    #[arg(long, default_value = "route")]
    route_topic: String,
    /// Frame of the map and of the planned paths.
    #[arg(long, default_value = "map")]
    map_frame: String,
    /// Robot frame a path starts at when the goal gives no start.
    #[arg(long, default_value = "base_link")]
    base_frame: String,
    /// Topic of geometry_msgs/PoseWithCovarianceStamped messages that give the
    /// robot's pose in the map frame, such as AMCL's amcl_pose. With it, /tf
    /// is not subscribed to and --base-frame is not used. Empty means /tf.
    #[arg(long, default_value = "")]
    pose_topic: String,
    /// Robot radius in metres.
    #[arg(long, default_value_t = RouteOptions::default().robot_radius_m)]
    robot_radius_m: f32,
    /// Distance from obstacles over which paths are pushed away, in metres.
    #[arg(long, default_value_t = RouteOptions::default().inflation_m)]
    inflation_m: f32,
}

async fn run(args: Args) -> anyhow::Result<()> {
    let options = RouteOptions {
        robot_radius_m: args.robot_radius_m,
        inflation_m: args.inflation_m,
        ..RouteOptions::default()
    };
    let routes = open_routes(&args.map, options)?;

    let ctx = ZContextBuilder::default()
        .build()
        .map_err(|e| anyhow!("opening Zenoh session: {e}"))?;
    let node = ctx
        .create_node("costmap_planner_server")
        .build()
        .map_err(|e| anyhow!("creating node: {e}"))?;
    let pose = if args.pose_topic.is_empty() {
        RobotPose::from_tf(&node, &args.map_frame, &args.base_frame)
            .map_err(|e| anyhow!("subscribing to tf: {e}"))?
    } else {
        RobotPose::from_topic(&node, &args.pose_topic, &args.map_frame)
            .map_err(|e| anyhow!("subscribing to {}: {e}", args.pose_topic))?
    };
    let pose = Arc::new(pose);

    // Dropping the server ends it, so it is held until shutdown.
    let _server = serve_routes(
        &node,
        pose,
        routes,
        RouteServerOptions {
            action: args.action,
            route_topic: args.route_topic,
            map_frame: args.map_frame,
        },
    )?;

    tokio::signal::ctrl_c()
        .await
        .context("waiting for shutdown")?;
    info!("shutting down");
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let default_filter = "warn,costmap_planner_server=info,costmap_ros=info";
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| default_filter.into()),
        )
        .init();
    let args = Args::parse();
    tokio::runtime::Runtime::new()
        .context("starting the async runtime")?
        .block_on(run(args))
}
