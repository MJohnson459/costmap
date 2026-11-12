//! Publishes the part of a map around the robot as `nav_msgs/OccupancyGrid`.
//!
//! The map is a ROS map YAML, held whole in memory, or a cost GeoTIFF, of
//! which only the chunks near the robot are held.
//!
//! The robot pose is the latest `map_frame -> base_frame` transform. A new
//! window goes out when the robot has moved `--min-move-m` since the last one,
//! and `--keep-alive-s` after it if that is set.
//!
//! With `--routes` the process also plans routes across the whole map, as
//! `costmap_planner_server` does, on the same `/tf` subscription.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, anyhow};
use clap::Parser;
use costmap::types::Pose2;
use costmap::{RosMapLoader, costmap_from_occupancy};
use costmap_ros::republish::LastPublish;
use costmap_ros::route_server::{RouteServerOptions, open_routes, serve_routes};
use costmap_ros::tf::TfListener;
use costmap_ros::{
    GeoTiffSource, MapSource, RepublishPolicy, RouteOptions, occupancy_grid_from_costmap,
};
use glam::UVec2;
use hiroz::Builder;
use hiroz::context::ZContextBuilder;
use hiroz::msg::ZMessage;
use hiroz::pubsub::ZPub;
use hiroz::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use hiroz_msgs::builtin_interfaces::Time;
use hiroz_msgs::nav_msgs::OccupancyGrid;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

type GridPub = ZPub<OccupancyGrid, <OccupancyGrid as ZMessage>::Serdes>;

#[derive(Debug, Parser)]
#[command(about = "Publish a rolling window of a map for Nav2's StaticLayer")]
struct Args {
    /// Map file, chosen by extension: .yaml/.yml is a ROS map (image,
    /// resolution, origin, thresholds) held in memory; .tif/.tiff is a
    /// single-band u8 cost GeoTIFF of which only the chunks near the robot
    /// are held, with the map frame at its lower-left corner.
    #[arg(long)]
    map: PathBuf,
    /// Side length of the published square window in metres.
    #[arg(long, default_value_t = 50.0)]
    window_m: f32,
    /// Topic to publish on.
    #[arg(long, default_value = "map")]
    topic: String,
    /// Frame of the map and of the published grid.
    #[arg(long, default_value = "map")]
    map_frame: String,
    /// Robot frame the window is centred on.
    #[arg(long, default_value = "base_link")]
    base_frame: String,
    /// How often the robot pose is checked, in Hz.
    #[arg(long, default_value_t = 2.0)]
    rate_hz: f32,
    /// Distance the robot moves before a new window is published.
    #[arg(long, default_value_t = 5.0)]
    min_move_m: f32,
    /// Seconds after which the window is republished even if the robot is
    /// still; 0 never republishes it. The topic is transient local, so a late
    /// subscriber receives the last window either way.
    #[arg(long, default_value_t = 0.0)]
    keep_alive_s: f32,
    /// Also plan routes across the whole map, as costmap_planner_server does.
    #[arg(long)]
    routes: bool,
    /// With --routes: action the routes are served on.
    #[arg(long, default_value = "compute_route_to_pose")]
    route_action: String,
    /// With --routes: topic each route is also published on.
    #[arg(long, default_value = "route")]
    route_topic: String,
    /// With --routes: robot radius in metres.
    #[arg(long, default_value_t = RouteOptions::default().robot_radius_m)]
    robot_radius_m: f32,
    /// With --routes: distance from obstacles over which routes are pushed
    /// away, in metres.
    #[arg(long, default_value_t = RouteOptions::default().inflation_m)]
    inflation_m: f32,
}

fn main() -> anyhow::Result<()> {
    // Hiroz logs every endpoint it declares at info, so it is held at warn.
    let default_filter = "warn,costmap_window_server=info,costmap_ros=info";
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| default_filter.into()),
        )
        .init();
    let args = Args::parse();
    let period = positive_seconds(1.0 / args.rate_hz, "--rate-hz")?;
    if !(args.min_move_m.is_finite() && args.min_move_m >= 0.0) {
        return Err(anyhow!(
            "--min-move-m must be a finite, non-negative distance"
        ));
    }
    let policy = RepublishPolicy {
        min_move_m: args.min_move_m,
        keep_alive: if args.keep_alive_s == 0.0 {
            None
        } else {
            Some(positive_seconds(args.keep_alive_s, "--keep-alive-s")?)
        },
    };

    let mut map = open_map(&args.map)?;
    let size = window_cells(args.window_m, map.resolution())?;

    let ctx = ZContextBuilder::default()
        .build()
        .map_err(|e| anyhow!("opening Zenoh session: {e}"))?;
    let node = ctx
        .create_node("costmap_window_server")
        .build()
        .map_err(|e| anyhow!("creating node: {e}"))?;
    let publisher = node
        .create_pub::<OccupancyGrid>(&args.topic)
        .with_qos(map_qos())
        .build()
        .map_err(|e| anyhow!("creating publisher on {}: {e}", args.topic))?;
    let tf = Arc::new(TfListener::new(&node).map_err(|e| anyhow!("subscribing to tf: {e}"))?);

    // Route goals run on this runtime's threads. The runtime and the server
    // are held until shutdown; dropping the server ends it.
    let runtime = tokio::runtime::Runtime::new().context("starting the async runtime")?;
    let _route_server = if args.routes {
        let options = RouteOptions {
            robot_radius_m: args.robot_radius_m,
            inflation_m: args.inflation_m,
            ..RouteOptions::default()
        };
        let routes = open_routes(&args.map, options)?;
        let _in_runtime = runtime.enter();
        Some(serve_routes(
            &node,
            Arc::clone(&tf),
            routes,
            RouteServerOptions {
                action: args.route_action.clone(),
                route_topic: args.route_topic.clone(),
                map_frame: args.map_frame.clone(),
                base_frame: args.base_frame.clone(),
            },
        )?)
    } else {
        None
    };

    let (stop_tx, stop_rx) = mpsc::channel();
    ctrlc::set_handler(move || {
        // A send fails only once the loop has exited, which is the goal.
        let _ = stop_tx.send(());
    })
    .context("installing signal handler")?;

    info!(
        "serving {} ({} m per cell) on {} as {}x{}-cell windows; robot {} in {}, \
         check {} Hz, republish after {} m{}",
        args.map.display(),
        map.resolution(),
        args.topic,
        size.x,
        size.y,
        args.base_frame,
        args.map_frame,
        args.rate_hz,
        policy.min_move_m,
        policy
            .keep_alive
            .map_or(String::new(), |age| format!(" or {} s", age.as_secs_f32())),
    );

    let mut last: Option<LastPublish> = None;
    let mut pose_ok = true;
    let mut next_tick = Instant::now();
    loop {
        next_tick = (next_tick + period).max(Instant::now());
        match stop_rx.recv_timeout(next_tick.saturating_duration_since(Instant::now())) {
            Err(RecvTimeoutError::Timeout) => {}
            Ok(()) | Err(RecvTimeoutError::Disconnected) => break,
        }
        let pose = match tf.lookup(&args.map_frame, &args.base_frame) {
            Ok(transform) => transform.planar_pose(),
            Err(err) => {
                if pose_ok {
                    warn!("no robot pose, not publishing: {err}");
                }
                pose_ok = false;
                continue;
            }
        };
        if !pose_ok {
            info!("robot pose available");
            pose_ok = true;
        }
        let now = Instant::now();
        if !policy.is_due(last.as_ref(), pose.position, now) {
            continue;
        }
        match publish_window(&publisher, map.as_mut(), &args.map_frame, pose, size) {
            Ok(()) => {
                last = Some(LastPublish {
                    center: pose.position,
                    at: now,
                })
            }
            Err(err) => warn!("publishing window failed: {err}"),
        }
    }
    info!("shutting down");
    Ok(())
}

/// Map file formats `--map` accepts, by extension.
const MAP_EXTENSIONS: &str = ".yaml, .yml (ROS map), .tif, .tiff (cost GeoTIFF)";

enum MapKind {
    RosYaml,
    GeoTiff,
}

impl MapKind {
    fn of(path: &Path) -> anyhow::Result<Self> {
        let extension = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase);
        match extension.as_deref() {
            Some("yaml" | "yml") => Ok(Self::RosYaml),
            Some("tif" | "tiff") => Ok(Self::GeoTiff),
            _ => Err(anyhow!(
                "--map {}: unsupported extension; accepted: {MAP_EXTENSIONS}",
                path.display()
            )),
        }
    }
}

fn open_map(path: &Path) -> anyhow::Result<Box<dyn MapSource>> {
    let source: Box<dyn MapSource> = match MapKind::of(path)? {
        MapKind::RosYaml => {
            let occupancy = RosMapLoader::load_from_yaml(path)
                .with_context(|| format!("loading map {}", path.display()))?;
            Box::new(costmap_from_occupancy(&occupancy).context("converting map to costs")?)
        }
        MapKind::GeoTiff => Box::new(
            GeoTiffSource::open(path)
                .with_context(|| format!("opening GeoTIFF {}", path.display()))?,
        ),
    };
    Ok(source)
}

/// Reliable, transient local, depth 1: what `StaticLayer` subscribes with
/// when `map_subscribe_transient_local` is true, so a late Nav2 gets the
/// current window.
fn map_qos() -> QosProfile {
    QosProfile {
        reliability: QosReliability::Reliable,
        durability: QosDurability::TransientLocal,
        history: QosHistory::KeepLast(std::num::NonZeroUsize::MIN),
        ..Default::default()
    }
}

fn publish_window(
    publisher: &GridPub,
    source: &mut dyn MapSource,
    frame_id: &str,
    pose: Pose2,
    size: UVec2,
) -> anyhow::Result<()> {
    let window = source.window(pose.position, size)?;
    let msg = occupancy_grid_from_costmap(&window, frame_id, now_stamp());
    publisher
        .publish(&msg)
        .map_err(|e| anyhow!("publishing: {e}"))?;
    debug!(
        "published {}x{} window at origin ({:.2}, {:.2}) for robot at ({:.2}, {:.2})",
        size.x,
        size.y,
        window.info().origin.x,
        window.info().origin.y,
        pose.position.x,
        pose.position.y,
    );
    Ok(())
}

/// Number of cells along each side of a `window_m` window, to the nearest cell.
fn window_cells(window_m: f32, resolution: f64) -> anyhow::Result<UVec2> {
    let cells = (f64::from(window_m) / resolution).round();
    if !(cells >= 1.0 && cells <= f64::from(u32::MAX)) {
        return Err(anyhow!(
            "--window-m {window_m} at {resolution} m per cell is not a usable window"
        ));
    }
    Ok(UVec2::splat(cells as u32))
}

fn positive_seconds(seconds: f32, flag: &str) -> anyhow::Result<Duration> {
    Duration::try_from_secs_f32(seconds)
        .ok()
        .filter(|d| !d.is_zero())
        .ok_or_else(|| anyhow!("{flag} must give a positive, finite period"))
}

/// Wall-clock time as a ROS stamp.
fn now_stamp() -> Time {
    let since_epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    Time {
        sec: i32::try_from(since_epoch.as_secs()).unwrap_or(i32::MAX),
        nanosec: since_epoch.subsec_nanos(),
    }
}
