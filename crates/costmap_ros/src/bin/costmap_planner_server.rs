//! Serves Nav2's `compute_path_to_pose` action from a whole cost GeoTIFF.
//!
//! Nav2's own planner searches its global costmap, so it cannot reach a goal
//! outside the window it holds. This node takes the place of `planner_server`:
//! Nav2's behaviour tree calls the same action and gets a path across the
//! whole map, planned on an overview held in memory.
//!
//! The path starts at the goal's `start` pose when `use_start` is set, and at
//! the latest `map_frame -> base_frame` transform otherwise.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, anyhow};
use clap::Parser;
use costmap_ros::nav2_msgs::action::ComputePathToPose;
use costmap_ros::nav2_msgs::{ComputePathToPoseGoal, ComputePathToPoseResult};
use costmap_ros::tf::TfListener;
use costmap_ros::{Route, RouteError, RouteOptions, RoutePlanner};
use glam::DVec2;
use hiroz::Builder;
use hiroz::action::server::ExecutingGoal;
use hiroz::context::ZContextBuilder;
use hiroz::msg::ZMessage;
use hiroz::pubsub::ZPub;
use hiroz_msgs::builtin_interfaces::{Duration as DurationMsg, Time};
use hiroz_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use hiroz_msgs::nav_msgs::Path;
use hiroz_msgs::std_msgs::Header;
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

/// Error codes of `nav2_msgs/action/ComputePathToPose`.
const ERROR_NONE: u16 = 0;
const ERROR_TF: u16 = 202;
const ERROR_START_OCCUPIED: u16 = 205;
const ERROR_GOAL_OCCUPIED: u16 = 206;
const ERROR_NO_VALID_PATH: u16 = 208;

type PathPub = ZPub<Path, <Path as ZMessage>::Serdes>;

#[derive(Debug, Parser)]
#[command(about = "Plan Nav2's global paths across a whole cost GeoTIFF")]
struct Args {
    /// Single-band u8 cost GeoTIFF; the map frame is at its lower-left corner.
    #[arg(long)]
    map: PathBuf,
    /// Action to serve.
    #[arg(long, default_value = "compute_path_to_pose")]
    action: String,
    /// Topic each planned path is also published on.
    #[arg(long, default_value = "plan")]
    plan_topic: String,
    /// Frame of the map and of the planned paths.
    #[arg(long, default_value = "map")]
    map_frame: String,
    /// Robot frame a path starts at when the goal gives no start.
    #[arg(long, default_value = "base_link")]
    base_frame: String,
    /// Largest cell of the overview the paths are planned on, in metres.
    #[arg(long, default_value_t = RouteOptions::default().overview_m)]
    overview_m: f64,
    /// Robot radius in metres.
    #[arg(long, default_value_t = RouteOptions::default().robot_radius_m)]
    robot_radius_m: f32,
    /// Distance from obstacles over which paths are pushed away, in metres.
    #[arg(long, default_value_t = RouteOptions::default().inflation_m)]
    inflation_m: f32,
}

struct Planner {
    routes: RoutePlanner,
    tf: TfListener,
    plan_pub: PathPub,
    map_frame: String,
    base_frame: String,
}

impl Planner {
    /// Answers one goal: `Ok` holds a path, `Err` an error code and message.
    fn answer(&self, goal: &ComputePathToPoseGoal) -> Result<Route, (u16, String)> {
        let position = |pose: &PoseStamped| DVec2::new(pose.pose.position.x, pose.pose.position.y);
        let start = if goal.use_start {
            position(&goal.start)
        } else {
            self.tf
                .lookup(&self.map_frame, &self.base_frame)
                .map_err(|err| (ERROR_TF, err.to_string()))?
                .planar_pose()
                .position
        };
        self.routes
            .plan(start, position(&goal.goal))
            .map_err(|err| {
                let code = match err {
                    RouteError::Blocked { name: "start", .. } => ERROR_START_OCCUPIED,
                    RouteError::Blocked { .. } => ERROR_GOAL_OCCUPIED,
                    RouteError::NoRoute { .. } => ERROR_NO_VALID_PATH,
                };
                (code, err.to_string())
            })
    }

    /// The route as a path whose poses face along it; the last takes the goal's heading.
    fn path(&self, route: &Route, goal: &PoseStamped) -> Path {
        let header = Header {
            stamp: now_stamp(),
            frame_id: self.map_frame.clone(),
        };
        let last = route.points.len() - 1;
        let poses = route
            .points
            .iter()
            .enumerate()
            .map(|(i, point)| {
                let orientation = if i == last {
                    goal.pose.orientation
                } else {
                    let ahead = route.points[i + 1] - *point;
                    let half_yaw = ahead.y.atan2(ahead.x) / 2.0;
                    Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: half_yaw.sin(),
                        w: half_yaw.cos(),
                    }
                };
                PoseStamped {
                    header: header.clone(),
                    pose: Pose {
                        position: Point {
                            x: point.x,
                            y: point.y,
                            z: 0.0,
                        },
                        orientation,
                    },
                }
            })
            .collect();
        Path { header, poses }
    }

    fn execute(&self, executing: ExecutingGoal<ComputePathToPose>) {
        let started = Instant::now();
        let goal = executing.goal.clone();
        let answer = self.answer(&goal);
        let elapsed = started.elapsed();
        let planning_time = DurationMsg {
            sec: i32::try_from(elapsed.as_secs()).unwrap_or(i32::MAX),
            nanosec: elapsed.subsec_nanos(),
        };
        let sent = match answer {
            Ok(route) => {
                let path = self.path(&route, &goal.goal);
                info!(
                    "path of {:.1} m, {} poses, in {:.1} ms",
                    route.length_m,
                    path.poses.len(),
                    elapsed.as_secs_f64() * 1e3
                );
                if let Err(err) = self.plan_pub.publish(&path) {
                    warn!("publishing the plan failed: {err}");
                }
                executing.succeed(ComputePathToPoseResult {
                    path,
                    planning_time,
                    error_code: ERROR_NONE,
                    error_msg: String::new(),
                })
            }
            Err((error_code, error_msg)) => {
                warn!("no path: {error_msg}");
                executing.abort(ComputePathToPoseResult {
                    path: Path::default(),
                    planning_time,
                    error_code,
                    error_msg,
                })
            }
        };
        if let Err(err) = sent {
            warn!("sending the result failed: {err}");
        }
    }
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

async fn run(args: Args) -> anyhow::Result<()> {
    let options = RouteOptions {
        overview_m: args.overview_m,
        robot_radius_m: args.robot_radius_m,
        inflation_m: args.inflation_m,
        ..RouteOptions::default()
    };
    let loaded = Instant::now();
    let routes = RoutePlanner::open(&args.map, options)
        .with_context(|| format!("opening GeoTIFF {}", args.map.display()))?;
    info!(
        "overview of {} is {}x{} cells at {:.3} m, read in {:.1} s",
        args.map.display(),
        routes.size().x,
        routes.size().y,
        routes.resolution(),
        loaded.elapsed().as_secs_f64()
    );

    let ctx = ZContextBuilder::default()
        .build()
        .map_err(|e| anyhow!("opening Zenoh session: {e}"))?;
    let node = ctx
        .create_node("costmap_planner_server")
        .build()
        .map_err(|e| anyhow!("creating node: {e}"))?;
    let plan_pub = node
        .create_pub::<Path>(&args.plan_topic)
        .build()
        .map_err(|e| anyhow!("creating publisher on {}: {e}", args.plan_topic))?;
    let tf = TfListener::new(&node).map_err(|e| anyhow!("subscribing to tf: {e}"))?;
    let planner = Arc::new(Planner {
        routes,
        tf,
        plan_pub,
        map_frame: args.map_frame,
        base_frame: args.base_frame,
    });

    // Dropping the server ends it, so it is held until shutdown.
    let _server = node
        .create_action_server::<ComputePathToPose>(&args.action)
        .with_result_timeout(Duration::from_secs(10))
        .build()
        .map_err(|e| anyhow!("creating action server {}: {e}", args.action))?
        .with_handler(move |executing: ExecutingGoal<ComputePathToPose>| {
            let planner = Arc::clone(&planner);
            async move { planner.execute(executing) }
        });
    info!("serving {}", args.action);

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
