//! Serves [`RoutePlanner`] routes as Nav2's `ComputePathToPose` action.
//!
//! Both programs use it: `costmap_planner_server` on its own, and
//! `costmap_window_server --routes` beside the map window, where the two share
//! one source of the robot's pose.

use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::anyhow;
use glam::DVec2;
use hiroz::Builder;
use hiroz::action::server::ExecutingGoal;
use hiroz::msg::ZMessage;
use hiroz::node::ZNode;
use hiroz::pubsub::ZPub;
use hiroz_msgs::builtin_interfaces::{Duration as DurationMsg, Time};
use hiroz_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use hiroz_msgs::nav_msgs::Path;
use hiroz_msgs::std_msgs::Header;
use tracing::{info, warn};

use crate::nav2_msgs::action::ComputePathToPose;
use crate::nav2_msgs::{ComputePathToPoseGoal, ComputePathToPoseResult};
use crate::pose::RobotPose;
use crate::{Route, RouteError, RoutePlanner};

/// Error codes of `nav2_msgs/action/ComputePathToPose`.
const ERROR_NONE: u16 = 0;
const ERROR_UNKNOWN: u16 = 200;
const ERROR_TF: u16 = 202;
const ERROR_START_OCCUPIED: u16 = 205;
const ERROR_GOAL_OCCUPIED: u16 = 206;
const ERROR_NO_VALID_PATH: u16 = 208;

type PathPub = ZPub<Path, <Path as ZMessage>::Serdes>;

/// Names a route server is known by.
#[derive(Debug, Clone)]
pub struct RouteServerOptions {
    /// Action served.
    pub action: String,
    /// Topic each route is also published on.
    pub route_topic: String,
    /// Frame of the map and of the routes.
    pub map_frame: String,
}

/// Serve routes from `routes` on `node` until the returned value is dropped.
///
/// A route starts at the goal's `start` pose when `use_start` is set, and at
/// the robot's pose from `pose` otherwise. It passes
/// through the goal's `viapoints` in order. There is one planner, so
/// `planner_id` is not read.
///
/// Must be called inside a Tokio runtime, which runs the goals.
pub fn serve_routes(
    node: &ZNode,
    pose: Arc<RobotPose>,
    routes: RoutePlanner,
    options: RouteServerOptions,
) -> anyhow::Result<impl Sized + use<>> {
    let route_pub = node
        .create_pub::<Path>(&options.route_topic)
        .build()
        .map_err(|e| anyhow!("creating publisher on {}: {e}", options.route_topic))?;
    let action = options.action.clone();
    let server = Arc::new(Server {
        routes,
        pose,
        route_pub,
        map_frame: options.map_frame,
    });
    let handle = node
        .create_action_server::<ComputePathToPose>(&action)
        .with_result_timeout(Duration::from_secs(10))
        .build()
        .map_err(|e| anyhow!("creating action server {action}: {e}"))?
        .with_handler(move |executing: ExecutingGoal<ComputePathToPose>| {
            let server = Arc::clone(&server);
            async move { server.execute(executing) }
        });
    info!("serving {action}");
    Ok(handle)
}

struct Server {
    routes: RoutePlanner,
    pose: Arc<RobotPose>,
    route_pub: PathPub,
    map_frame: String,
}

impl Server {
    /// Answers one goal: `Ok` holds a route, `Err` an error code and message.
    fn answer(&self, goal: &ComputePathToPoseGoal) -> Result<Route, (u16, String)> {
        // Poses are taken as map coordinates, so any other frame is refused.
        let position = |pose: &PoseStamped| {
            let frame = pose.header.frame_id.trim_start_matches('/');
            if frame.is_empty() || frame == self.map_frame {
                Ok(DVec2::new(pose.pose.position.x, pose.pose.position.y))
            } else {
                Err((
                    ERROR_TF,
                    format!("pose is in frame \"{frame}\", not \"{}\"", self.map_frame),
                ))
            }
        };
        let start = if goal.use_start {
            position(&goal.start)?
        } else {
            self.pose.get().map_err(|err| (ERROR_TF, err))?.position
        };
        let via = goal
            .viapoints
            .iter()
            .map(position)
            .collect::<Result<Vec<_>, _>>()?;
        self.routes
            .plan_through(start, &via, position(&goal.goal)?)
            .map_err(|err| {
                let code = match err {
                    RouteError::Blocked { name: "start", .. } => ERROR_START_OCCUPIED,
                    RouteError::Blocked { name: "goal", .. } => ERROR_GOAL_OCCUPIED,
                    RouteError::Blocked { .. } | RouteError::NoRoute { .. } => ERROR_NO_VALID_PATH,
                    RouteError::Map(_) => ERROR_UNKNOWN,
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
            _ if executing.is_cancel_requested() => {
                info!("goal cancelled");
                executing.canceled(ComputePathToPoseResult {
                    path: Path::default(),
                    planning_time,
                    error_code: ERROR_NONE,
                    error_msg: String::new(),
                })
            }
            Ok(route) => {
                let path = self.path(&route, &goal.goal);
                info!(
                    "route of {:.1} m, {} poses, in {:.1} ms",
                    route.length_m,
                    path.poses.len(),
                    elapsed.as_secs_f64() * 1e3
                );
                if let Err(err) = self.route_pub.publish(&path) {
                    warn!("publishing the route failed: {err}");
                }
                executing.succeed(ComputePathToPoseResult {
                    path,
                    planning_time,
                    error_code: ERROR_NONE,
                    error_msg: String::new(),
                })
            }
            Err((error_code, error_msg)) => {
                warn!("no route: {error_msg}");
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

/// Open the map at `path` for routes and log what was opened.
pub fn open_routes(
    path: &std::path::Path,
    options: crate::RouteOptions,
) -> anyhow::Result<RoutePlanner> {
    let loaded = Instant::now();
    let routes = RoutePlanner::open(path, options)
        .map_err(|e| anyhow!("opening map {}: {e}", path.display()))?;
    let graph = routes.graph();
    info!(
        "{} is {}x{} cells at {:.3} m; its tile graph has {} crossings in {:.1} MB, ready in {:.1} s",
        path.display(),
        graph.layout().size().x,
        graph.layout().size().y,
        graph.resolution(),
        graph.node_count() / 2,
        graph.memory_bytes() as f64 / 1e6,
        loaded.elapsed().as_secs_f64()
    );
    Ok(routes)
}
