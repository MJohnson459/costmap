//! Where the robot is in the map frame: from `/tf`, or from a pose topic.
//!
//! `/tf` carries every transform on the robot, often at 50 Hz or more, and each
//! message costs its subscriber time whether or not it is needed. A robot whose
//! localisation also publishes the pose itself, as AMCL does on `amcl_pose`,
//! can give that topic instead, and nothing subscribes to `/tf`.

use std::sync::{Arc, Mutex, PoisonError};

use costmap::types::Pose2;
use glam::DVec2;
use hiroz::msg::ZMessage;
use hiroz::node::ZNode;
use hiroz::pubsub::ZSub;
use hiroz::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use hiroz_msgs::geometry_msgs::PoseWithCovarianceStamped;

use crate::tf::TfListener;

type PoseSub = ZSub<PoseWithCovarianceStamped, (), <PoseWithCovarianceStamped as ZMessage>::Serdes>;

/// The latest pose from a topic, or why it was not usable.
type Latest = Arc<Mutex<Option<Result<Pose2, String>>>>;

/// A source of the robot's pose in the map frame.
pub enum RobotPose {
    /// The latest `map_frame -> base_frame` transform on `/tf`.
    Tf {
        listener: Box<TfListener>,
        map_frame: String,
        base_frame: String,
    },
    /// The latest message on a `geometry_msgs/PoseWithCovarianceStamped` topic.
    Topic {
        topic: String,
        latest: Latest,
        _subscription: Box<PoseSub>,
    },
}

impl RobotPose {
    /// Follow `map_frame -> base_frame` on `/tf` and `/tf_static`.
    pub fn from_tf(node: &ZNode, map_frame: &str, base_frame: &str) -> hiroz::Result<Self> {
        Ok(Self::Tf {
            listener: Box::new(TfListener::new(node)?),
            map_frame: map_frame.to_owned(),
            base_frame: base_frame.to_owned(),
        })
    }

    /// Follow the poses published on `topic`, which must be in `map_frame`. A
    /// message in another frame is not used, and [`get`](Self::get) says so.
    ///
    /// The subscription is reliable and transient local with depth 1, which is
    /// how AMCL publishes `amcl_pose`.
    pub fn from_topic(node: &ZNode, topic: &str, map_frame: &str) -> hiroz::Result<Self> {
        let latest: Latest = Arc::new(Mutex::new(None));
        let store = Arc::clone(&latest);
        let map_frame = map_frame.to_owned();
        let qos = QosProfile {
            reliability: QosReliability::Reliable,
            durability: QosDurability::TransientLocal,
            history: QosHistory::KeepLast(1.try_into().expect("1 is not zero")),
            ..Default::default()
        };
        let subscription = node
            .create_sub::<PoseWithCovarianceStamped>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: PoseWithCovarianceStamped| {
                let pose = pose_in(&msg, &map_frame);
                *store.lock().unwrap_or_else(PoisonError::into_inner) = Some(pose);
            })?;
        Ok(Self::Topic {
            topic: topic.to_owned(),
            latest,
            _subscription: Box::new(subscription),
        })
    }

    /// The robot's pose now, or why it is not known.
    pub fn get(&self) -> Result<Pose2, String> {
        match self {
            Self::Tf {
                listener,
                map_frame,
                base_frame,
            } => listener
                .lookup(map_frame, base_frame)
                .map(|transform| transform.planar_pose())
                .map_err(|err| err.to_string()),
            Self::Topic { topic, latest, .. } => latest
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone()
                .unwrap_or_else(|| Err(format!("no pose received on {topic} yet"))),
        }
    }
}

/// The planar pose a message holds, if it is in `map_frame`.
fn pose_in(msg: &PoseWithCovarianceStamped, map_frame: &str) -> Result<Pose2, String> {
    let frame = msg.header.frame_id.trim_start_matches('/');
    if !frame.is_empty() && frame != map_frame {
        return Err(format!("pose is in frame \"{frame}\", not \"{map_frame}\""));
    }
    let (position, q) = (&msg.pose.pose.position, &msg.pose.pose.orientation);
    let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if !(position.x.is_finite() && position.y.is_finite() && yaw.is_finite()) {
        return Err("pose is not finite".to_owned());
    }
    Ok(Pose2::new(DVec2::new(position.x, position.y), yaw as f32))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message(frame: &str, x: f64, y: f64, yaw: f64) -> PoseWithCovarianceStamped {
        let mut msg = PoseWithCovarianceStamped::default();
        msg.header.frame_id = frame.to_owned();
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.orientation.z = (yaw / 2.0).sin();
        msg.pose.pose.orientation.w = (yaw / 2.0).cos();
        msg
    }

    #[test]
    fn a_pose_in_the_map_frame_gives_position_and_yaw() {
        let pose = pose_in(&message("map", 3.0, -2.0, 1.2), "map").unwrap();
        assert_eq!(pose.position, DVec2::new(3.0, -2.0));
        assert!((pose.yaw - 1.2).abs() < 1e-6);
        assert!(pose_in(&message("/map", 0.0, 0.0, 0.0), "map").is_ok());
    }

    #[test]
    fn a_pose_in_another_frame_or_not_finite_is_refused() {
        assert!(pose_in(&message("odom", 3.0, -2.0, 0.0), "map").is_err());
        assert!(pose_in(&message("map", f64::NAN, 0.0, 0.0), "map").is_err());
    }
}
