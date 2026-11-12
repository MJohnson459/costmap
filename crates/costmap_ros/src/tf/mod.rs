//! Transform lookup over the latest `/tf` and `/tf_static` data.
//!
//! [`TfTree`] keeps only the newest transform for each frame, so a lookup
//! composes the latest transforms with no interpolation to a common time.

mod listener;

pub use listener::TfListener;

use std::collections::HashMap;

use costmap::types::Pose2;
use glam::{DQuat, DVec3, EulerRot};
use hiroz_msgs::geometry_msgs::TransformStamped;
use thiserror::Error;

/// Failure to resolve a transform between two frames.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum TfError {
    #[error("frame \"{0}\" has not been received")]
    UnknownFrame(String),
    #[error("frames \"{target_frame}\" and \"{source_frame}\" are not connected")]
    NotConnected {
        target_frame: String,
        source_frame: String,
    },
    #[error("frame \"{0}\" is part of a loop")]
    Loop(String),
    #[error("transform {parent} -> {child} has a zero-length rotation")]
    InvalidRotation { parent: String, child: String },
}

/// Rotation then translation, as in `geometry_msgs/Transform`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidTransform {
    pub translation: DVec3,
    pub rotation: DQuat,
}

impl RigidTransform {
    pub const IDENTITY: Self = Self {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };

    /// `self * other`: applies `other` first. With `self` = A→B and
    /// `other` = B→C this is A→C.
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            translation: self.translation + self.rotation * other.translation,
            rotation: (self.rotation * other.rotation).normalize(),
        }
    }

    pub fn inverse(&self) -> Self {
        let rotation = self.rotation.inverse();
        Self {
            translation: -(rotation * self.translation),
            rotation,
        }
    }

    /// Position in the plane and rotation about z.
    pub fn planar_pose(&self) -> Pose2 {
        let (yaw, _, _) = self.rotation.to_euler(EulerRot::ZYX);
        Pose2::new(self.translation.truncate(), yaw as f32)
    }
}

/// One frame's transform relative to its parent.
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    pub parent: String,
    pub child: String,
    /// Pose of `child` in `parent`.
    pub transform: RigidTransform,
}

impl Edge {
    /// Reads a ROS transform, stripping any leading `/` from the frame names
    /// as tf2 does.
    pub fn from_msg(msg: &TransformStamped) -> Result<Self, TfError> {
        let parent = frame_name(&msg.header.frame_id).to_owned();
        let child = frame_name(&msg.child_frame_id).to_owned();
        let t = &msg.transform.translation;
        let q = &msg.transform.rotation;
        let rotation = DQuat::from_xyzw(q.x, q.y, q.z, q.w);
        if rotation.length_squared() < 1e-12 {
            return Err(TfError::InvalidRotation { parent, child });
        }
        Ok(Self {
            parent,
            child,
            transform: RigidTransform {
                translation: DVec3::new(t.x, t.y, t.z),
                rotation: rotation.normalize(),
            },
        })
    }
}

fn frame_name(name: &str) -> &str {
    name.trim_start_matches('/')
}

/// The latest transform of every frame, keyed by child frame.
///
/// tf frames form a tree, so each child has exactly one parent and the child
/// name identifies the (parent, child) pair. A newer transform for a child
/// replaces the older one, including a change of parent.
#[derive(Debug, Clone, Default)]
pub struct TfTree {
    edges: HashMap<String, Edge>,
}

impl TfTree {
    pub fn insert(&mut self, edge: Edge) {
        self.edges.insert(edge.child.clone(), edge);
    }

    /// Pose of `source` in `target`: the transform that maps points in
    /// `source` to `target`.
    pub fn lookup(&self, target: &str, source: &str) -> Result<RigidTransform, TfError> {
        let target = frame_name(target);
        let source = frame_name(source);
        let (target_root, root_from_target) = self.to_root(target)?;
        let (source_root, root_from_source) = self.to_root(source)?;
        if target_root != source_root {
            return Err(TfError::NotConnected {
                target_frame: target.to_owned(),
                source_frame: source.to_owned(),
            });
        }
        Ok(root_from_target.inverse().compose(&root_from_source))
    }

    /// Walks parent links from `frame` to the root of its tree and returns
    /// the root name and the pose of `frame` in the root.
    fn to_root<'a>(&'a self, frame: &'a str) -> Result<(&'a str, RigidTransform), TfError> {
        if !self.is_known(frame) {
            return Err(TfError::UnknownFrame(frame.to_owned()));
        }
        let mut current = frame;
        let mut root_from_frame = RigidTransform::IDENTITY;
        // A tree with n edges has paths of at most n links; more means a loop.
        for _ in 0..=self.edges.len() {
            let Some(edge) = self.edges.get(current) else {
                return Ok((current, root_from_frame));
            };
            root_from_frame = edge.transform.compose(&root_from_frame);
            current = &edge.parent;
        }
        Err(TfError::Loop(frame.to_owned()))
    }

    fn is_known(&self, frame: &str) -> bool {
        self.edges.contains_key(frame) || self.edges.values().any(|e| e.parent == frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f64::consts::FRAC_PI_2;

    fn planar(parent: &str, child: &str, x: f64, y: f64, yaw: f64) -> Edge {
        Edge {
            parent: parent.to_owned(),
            child: child.to_owned(),
            transform: RigidTransform {
                translation: DVec3::new(x, y, 0.0),
                rotation: DQuat::from_rotation_z(yaw),
            },
        }
    }

    fn assert_pose(pose: Pose2, x: f64, y: f64, yaw: f32) {
        assert_abs_diff_eq!(pose.position.x, x, epsilon = 1e-9);
        assert_abs_diff_eq!(pose.position.y, y, epsilon = 1e-9);
        assert_abs_diff_eq!(pose.yaw, yaw, epsilon = 1e-5);
    }

    /// map -> odom -> base_link, with odom rotated a quarter turn.
    fn robot_tree() -> TfTree {
        let mut tree = TfTree::default();
        tree.insert(planar("map", "odom", 10.0, 5.0, FRAC_PI_2));
        tree.insert(planar("odom", "base_link", 2.0, 0.0, 0.0));
        tree
    }

    #[test]
    fn composes_a_chain_from_parent_to_child() {
        let pose = robot_tree().lookup("map", "base_link").unwrap();
        // 2 m along odom's x axis, which points along map +y.
        assert_pose(pose.planar_pose(), 10.0, 7.0, FRAC_PI_2 as f32);
    }

    #[test]
    fn reverse_lookup_is_the_inverse() {
        let tree = robot_tree();
        let forward = tree.lookup("map", "base_link").unwrap();
        let back = tree.lookup("base_link", "map").unwrap();
        let identity = forward.compose(&back);
        assert_abs_diff_eq!(identity.translation.length(), 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(
            identity.rotation.angle_between(DQuat::IDENTITY),
            0.0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn resolves_between_siblings() {
        let mut tree = robot_tree();
        tree.insert(planar("odom", "dock", 0.0, 3.0, 0.0));
        let pose = tree.lookup("base_link", "dock").unwrap().planar_pose();
        assert_pose(pose, -2.0, 3.0, 0.0);
    }

    #[test]
    fn lookup_of_a_frame_in_itself_is_identity() {
        let pose = robot_tree().lookup("odom", "odom").unwrap();
        assert_abs_diff_eq!(pose.translation.length(), 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(
            pose.rotation.angle_between(DQuat::IDENTITY),
            0.0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn newest_transform_replaces_the_old_one() {
        let mut tree = robot_tree();
        tree.insert(planar("odom", "base_link", 0.0, -1.0, 0.0));
        let pose = tree.lookup("map", "base_link").unwrap().planar_pose();
        assert_pose(pose, 11.0, 5.0, FRAC_PI_2 as f32);
    }

    #[test]
    fn leading_slashes_are_ignored() {
        let pose = robot_tree()
            .lookup("/map", "/base_link")
            .unwrap()
            .planar_pose();
        assert_pose(pose, 10.0, 7.0, FRAC_PI_2 as f32);
    }

    #[test]
    fn unknown_frame_is_reported() {
        let err = robot_tree().lookup("map", "camera").unwrap_err();
        assert_eq!(err, TfError::UnknownFrame("camera".to_owned()));
    }

    #[test]
    fn disconnected_trees_are_reported() {
        let mut tree = robot_tree();
        tree.insert(planar("world", "gps", 0.0, 0.0, 0.0));
        let err = tree.lookup("map", "gps").unwrap_err();
        assert!(matches!(err, TfError::NotConnected { .. }));
    }

    #[test]
    fn loops_are_reported() {
        let mut tree = TfTree::default();
        tree.insert(planar("a", "b", 1.0, 0.0, 0.0));
        tree.insert(planar("b", "a", 1.0, 0.0, 0.0));
        assert!(matches!(tree.lookup("a", "b"), Err(TfError::Loop(_))));
    }

    #[test]
    fn planar_pose_ignores_roll_and_pitch() {
        let transform = RigidTransform {
            translation: DVec3::new(1.0, 2.0, 3.0),
            rotation: DQuat::from_euler(EulerRot::ZYX, 0.5, 0.1, -0.2),
        };
        assert_pose(transform.planar_pose(), 1.0, 2.0, 0.5);
    }

    #[test]
    fn message_with_zero_rotation_is_rejected() {
        let mut msg = TransformStamped::default();
        msg.header.frame_id = "map".to_owned();
        msg.child_frame_id = "base_link".to_owned();
        assert!(matches!(
            Edge::from_msg(&msg),
            Err(TfError::InvalidRotation { .. })
        ));
    }

    #[test]
    fn message_is_read_and_normalised() {
        let mut msg = TransformStamped::default();
        msg.header.frame_id = "/map".to_owned();
        msg.child_frame_id = "base_link".to_owned();
        msg.transform.translation.x = 4.0;
        msg.transform.rotation.w = 2.0;
        let edge = Edge::from_msg(&msg).unwrap();
        assert_eq!(edge.parent, "map");
        assert_eq!(edge.transform.translation, DVec3::new(4.0, 0.0, 0.0));
        assert_eq!(edge.transform.rotation, DQuat::IDENTITY);
    }
}
