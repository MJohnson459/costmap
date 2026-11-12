//! Fills a [`TfTree`] from `/tf` and `/tf_static`.

use std::num::NonZeroUsize;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use hiroz::msg::ZMessage;
use hiroz::node::ZNode;
use hiroz::pubsub::ZSub;
use hiroz::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use hiroz_msgs::tf2_msgs::TFMessage;

use super::{Edge, RigidTransform, TfError, TfTree};

type TfSub = ZSub<TFMessage, (), <TFMessage as ZMessage>::Serdes>;

/// Queue depth of tf2_ros's own listeners.
const TF_DEPTH: NonZeroUsize = NonZeroUsize::new(100).unwrap();

/// Subscribes to `/tf` and `/tf_static` and answers lookups from the latest
/// transforms received. Dropping it ends both subscriptions.
pub struct TfListener {
    tree: Arc<Mutex<TfTree>>,
    _dynamic: TfSub,
    _static: TfSub,
}

impl TfListener {
    /// Subscribes with tf2_ros's QoS: reliable for both topics, transient
    /// local for `/tf_static` so transforms published before startup arrive.
    pub fn new(node: &ZNode) -> hiroz::Result<Self> {
        let tree = Arc::new(Mutex::new(TfTree::default()));
        let dynamic = subscribe(node, "/tf", QosDurability::Volatile, &tree)?;
        let statics = subscribe(node, "/tf_static", QosDurability::TransientLocal, &tree)?;
        Ok(Self {
            tree,
            _dynamic: dynamic,
            _static: statics,
        })
    }

    /// Pose of `source` in `target` from the latest transforms.
    pub fn lookup(&self, target: &str, source: &str) -> Result<RigidTransform, TfError> {
        lock(&self.tree).lookup(target, source)
    }
}

fn subscribe(
    node: &ZNode,
    topic: &str,
    durability: QosDurability,
    tree: &Arc<Mutex<TfTree>>,
) -> hiroz::Result<TfSub> {
    let qos = QosProfile {
        reliability: QosReliability::Reliable,
        durability,
        history: QosHistory::KeepLast(TF_DEPTH),
        ..Default::default()
    };
    let tree = Arc::clone(tree);
    node.create_sub::<TFMessage>(topic)
        .with_qos(qos)
        .build_with_callback(move |msg: TFMessage| store(&tree, &msg))
}

fn store(tree: &Mutex<TfTree>, msg: &TFMessage) {
    let mut tree = lock(tree);
    for transform in &msg.transforms {
        match Edge::from_msg(transform) {
            Ok(edge) => tree.insert(edge),
            Err(err) => tracing::warn!("ignoring transform: {err}"),
        }
    }
}

/// The tree is only changed by whole-edge inserts, so a panic elsewhere
/// cannot leave it inconsistent and a poisoned lock is safe to use.
fn lock(tree: &Mutex<TfTree>) -> MutexGuard<'_, TfTree> {
    tree.lock().unwrap_or_else(PoisonError::into_inner)
}
