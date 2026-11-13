//! When the node publishes a new window.

use std::time::{Duration, Instant};

use glam::DVec2;

/// Publish when the robot has moved far enough, or when the last publish is
/// older than an optional keep-alive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RepublishPolicy {
    /// Distance in metres from the last published centre that triggers a publish.
    pub min_move_m: f32,
    /// Age of the last publish that triggers a publish regardless of movement.
    /// `None` publishes on movement alone: the topic is transient local, so a
    /// late subscriber receives the last window without one.
    pub keep_alive: Option<Duration>,
}

/// Where and when the last window was published.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LastPublish {
    pub center: DVec2,
    pub at: Instant,
}

impl RepublishPolicy {
    /// Whether a window centred on `position` should be published at `now`.
    pub fn is_due(&self, last: Option<&LastPublish>, position: DVec2, now: Instant) -> bool {
        let Some(last) = last else {
            return true;
        };
        position.distance(last.center) >= f64::from(self.min_move_m)
            || self
                .keep_alive
                .is_some_and(|age| now.saturating_duration_since(last.at) >= age)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const POLICY: RepublishPolicy = RepublishPolicy {
        min_move_m: 1.0,
        keep_alive: Some(Duration::from_secs(10)),
    };

    fn last_at(center: DVec2, at: Instant) -> LastPublish {
        LastPublish { center, at }
    }

    #[test]
    fn first_window_is_always_due() {
        assert!(POLICY.is_due(None, DVec2::ZERO, Instant::now()));
    }

    #[test]
    fn small_moves_wait_for_the_keep_alive() {
        let t0 = Instant::now();
        let last = last_at(DVec2::ZERO, t0);
        let nearby = DVec2::new(0.6, 0.6);
        assert!(!POLICY.is_due(Some(&last), nearby, t0 + Duration::from_secs(9)));
        assert!(POLICY.is_due(Some(&last), nearby, t0 + Duration::from_secs(10)));
    }

    #[test]
    fn without_a_keep_alive_only_movement_is_due() {
        let policy = RepublishPolicy {
            keep_alive: None,
            ..POLICY
        };
        let t0 = Instant::now();
        let last = last_at(DVec2::ZERO, t0);
        let later = t0 + Duration::from_secs(3600);
        assert!(!policy.is_due(Some(&last), DVec2::new(0.6, 0.6), later));
        assert!(policy.is_due(Some(&last), DVec2::new(1.0, 0.0), later));
    }

    #[test]
    fn moving_the_threshold_distance_is_due() {
        let t0 = Instant::now();
        let last = last_at(DVec2::new(5.0, 5.0), t0);
        assert!(POLICY.is_due(Some(&last), DVec2::new(5.0, 6.0), t0));
        assert!(!POLICY.is_due(Some(&last), DVec2::new(5.0, 5.99), t0));
    }
}
