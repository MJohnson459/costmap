//! When the window server publishes a new window.

use std::time::{Duration, Instant};

use glam::DVec2;

/// Publish when the robot has moved far enough, or when the last publish is
/// old enough to need a keep-alive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RepublishPolicy {
    /// Distance in metres from the last published centre that triggers a publish.
    pub min_move_m: f32,
    /// Age of the last publish that triggers a publish regardless of movement.
    pub keep_alive: Duration,
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
            || now.saturating_duration_since(last.at) >= self.keep_alive
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const POLICY: RepublishPolicy = RepublishPolicy {
        min_move_m: 1.0,
        keep_alive: Duration::from_secs(10),
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
    fn moving_the_threshold_distance_is_due() {
        let t0 = Instant::now();
        let last = last_at(DVec2::new(5.0, 5.0), t0);
        assert!(POLICY.is_due(Some(&last), DVec2::new(5.0, 6.0), t0));
        assert!(!POLICY.is_due(Some(&last), DVec2::new(5.0, 5.99), t0));
    }
}
