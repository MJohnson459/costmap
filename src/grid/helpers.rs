use super::Grid2d;
use crate::types::{Footprint, Pose2};

impl<T> Grid2d<T> {
    /// Get the maximum cost value under a robot footprint at a given pose.
    ///
    /// This is a fundamental operation for path planning, collision checking, and pose validation.
    /// It checks what the highest cost is anywhere under the robot's footprint when placed
    /// at the specified pose.
    ///
    /// # Arguments
    ///
    /// * `position` - Robot position (x, y) in world coordinates (meters)
    /// * `yaw` - Robot orientation in radians
    /// * `footprint` - Footprint polygon vertices relative to robot center (meters)
    ///
    /// # Returns
    ///
    /// The maximum cost value found under the footprint. Returns `T::default()` if the
    /// footprint is completely outside the map bounds.
    ///
    /// # Example
    ///
    /// ```
    /// use costmap::{Grid2d, MapInfo};
    /// use costmap::types::{COST_LETHAL, Footprint, Pose2};
    /// use glam::Vec2;
    ///
    /// let info = MapInfo {
    ///     width: 100,
    ///     height: 100,
    ///     ..Default::default()
    /// };
    /// let costmap = Grid2d::<u8>::new(info);
    ///
    /// // Define a rectangular robot footprint (0.6m x 0.4m)
    /// let footprint = Footprint::rectangle(0.6, 0.4);
    ///
    /// // Check cost at a specific pose
    /// let pose = Pose2::new(Vec2::new(5.0, 5.0), 0.0);
    /// let max_cost = costmap.footprint_cost(pose, &footprint);
    ///
    /// // Use in planning: reject poses with high cost
    /// if max_cost < COST_LETHAL {
    ///     // This pose is safe for the robot
    /// }
    /// ```
    pub fn footprint_cost(&self, pose: Pose2, footprint: &Footprint) -> T
    where
        T: Default + Ord + Copy,
    {
        if !footprint.is_valid() {
            return T::default();
        }

        let transformed_footprint = footprint.transform(pose);

        // Iterate over all cells inside the transformed footprint polygon
        // and find the maximum cost
        let mut max_cost = T::default();

        if let Some(iter) = self.polygon_value(&transformed_footprint) {
            for cost in iter {
                if *cost > max_cost {
                    max_cost = *cost;
                }
            }
        }

        max_cost
    }
}
