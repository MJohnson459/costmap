// --- Occupancy grid constants (i8, used by OccupancyGrid / Grid2d<i8>) ---

pub const UNKNOWN: i8 = -1;
pub const FREE: i8 = 0;
pub const OCCUPIED: i8 = 100;

pub const DEFAULT_OCCUPIED_THRESH: f32 = 0.65;
pub const DEFAULT_FREE_THRESH: f32 = 0.196;

// --- Costmap constants (u8, used by Grid2d<u8> costmaps) ---

/// Free space in a costmap (no obstacle cost).
pub const COST_FREE: u8 = 0;
/// Inscribed cost — inside the robot's inscribed radius of a lethal obstacle.
pub const COST_INSCRIBED: u8 = 253;
/// Lethal obstacle in a costmap.
pub const COST_LETHAL: u8 = 254;
/// Unknown / no-information cell in a costmap.
pub const COST_UNKNOWN: u8 = 255;
