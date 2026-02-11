//! # Footprint Cost Checking
//!
//! Demonstrates footprint-based collision checking with a moving footprint through a funnel.
//! The footprint travels along a corridor that narrows; its color changes as it enters
//! high-cost regions (inflated walls):
//!
//! The footprint color matches the costmap palette (same as the underlying costmap):
//! - **Light blue** = FREE
//! - **Blue→red gradient** = elevated cost
//! - **Purple** = INSCRIBED
//! - **Cyan** = LETHAL
//! - **White** = FREE or UNKNOWN (so the footprint stays visible against blue/teal backgrounds)
//!
//! A small custom funnel map keeps the focus on the footprint checking behaviour.

use std::error::Error;
use std::f32::consts::FRAC_PI_2;
use std::time::Duration;

use costmap::rerun_viz::{cost_to_rerun_color, log_costmap, log_footprint_polygon};
use costmap::types::{COST_FREE, COST_UNKNOWN};
use costmap::{Footprint, Grid2d, MapInfo};
use glam::Vec2;

// Funnel map dimensions
const WIDTH: u32 = 50;
const HEIGHT: u32 = 80;
const RESOLUTION: f32 = 0.05;

// Footprint: length along direction of travel, width across
const ROBOT_LENGTH: f32 = 0.9;
const ROBOT_WIDTH: f32 = 0.54;

// Animation: frames per leg (in or out)
const FRAMES_PER_LEG: i64 = 80;
const DELAY_MS: u64 = 50;

const Z_COSTMAP: f32 = 0.0;
const Z_FOOTPRINT: f32 = 0.08;

fn main() -> Result<(), Box<dyn Error>> {
    let mut costmap = create_funnel_costmap();
    costmap.inflate_inscribed(0.15, 0.1, 3.0);

    let footprint = Footprint::rectangle(ROBOT_LENGTH, ROBOT_WIDTH);
    let info = costmap.info();

    // Path: move from wide end (low y) to narrow end (high y) and back
    let start_y = 0.5;
    let end_y = 3.5;
    let center = info.world_center();

    // Yaw = 90° so the robot faces along the corridor (length along y, width along x)
    let yaw = FRAC_PI_2;

    let rec = rerun::RecordingStreamBuilder::new("costmap_footprint_checking").spawn()?;

    // Log the inflated costmap once (static)
    rec.set_time_sequence("frame", 0);
    log_costmap(&rec, "world/costmap", &costmap, Z_COSTMAP)?;

    let mut frame: i64 = 0;
    loop {
        rec.set_time_sequence("frame", frame);

        // One cycle = in (0..FRAMES_PER_LEG) then out (FRAMES_PER_LEG..2*FRAMES_PER_LEG)
        let cycle_pos = frame % (2 * FRAMES_PER_LEG);
        let t = if cycle_pos < FRAMES_PER_LEG {
            cycle_pos as f32 / (FRAMES_PER_LEG - 1).max(1) as f32
        } else {
            let out_pos = cycle_pos - FRAMES_PER_LEG;
            (FRAMES_PER_LEG - 1 - out_pos) as f32 / (FRAMES_PER_LEG - 1).max(1) as f32
        };

        let position = Vec2::new(center.x, start_y + t * (end_y - start_y));

        let cost = costmap.footprint_cost(position, yaw, &footprint.points);
        let world_footprint = footprint.transform(position, yaw);
        // Use costmap palette, but white when cost matches background (free=blue, unknown=teal)
        let color = if cost == COST_UNKNOWN || cost == COST_FREE {
            rerun::Color::from_rgb(255, 255, 255)
        } else {
            cost_to_rerun_color(cost)
        };

        log_footprint_polygon(
            &rec,
            "world/footprint",
            &world_footprint,
            Z_FOOTPRINT,
            color,
        )?;

        if DELAY_MS > 0 {
            std::thread::sleep(Duration::from_millis(DELAY_MS));
        }

        frame = frame.wrapping_add(1);
    }
}

/// Create a funnel-shaped costmap: wide corridor at bottom, narrow at top.
fn create_funnel_costmap() -> Grid2d<u8> {
    let info = MapInfo {
        width: WIDTH,
        height: HEIGHT,
        resolution: RESOLUTION,
        ..Default::default()
    };

    // Funnel walls: left and right boundaries that converge
    // At y=0 (bottom): corridor x in [12, 38]
    // At y=HEIGHT-1 (top): corridor x in [22, 28] — narrower, footprint gets squeezed
    let left_at_bottom = 12u32;
    let right_at_bottom = 38u32;
    let left_at_top = 22u32;
    let right_at_top = 28u32;

    Grid2d::from_fn(info, |x, y| {
        let t = y as f32 / (HEIGHT - 1).max(1) as f32;
        let left_x = (left_at_bottom as f32 * (1.0 - t) + left_at_top as f32 * t) as u32;
        let right_x = (right_at_bottom as f32 * (1.0 - t) + right_at_top as f32 * t) as u32;
        if x < left_x || x > right_x {
            costmap::types::COST_LETHAL
        } else {
            COST_FREE
        }
    })
}
