//! Rerun logging.
//!
//! Everything is logged under `world/` in the map's local frame, on the `step` and
//! `sim_time` timelines. The robot is a transform at `world/robot`; the "Robot" view
//! uses it as its origin, so that view follows the robot while "Farm" shows the
//! whole map.

use std::error::Error;

use costmap::{
    Costmap, OccupancyGrid,
    rerun_viz::{log_costmap, log_occupancy_grid_static},
    types::{COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN, OCCUPIED, UNKNOWN},
};
use glam::Vec2;
use rerun::{
    Color, RecordingStream, RecordingStreamBuilder,
    blueprint::{Blueprint, Horizontal, Spatial3DView},
};

use crate::{
    ROBOT_RADIUS_M,
    cli::Viewer,
    sim::{Simulation, Status},
};

const APPLICATION_ID: &str = "costmap_farm_demo";
/// The local costmap, chunk outlines and driven path are logged every this many steps
/// and at the last step; the robot, lidar and local plan every step.
const SLOW_LOG_PERIOD_STEPS: u64 = 10;
/// The driven path is logged in pieces of this many steps, each its own entity under
/// `world/driven`, so each log carries at most one piece rather than the whole path.
/// A multiple of [`SLOW_LOG_PERIOD_STEPS`].
const DRIVEN_PIECE_STEPS: u64 = 500;
/// Line segments approximating the robot's outline.
const ROBOT_OUTLINE_SEGMENTS: usize = 24;

/// Line widths in UI points.
const LINE_UI_POINTS: f32 = 2.0;
const THIN_LINE_UI_POINTS: f32 = 1.0;
const HIT_UI_POINTS: f32 = 2.0;
/// Length of the heading arrow in robot radii.
const HEADING_ARROW_RADII: f32 = 2.0;

/// Heights that keep the layers stacked in the 3D view.
const Z_OVERVIEW: f32 = 0.0;
const Z_LOCAL_COSTMAP: f32 = 0.05;
const Z_CHUNKS: f32 = 0.1;
const Z_ROUTE: f32 = 0.15;
const Z_DRIVEN: f32 = 0.2;
const Z_LOCAL_PLAN: f32 = 0.25;
const Z_HITS: f32 = 0.3;
const Z_SURPRISES: f32 = 0.32;
const Z_ROBOT: f32 = 0.35;

const ROUTE: [u8; 3] = [40, 100, 210];
const DRIVEN: [u8; 3] = [225, 90, 30];
const LOCAL_PLAN: [u8; 3] = [0, 170, 200];
const ROBOT: [u8; 3] = [210, 30, 60];
const CHUNK: [u8; 3] = [0, 150, 120];
const WINDOW: [u8; 3] = [150, 60, 180];
const HIT: [u8; 3] = [20, 200, 60];
const SURPRISE: [u8; 3] = [240, 170, 0];

pub struct RerunLog {
    rec: RecordingStream,
    /// Route replan count when the route was last logged.
    logged_route: Option<usize>,
}

impl RerunLog {
    /// Open the viewer or the output file; `None` when headless.
    pub fn open(viewer: &Viewer) -> Result<Option<Self>, Box<dyn Error>> {
        let builder =
            RecordingStreamBuilder::new(APPLICATION_ID).with_default_blueprint(blueprint());
        let rec = match viewer {
            Viewer::Headless => return Ok(None),
            Viewer::Spawn => builder.spawn()?,
            Viewer::Save(path) => builder
                .save(path)
                .map_err(|e| format!("cannot write {}: {e}", path.display()))?,
        };
        Ok(Some(Self {
            rec,
            logged_route: None,
        }))
    }

    /// Log what does not change during the run.
    pub fn log_static(&self, sim: &Simulation) -> Result<(), Box<dyn Error>> {
        log_occupancy_grid_static(
            &self.rec,
            "world/overview",
            &to_occupancy(&sim.overview().costs),
            Z_OVERVIEW,
        )?;
        let outlines = sim.surprises().iter().map(|s| closed(&s.corners()));
        self.rec.log_static(
            "world/surprises",
            &strips(outlines, Z_SURPRISES, SURPRISE, LINE_UI_POINTS),
        )?;
        // The outline is fixed in the robot's frame, which `world/robot` moves.
        let outline = (0..=ROBOT_OUTLINE_SEGMENTS).map(|i| {
            Vec2::from_angle(std::f32::consts::TAU * i as f32 / ROBOT_OUTLINE_SEGMENTS as f32)
                * ROBOT_RADIUS_M
        });
        self.rec.log_static(
            "world/robot/outline",
            &strips([outline.collect()], Z_ROBOT, ROBOT, LINE_UI_POINTS),
        )?;
        Ok(())
    }

    /// Log the state after a simulation step.
    pub fn log_step(&mut self, sim: &Simulation) -> Result<(), Box<dyn Error>> {
        let rec = &self.rec;
        rec.set_time_sequence("step", sim.steps() as i64);
        rec.set_duration_secs("sim_time", sim.time_s());

        let route_version = sim.stats().global_replans;
        if self.logged_route != Some(route_version) {
            rec.log(
                "world/route",
                &strips(
                    [sim.route().points().to_vec()],
                    Z_ROUTE,
                    ROUTE,
                    LINE_UI_POINTS,
                ),
            )?;
            self.logged_route = Some(route_version);
        }

        let pose = sim.pose();
        let position = pose.position;
        rec.log(
            "world/robot",
            &rerun::Transform3D::from_translation([position.x, position.y, 0.0]),
        )?;
        let heading = Vec2::from_angle(pose.yaw) * HEADING_ARROW_RADII * ROBOT_RADIUS_M;
        rec.log(
            "world/robot/heading",
            &rerun::Arrows3D::from_vectors([[heading.x, heading.y, 0.0]])
                .with_origins([[0.0, 0.0, Z_ROBOT]])
                .with_colors([rgb(ROBOT)]),
        )?;
        rec.log(
            "world/lidar_hits",
            &rerun::Points3D::new(sim.hits().iter().map(|h| [h.x, h.y, Z_HITS]))
                .with_colors([rgb(HIT)])
                .with_radii([rerun::Radius::new_ui_points(HIT_UI_POINTS)]),
        )?;
        let plan = sim.local_plan();
        rec.log(
            "world/local_plan",
            &strips([plan], Z_LOCAL_PLAN, LOCAL_PLAN, LINE_UI_POINTS),
        )?;

        if sim.steps().is_multiple_of(SLOW_LOG_PERIOD_STEPS) || sim.status() != Status::Driving {
            let local = sim.local_costmap();
            log_costmap(rec, "world/local_costmap", local, Z_LOCAL_COSTMAP)?;
            rec.log(
                "world/local_window",
                &strips(
                    [closed(&extent_corners(local))],
                    Z_CHUNKS,
                    WINDOW,
                    LINE_UI_POINTS,
                ),
            )?;
            let tiles = sim.tiles();
            let mut chunks: Vec<_> = tiles.loaded_chunks().collect();
            chunks.sort_by_key(|c| (c.y, c.x));
            let edge = tiles.chunk_size() as f32 * tiles.resolution();
            let outlines = chunks.into_iter().map(|c| {
                let min = c.as_vec2() * edge;
                closed(&[
                    min,
                    min + Vec2::new(edge, 0.0),
                    min + edge,
                    min + Vec2::new(0.0, edge),
                ])
            });
            rec.log(
                "world/chunks",
                &strips(outlines, Z_CHUNKS, CHUNK, THIN_LINE_UI_POINTS),
            )?;
            let (piece, points) = driven_piece(sim.driven(), sim.steps());
            rec.log(
                format!("world/driven/{piece}"),
                &strips([points.to_vec()], Z_DRIVEN, DRIVEN, LINE_UI_POINTS),
            )?;
        }
        Ok(())
    }
}

/// The piece of `driven` that holds the latest point after `steps` steps, as its
/// index and points. Piece `k` runs from point `k * DRIVEN_PIECE_STEPS` to the next
/// piece's first point, so consecutive pieces join, and a piece logged at the step it
/// ends on is complete.
fn driven_piece(driven: &[Vec2], steps: u64) -> (u64, &[Vec2]) {
    let piece = steps.saturating_sub(1) / DRIVEN_PIECE_STEPS;
    let first = (piece * DRIVEN_PIECE_STEPS) as usize;
    (piece, &driven[first.min(driven.len())..])
}

/// Two views side by side: the whole farm, and one centred on the robot.
fn blueprint() -> Blueprint {
    Blueprint::new(Horizontal::new([
        Spatial3DView::new("Farm").with_origin("/world").into(),
        Spatial3DView::new("Robot")
            .with_origin("/world/robot")
            .with_contents(["+ /world/**", "- /world/overview", "- /world/chunks"])
            .into(),
    ]))
}

/// Costs as occupancy for the `RvizMap` colormap: free white, lethal black, unknown teal.
fn to_occupancy(costs: &Costmap) -> OccupancyGrid {
    OccupancyGrid::from_fn(*costs.info(), |x, y| match costs[glam::UVec2::new(x, y)] {
        COST_UNKNOWN => UNKNOWN,
        COST_LETHAL | COST_INSCRIBED => OCCUPIED,
        cost => (i32::from(cost) * (i32::from(OCCUPIED) - 1) / i32::from(COST_INSCRIBED - 1)) as i8,
    })
}

fn extent_corners(grid: &Costmap) -> [Vec2; 4] {
    let info = grid.info();
    let size = Vec2::new(info.world_width(), info.world_height());
    let min = info.origin;
    [
        min,
        min + Vec2::new(size.x, 0.0),
        min + size,
        min + Vec2::new(0.0, size.y),
    ]
}

fn closed(corners: &[Vec2]) -> Vec<Vec2> {
    corners.iter().chain(corners.first()).copied().collect()
}

fn strips(
    lines: impl IntoIterator<Item = Vec<Vec2>>,
    z: f32,
    colour: [u8; 3],
    radius_ui_points: f32,
) -> rerun::LineStrips3D {
    rerun::LineStrips3D::new(
        lines
            .into_iter()
            .map(|line| line.into_iter().map(|p| [p.x, p.y, z]).collect::<Vec<_>>()),
    )
    .with_colors([rgb(colour)])
    .with_radii([rerun::Radius::new_ui_points(radius_ui_points)])
}

fn rgb([r, g, b]: [u8; 3]) -> Color {
    Color::from_rgb(r, g, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn driven_pieces_are_bounded_join_up_and_cover_the_path() {
        let steps = 3 * DRIVEN_PIECE_STEPS + 70;
        let driven: Vec<Vec2> = (0..=steps).map(|i| Vec2::new(i as f32, 0.0)).collect();
        let mut last_logged: Vec<Vec<Vec2>> = Vec::new();
        for step in (SLOW_LOG_PERIOD_STEPS..=steps).step_by(SLOW_LOG_PERIOD_STEPS as usize) {
            let (piece, points) = driven_piece(&driven[..=step as usize], step);
            assert!(points.len() as u64 <= DRIVEN_PIECE_STEPS + 1, "step {step}");
            match last_logged.get_mut(piece as usize) {
                Some(logged) => *logged = points.to_vec(),
                None => last_logged.push(points.to_vec()),
            }
        }
        let joined: Vec<Vec2> = last_logged
            .iter()
            .enumerate()
            .flat_map(|(k, piece)| piece[usize::from(k > 0)..].iter().copied())
            .collect();
        assert_eq!(joined, driven);
    }
}
