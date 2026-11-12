//! Writing numbered PNG frames for a video.

use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};

use glam::DVec2;

use crate::{
    ROBOT_RADIUS_M,
    render::{self, Scene},
    sim::{STEP_S, Simulation, Status},
};

/// Frames aimed for in a full run: 45 s at 30 frames per second.
const TARGET_FRAMES: f32 = 1350.0;
/// Frame rate the printed `ffmpeg` command assumes.
pub const VIDEO_FPS: u32 = 30;

pub struct FrameWriter {
    dir: PathBuf,
    /// A frame is written every this many simulation steps.
    interval_steps: u64,
    written: u32,
}

impl FrameWriter {
    /// Write into `dir`, creating it, at an interval that gives about
    /// [`TARGET_FRAMES`] frames for a route of `route_length_m` driven at `speed_mps`.
    pub fn new(dir: &Path, route_length_m: f32, speed_mps: f32) -> Result<Self, Box<dyn Error>> {
        fs::create_dir_all(dir).map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
        let expected_steps = route_length_m / speed_mps / STEP_S;
        Ok(Self {
            dir: dir.to_path_buf(),
            interval_steps: (expected_steps / TARGET_FRAMES).round().max(1.0) as u64,
            written: 0,
        })
    }

    /// Write a frame if one is due at this step, and always for the last step.
    pub fn after_step(&mut self, sim: &Simulation) -> Result<(), Box<dyn Error>> {
        if !sim.steps().is_multiple_of(self.interval_steps) && sim.status() == Status::Driving {
            return Ok(());
        }
        let path = self.dir.join(format!("frame_{:05}.png", self.written));
        render::render(&scene(sim))
            .save(&path)
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        self.written += 1;
        Ok(())
    }

    pub fn written(&self) -> u32 {
        self.written
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }
}

fn scene(sim: &Simulation) -> Scene<'_> {
    let tiles = sim.tiles();
    let edge = f64::from(tiles.chunk_size()) * tiles.resolution();
    let chunks = tiles
        .loaded_chunks()
        .map(|c| {
            let min = c.as_dvec2() * edge;
            (min, min + DVec2::splat(edge))
        })
        .collect();
    Scene {
        overview: &sim.overview().costs,
        route: sim.route().points(),
        driven: sim.driven(),
        robot: sim.pose(),
        robot_radius_m: ROBOT_RADIUS_M,
        chunks,
        local: sim.local_costmap(),
        hits: sim.hits(),
        local_plan: sim.local_plan(),
        surprises: sim.surprises().iter().map(|s| s.corners()).collect(),
    }
}
