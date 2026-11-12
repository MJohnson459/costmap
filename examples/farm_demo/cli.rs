//! Command-line parsing.
//!
//! Hand-rolled rather than clap: the demo has seven flags, and clap as a
//! dev-dependency would be compiled for every `cargo test` of the crate.

use std::{collections::HashSet, path::PathBuf};

use glam::DVec2;

use crate::route::DEFAULT_OVERVIEW_RESOLUTION_M;

/// Ground speed used when `--speed` is not given.
const DEFAULT_SPEED_MPS: f32 = 1.5;
/// Accepted `--speed` values. At the top a 0.1 s step moves 1 m, a fraction of the
/// 30 m local window; at the bottom a 2 km route takes under six simulated hours.
const SPEED_RANGE_MPS: std::ops::RangeInclusive<f32> = 0.1..=10.0;
/// Accepted `--overview-m` values: from finer than any row crop's lanes need up to
/// field-scale cells.
const OVERVIEW_RANGE_M: std::ops::RangeInclusive<f64> = 0.05..=5.0;

pub const USAGE: &str = "\
Usage: farm_demo [OPTIONS]

Drives a simulated robot across a farm-scale cost map that is paged in from a
GeoTIFF, replanning locally around obstacles the map does not contain.

Options:
  --map PATH      single-band u8 cost GeoTIFF (0 free .. 254 lethal, 255 unknown);
                  without it a synthetic farm is generated and written to a
                  temporary GeoTIFF first
  --start X,Y     start in metres east and north of the map's lower-left corner
  --goal X,Y      goal, same frame; --start and --goal go together
  --save PATH     write a Rerun recording to PATH instead of opening the viewer
  --frames DIR    also write PNG frames to DIR for a video
  --headless      log nothing to Rerun; print the summary only
  --speed MPS     robot speed in metres per second, 0.1 to 10 (default 1.5)
  --overview-m M  largest cell size of the route-planning overview in metres,
                  0.05 to 5 (default 0.5); the route can only pass gaps a few
                  overview cells wide, so use about 0.1 for crop rows
  --help          print this message

Each option may be given once.
";

/// Where the Rerun log goes.
#[derive(Debug, Clone, PartialEq)]
pub enum Viewer {
    Spawn,
    Save(PathBuf),
    Headless,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub map: Option<PathBuf>,
    /// Start and goal in the map's local frame.
    pub endpoints: Option<(DVec2, DVec2)>,
    pub viewer: Viewer,
    pub frames: Option<PathBuf>,
    pub speed_mps: f32,
    /// Largest cell size of the route-planning overview.
    pub overview_m: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            map: None,
            endpoints: None,
            viewer: Viewer::Spawn,
            frames: None,
            speed_mps: DEFAULT_SPEED_MPS,
            overview_m: DEFAULT_OVERVIEW_RESOLUTION_M,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum Command {
    Run(Options),
    Help,
}

/// Parse the arguments after the program name.
pub fn parse(args: impl IntoIterator<Item = String>) -> Result<Command, String> {
    let mut options = Options::default();
    let (mut start, mut goal) = (None, None);
    let (mut save, mut headless) = (None, false);
    let mut args = args.into_iter();
    let mut seen = HashSet::new();
    while let Some(flag) = args.next() {
        if !seen.insert(flag.clone()) {
            return Err(format!("{flag} is given more than once"));
        }
        let mut value = || args.next().ok_or_else(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--help" | "-h" => return Ok(Command::Help),
            "--map" => options.map = Some(PathBuf::from(value()?)),
            "--start" => start = Some(parse_point(&flag, &value()?)?),
            "--goal" => goal = Some(parse_point(&flag, &value()?)?),
            "--save" => save = Some(PathBuf::from(value()?)),
            "--frames" => options.frames = Some(PathBuf::from(value()?)),
            "--headless" => headless = true,
            "--speed" => options.speed_mps = parse_speed(&value()?)?,
            "--overview-m" => options.overview_m = parse_overview(&value()?)?,
            other => return Err(format!("unknown argument {other:?}")),
        }
    }
    options.endpoints = match (start, goal) {
        (Some(start), Some(goal)) => Some((start, goal)),
        (None, None) => None,
        _ => return Err("--start and --goal must be given together".into()),
    };
    options.viewer = match (save, headless) {
        (Some(_), true) => return Err("--save and --headless exclude each other".into()),
        (Some(path), false) => Viewer::Save(path),
        (None, true) => Viewer::Headless,
        (None, false) => Viewer::Spawn,
    };
    Ok(Command::Run(options))
}

fn parse_point(flag: &str, text: &str) -> Result<DVec2, String> {
    let invalid = || format!("{flag} expects X,Y in metres, got {text:?}");
    let (x, y) = text.split_once(',').ok_or_else(invalid)?;
    let x: f64 = x.trim().parse().map_err(|_| invalid())?;
    let y: f64 = y.trim().parse().map_err(|_| invalid())?;
    if x.is_finite() && y.is_finite() {
        Ok(DVec2::new(x, y))
    } else {
        Err(invalid())
    }
}

fn parse_speed(text: &str) -> Result<f32, String> {
    text.parse::<f32>()
        .ok()
        .filter(|s| SPEED_RANGE_MPS.contains(s))
        .ok_or_else(|| {
            format!(
                "--speed expects {} to {} m/s, got {text:?}",
                SPEED_RANGE_MPS.start(),
                SPEED_RANGE_MPS.end()
            )
        })
}

fn parse_overview(text: &str) -> Result<f64, String> {
    text.parse::<f64>()
        .ok()
        .filter(|m| OVERVIEW_RANGE_M.contains(m))
        .ok_or_else(|| {
            format!(
                "--overview-m expects {} to {} m, got {text:?}",
                OVERVIEW_RANGE_M.start(),
                OVERVIEW_RANGE_M.end()
            )
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(args: &[&str]) -> Result<Options, String> {
        match parse(args.iter().map(|s| s.to_string()))? {
            Command::Run(options) => Ok(options),
            Command::Help => Err("help".into()),
        }
    }

    #[test]
    fn no_arguments_spawn_the_viewer_on_the_synthetic_farm() {
        assert_eq!(run(&[]).unwrap(), Options::default());
    }

    #[test]
    fn all_flags_parse() {
        let options = run(&[
            "--map", "cost.tif", "--start", "1.5,2", "--goal", "-3, 4e1", "--save", "out.rrd",
            "--frames", "frames", "--speed", "2.5",
        ])
        .unwrap();
        assert_eq!(options.map, Some(PathBuf::from("cost.tif")));
        assert_eq!(
            options.endpoints,
            Some((DVec2::new(1.5, 2.0), DVec2::new(-3.0, 40.0)))
        );
        assert_eq!(options.viewer, Viewer::Save(PathBuf::from("out.rrd")));
        assert_eq!(options.frames, Some(PathBuf::from("frames")));
        assert_eq!(options.speed_mps, 2.5);
        assert_eq!(run(&["--headless"]).unwrap().viewer, Viewer::Headless);
    }

    #[test]
    fn overview_cell_size_is_parsed_and_bounded() {
        assert_eq!(run(&[]).unwrap().overview_m, 0.5);
        assert_eq!(run(&["--overview-m", "0.12"]).unwrap().overview_m, 0.12);
        assert!(run(&["--overview-m", "0.01"]).is_err());
        assert!(run(&["--overview-m", "6"]).is_err());
        assert!(run(&["--overview-m", "fine"]).is_err());
    }

    #[test]
    fn speed_bounds_are_accepted() {
        assert_eq!(run(&["--speed", "0.1"]).unwrap().speed_mps, 0.1);
        assert_eq!(run(&["--speed", "10"]).unwrap().speed_mps, 10.0);
    }

    #[test]
    fn help_wins() {
        let parsed = parse(["--headless", "--help"].map(String::from)).unwrap();
        assert_eq!(parsed, Command::Help);
    }

    #[test]
    fn bad_input_is_rejected() {
        for args in [
            &["--bogus"][..],
            &["--map"],
            &["--start", "1,2"],
            &["--start", "1;2", "--goal", "3,4"],
            &["--start", "1,nan", "--goal", "3,4"],
            &["--speed", "0"],
            &["--speed", "fast"],
            &["--save", "a.rrd", "--headless"],
            &["--map", "a.tif", "--map", "b.tif"],
            &["--headless", "--headless"],
            &["--start", "1,2", "--goal", "3,4", "--start", "5,6"],
            &["--speed", "0.05"],
            &["--speed", "10.5"],
            &["--speed", "inf"],
        ] {
            assert!(run(args).is_err(), "{args:?}");
        }
    }
}
