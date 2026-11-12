//! Cost overrides drawn as GeoJSON polygons.
//!
//! The file is a FeatureCollection of Polygon and MultiPolygon features in the same
//! projected CRS as the rasters. The CRS is not checked and nothing is reprojected.
//! Each feature has an integer property `cost` in 0..=255.

use std::path::Path;

use costmap::DenseGrid;
use glam::DVec2;
use serde_json::Value;

use crate::{error::GeoError, frame::CellFrame};

/// One closed ring of vertices. The closing edge from the last vertex to the first is implicit.
type Ring = Vec<DVec2>;

/// A polygon: an exterior ring followed by hole rings.
#[derive(Debug, Clone, PartialEq)]
struct Polygon {
    rings: Vec<Ring>,
}

/// The polygons of one feature and the cost they impose.
#[derive(Debug, Clone, PartialEq)]
pub struct Zone {
    pub cost: u8,
    polygons: Vec<Polygon>,
}

/// Read the zones of a GeoJSON file, in file order.
pub fn load_zones(path: &Path) -> Result<Vec<Zone>, GeoError> {
    let text = std::fs::read_to_string(path).map_err(|e| GeoError::from(e).at(path))?;
    parse_zones(&text).map_err(|e| e.at(path))
}

/// Parse a GeoJSON FeatureCollection into zones.
///
/// Errors name the index of the offending feature.
pub fn parse_zones(text: &str) -> Result<Vec<Zone>, GeoError> {
    let root: Value = serde_json::from_str(text)
        .map_err(|e| GeoError::Overrides(format!("invalid JSON: {e}")))?;
    if root.get("type").and_then(Value::as_str) != Some("FeatureCollection") {
        return Err(GeoError::Overrides(
            "overrides must be a GeoJSON FeatureCollection".into(),
        ));
    }
    let features = root
        .get("features")
        .and_then(Value::as_array)
        .ok_or_else(|| GeoError::Overrides("FeatureCollection has no features array".into()))?;
    features
        .iter()
        .enumerate()
        .map(|(index, feature)| {
            parse_feature(feature)
                .map_err(|message| GeoError::Overrides(format!("feature {index}: {message}")))
        })
        .collect()
}

fn parse_feature(feature: &Value) -> Result<Zone, String> {
    Ok(Zone {
        cost: parse_cost(feature)?,
        polygons: parse_geometry(feature.get("geometry"))?,
    })
}

fn parse_cost(feature: &Value) -> Result<u8, String> {
    let cost = feature
        .get("properties")
        .and_then(|p| p.get("cost"))
        .ok_or("missing property `cost`")?;
    let cost = cost
        .as_u64()
        .ok_or_else(|| format!("`cost` must be an integer in 0..=255, found {cost}"))?;
    u8::try_from(cost).map_err(|_| format!("`cost` {cost} is outside 0..=255"))
}

fn parse_geometry(geometry: Option<&Value>) -> Result<Vec<Polygon>, String> {
    let geometry = geometry
        .filter(|g| !g.is_null())
        .ok_or("missing geometry")?;
    let kind = geometry
        .get("type")
        .and_then(Value::as_str)
        .ok_or("geometry has no type")?;
    let coordinates = geometry
        .get("coordinates")
        .ok_or("geometry has no coordinates")?;
    match kind {
        "Polygon" => Ok(vec![parse_polygon(coordinates)?]),
        "MultiPolygon" => coordinates
            .as_array()
            .ok_or("MultiPolygon coordinates must be an array")?
            .iter()
            .map(parse_polygon)
            .collect(),
        other => Err(format!(
            "geometry type {other} is not supported, use Polygon or MultiPolygon"
        )),
    }
}

fn parse_polygon(coordinates: &Value) -> Result<Polygon, String> {
    let rings = coordinates
        .as_array()
        .ok_or("polygon coordinates must be an array of rings")?
        .iter()
        .map(parse_ring)
        .collect::<Result<Vec<_>, _>>()?;
    if rings.is_empty() {
        return Err("polygon has no rings".into());
    }
    Ok(Polygon { rings })
}

fn parse_ring(ring: &Value) -> Result<Ring, String> {
    let positions = ring
        .as_array()
        .ok_or("a ring must be an array of positions")?;
    if positions.len() < 3 {
        return Err("a ring needs at least 3 positions".into());
    }
    positions.iter().map(parse_position).collect()
}

fn parse_position(position: &Value) -> Result<DVec2, String> {
    let numbers = position
        .as_array()
        .ok_or("a position must be an array of numbers")?;
    let coordinate = |i: usize| {
        numbers
            .get(i)
            .and_then(Value::as_f64)
            .filter(|v| v.is_finite())
            .ok_or("a position needs two finite numbers")
    };
    Ok(DVec2::new(coordinate(0)?, coordinate(1)?))
}

/// Set every cell of `grid` whose centre lies inside a zone to the zone's cost.
///
/// Zones apply in order, so later zones overwrite earlier ones. Inside a polygon the
/// even-odd rule decides: a point enclosed by a hole ring is outside.
pub fn apply_zones(grid: &mut DenseGrid<u8>, frame: &CellFrame, zones: &[Zone]) {
    for zone in zones {
        for polygon in &zone.polygons {
            fill_polygon(grid, frame, polygon, zone.cost);
        }
    }
}

/// Scanline fill: for each row of cell centres, find where the row crosses the rings,
/// sort the crossings, and fill between each odd-numbered crossing and the next.
fn fill_polygon(grid: &mut DenseGrid<u8>, frame: &CellFrame, polygon: &Polygon, cost: u8) {
    let size = grid.size();
    let Some((min_y, max_y)) = y_extent(polygon) else {
        return;
    };
    let to_row = |y: f64| (y - frame.origin.y) / frame.resolution - 0.5;
    let first_row = (to_row(min_y).ceil().max(0.0)) as u32;
    let last_row = to_row(max_y).floor().min(f64::from(size.y) - 1.0);
    if last_row < 0.0 {
        return;
    }
    let mut crossings = Vec::new();
    for row in first_row..=last_row as u32 {
        let y = frame.centre(0, i64::from(row)).y;
        row_crossings(polygon, y, &mut crossings);
        for &[enter, leave] in crossings.as_chunks::<2>().0 {
            let to_column = |x: f64| ((x - frame.origin.x) / frame.resolution - 0.5).ceil();
            let start = to_column(enter).max(0.0);
            let end = to_column(leave).min(f64::from(size.x));
            if start < end {
                let row_start = row as usize * size.x as usize;
                grid.as_mut_slice()[row_start + start as usize..row_start + end as usize]
                    .fill(cost);
            }
        }
    }
}

fn y_extent(polygon: &Polygon) -> Option<(f64, f64)> {
    let mut ys = polygon.rings.iter().flatten().map(|v| v.y);
    let first = ys.next()?;
    Some(ys.fold((first, first), |(lo, hi), y| (lo.min(y), hi.max(y))))
}

/// Sorted x positions where the horizontal line at `y` crosses the polygon's edges.
///
/// An edge counts when it spans `y` half-open, `[low, high)`, so a vertex lying exactly
/// on the line is counted once for the pair of edges that meet there, not twice.
fn row_crossings(polygon: &Polygon, y: f64, crossings: &mut Vec<f64>) {
    crossings.clear();
    for ring in &polygon.rings {
        let edges = ring.iter().zip(ring.iter().cycle().skip(1));
        for (a, b) in edges {
            if (a.y <= y) != (b.y <= y) {
                crossings.push(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
            }
        }
    }
    crossings.sort_by(f64::total_cmp);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::cells;
    use glam::UVec2;

    /// A 20 x 20 grid of 1 m cells with its lower-left corner at (100, 200).
    fn setup() -> (DenseGrid<u8>, CellFrame) {
        (
            DenseGrid::new_with_value(cells(UVec2::new(20, 20)), 7),
            CellFrame {
                origin: DVec2::new(100.0, 200.0),
                resolution: 1.0,
            },
        )
    }

    fn feature(cost: &str, geometry: &str) -> String {
        format!(r#"{{"type":"Feature","properties":{{"cost":{cost}}},"geometry":{geometry}}}"#)
    }

    fn collection(features: &[String]) -> String {
        format!(
            r#"{{"type":"FeatureCollection","features":[{}]}}"#,
            features.join(",")
        )
    }

    fn polygon(rings: &[&[(f64, f64)]]) -> String {
        let rings: Vec<String> = rings
            .iter()
            .map(|ring| {
                let points: Vec<String> = ring.iter().map(|(x, y)| format!("[{x},{y}]")).collect();
                format!("[{}]", points.join(","))
            })
            .collect();
        format!(
            r#"{{"type":"Polygon","coordinates":[{}]}}"#,
            rings.join(",")
        )
    }

    fn painted(zones: &[String]) -> DenseGrid<u8> {
        let (mut grid, frame) = setup();
        apply_zones(&mut grid, &frame, &parse_zones(&collection(zones)).unwrap());
        grid
    }

    fn count(grid: &DenseGrid<u8>, value: u8) -> usize {
        grid.as_slice().iter().filter(|&&v| v == value).count()
    }

    #[test]
    fn rectangle_fills_the_cells_whose_centres_are_inside() {
        let square = polygon(&[&[
            (102.0, 203.0),
            (106.0, 203.0),
            (106.0, 206.0),
            (102.0, 206.0),
        ]]);
        let grid = painted(&[feature("254", &square)]);
        assert_eq!(count(&grid, 254), 12);
        assert_eq!(grid[UVec2::new(2, 3)], 254);
        assert_eq!(grid[UVec2::new(5, 5)], 254);
        assert_eq!(grid[UVec2::new(1, 3)], 7);
        assert_eq!(grid[UVec2::new(6, 3)], 7);
        assert_eq!(grid[UVec2::new(2, 6)], 7);
    }

    #[test]
    fn cell_centre_decides_not_cell_overlap() {
        let sliver = polygon(&[&[
            (102.0, 203.0),
            (102.4, 203.0),
            (102.4, 206.0),
            (102.0, 206.0),
        ]]);
        assert_eq!(count(&painted(&[feature("0", &sliver)]), 0), 0);
        let half = polygon(&[&[
            (102.0, 203.0),
            (102.6, 203.0),
            (102.6, 206.0),
            (102.0, 206.0),
        ]]);
        assert_eq!(count(&painted(&[feature("0", &half)]), 0), 3);
    }

    #[test]
    fn concave_polygon_leaves_its_notch_empty() {
        // A "U": two 2-wide arms joined by a 2-high base, opening upwards.
        let u = polygon(&[&[
            (102.0, 202.0),
            (110.0, 202.0),
            (110.0, 210.0),
            (108.0, 210.0),
            (108.0, 204.0),
            (104.0, 204.0),
            (104.0, 210.0),
            (102.0, 210.0),
        ]]);
        let grid = painted(&[feature("254", &u)]);
        assert_eq!(count(&grid, 254), 8 * 2 + 2 * 2 * 6);
        assert_eq!(grid[UVec2::new(5, 8)], 7, "notch between the arms");
        assert_eq!(grid[UVec2::new(5, 3)], 254, "base");
        assert_eq!(grid[UVec2::new(3, 8)], 254, "left arm");
        assert_eq!(grid[UVec2::new(8, 8)], 254, "right arm");
    }

    #[test]
    fn vertex_on_a_cell_centre_row_is_counted_once() {
        let diamond = polygon(&[&[
            (105.0, 202.5),
            (108.0, 205.5),
            (105.0, 208.5),
            (102.0, 205.5),
        ]]);
        let grid = painted(&[feature("254", &diamond)]);
        assert_eq!(
            grid[UVec2::new(4, 2)],
            7,
            "bottom vertex row has zero width"
        );
        assert_eq!(grid[UVec2::new(4, 8)], 7, "top vertex row has zero width");
        assert_eq!(count(&grid, 254), 2 + 4 + 6 + 4 + 2);
    }

    #[test]
    fn hole_stays_unchanged() {
        let outer = [
            (102.0, 202.0),
            (112.0, 202.0),
            (112.0, 212.0),
            (102.0, 212.0),
        ];
        let hole = [
            (105.0, 205.0),
            (109.0, 205.0),
            (109.0, 209.0),
            (105.0, 209.0),
        ];
        let grid = painted(&[feature("254", &polygon(&[&outer, &hole]))]);
        assert_eq!(count(&grid, 254), 100 - 16);
        assert_eq!(grid[UVec2::new(6, 6)], 7);
        assert_eq!(grid[UVec2::new(3, 3)], 254);
    }

    #[test]
    fn polygon_partly_outside_the_grid_is_clipped() {
        let big = polygon(&[&[(90.0, 190.0), (105.0, 190.0), (105.0, 205.0), (90.0, 205.0)]]);
        let grid = painted(&[feature("254", &big)]);
        assert_eq!(count(&grid, 254), 25);
        assert_eq!(grid[UVec2::new(0, 0)], 254);
        assert_eq!(grid[UVec2::new(4, 4)], 254);
        assert_eq!(grid[UVec2::new(5, 4)], 7);
    }

    #[test]
    fn polygon_entirely_outside_the_grid_changes_nothing() {
        let far = polygon(&[&[
            (300.0, 300.0),
            (310.0, 300.0),
            (310.0, 310.0),
            (300.0, 310.0),
        ]]);
        let below = polygon(&[&[
            (100.0, 100.0),
            (110.0, 100.0),
            (110.0, 110.0),
            (100.0, 110.0),
        ]]);
        let grid = painted(&[feature("0", &far), feature("0", &below)]);
        assert_eq!(count(&grid, 7), 400);
    }

    #[test]
    fn later_features_win() {
        let wide = polygon(&[&[
            (102.0, 202.0),
            (110.0, 202.0),
            (110.0, 210.0),
            (102.0, 210.0),
        ]]);
        let narrow = polygon(&[&[
            (104.0, 204.0),
            (108.0, 204.0),
            (108.0, 208.0),
            (104.0, 208.0),
        ]]);
        let grid = painted(&[feature("254", &wide), feature("0", &narrow)]);
        assert_eq!(grid[UVec2::new(5, 5)], 0);
        assert_eq!(grid[UVec2::new(3, 3)], 254);
        let swapped = painted(&[feature("0", &narrow), feature("254", &wide)]);
        assert_eq!(swapped[UVec2::new(5, 5)], 254);
    }

    #[test]
    fn multipolygon_fills_every_part() {
        let a = "[[[102,202],[104,202],[104,204],[102,204]]]";
        let b = "[[[110,210],[112,210],[112,212],[110,212]]]";
        let geometry = format!(r#"{{"type":"MultiPolygon","coordinates":[{a},{b}]}}"#);
        let grid = painted(&[feature("100", &geometry)]);
        assert_eq!(count(&grid, 100), 8);
        assert_eq!(grid[UVec2::new(2, 2)], 100);
        assert_eq!(grid[UVec2::new(11, 11)], 100);
    }

    fn error_of(json: &str) -> String {
        parse_zones(json).unwrap_err().to_string()
    }

    #[test]
    fn malformed_features_are_rejected_with_their_index() {
        let square = polygon(&[&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]]);
        let good = feature("5", &square);
        let missing = format!(r#"{{"type":"Feature","properties":{{}},"geometry":{square}}}"#);
        assert!(error_of(&collection(&[good.clone(), missing])).contains("feature 1"));
        assert!(error_of(&collection(&[feature("256", &square)])).contains("feature 0"));
        assert!(
            error_of(&collection(&[good.clone(), feature("-1", &square)])).contains("feature 1")
        );
        assert!(error_of(&collection(&[feature("1.5", &square)])).contains("integer"));
        assert!(error_of(&collection(&[feature("\"7\"", &square)])).contains("integer"));
        let point = r#"{"type":"Point","coordinates":[1,2]}"#;
        let message = error_of(&collection(&[
            good.clone(),
            good.clone(),
            feature("1", point),
        ]));
        assert!(
            message.contains("feature 2") && message.contains("Point"),
            "{message}"
        );
        let no_geometry = r#"{"type":"Feature","properties":{"cost":1},"geometry":null}"#;
        assert!(error_of(&collection(&[no_geometry.into()])).contains("feature 0"));
        let short = polygon(&[&[(0.0, 0.0), (1.0, 0.0)]]);
        assert!(error_of(&collection(&[feature("1", &short)])).contains("feature 0"));
        let text = r#"{"type":"Polygon","coordinates":[[["a",0],[1,0],[1,1]]]}"#;
        assert!(error_of(&collection(&[feature("1", text)])).contains("feature 0"));
    }

    #[test]
    fn malformed_documents_are_rejected() {
        assert!(error_of("not json").contains("invalid JSON"));
        assert!(error_of(r#"{"type":"Feature"}"#).contains("FeatureCollection"));
        assert!(error_of(r#"{"type":"FeatureCollection"}"#).contains("features"));
    }

    #[test]
    fn empty_collection_is_valid() {
        assert_eq!(parse_zones(&collection(&[])).unwrap(), Vec::new());
    }
}
