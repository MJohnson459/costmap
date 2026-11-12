//! A deterministic synthetic farm, written to a cost GeoTIFF.
//!
//! The farm is a ring of hedge inside an unknown margin, cut into six fields by
//! hedgerows with one gate per hedge segment. Fields have different surface costs,
//! and there are hedgerow trees, tree clumps and a barn with a yard. The generator
//! holds the whole raster while writing it; the navigation side never does.

use std::{
    fs,
    path::{Path, PathBuf},
    sync::atomic::{AtomicU32, Ordering},
};

use costmap::{
    DenseGrid, GeoReference, MapInfo, TiledGrid, VoxelError,
    types::{COST_FREE, COST_LETHAL, COST_UNKNOWN},
    write_cost_geotiff,
};
use glam::{DVec2, UVec2, Vec2};

/// Farm extent in metres, including the unknown margin.
const FARM_SIZE_M: Vec2 = Vec2::new(400.0, 300.0);
const RESOLUTION_M: f32 = 0.1;
/// Unsurveyed border around the farm.
const MARGIN_M: f32 = 6.0;
const SEED: u64 = 0x5EED_FA12_2026;
/// An arbitrary location in UTM zone 30N.
const GEOREFERENCE: GeoReference = GeoReference {
    epsg: Some(32630),
    anchor: DVec2::new(512_000.0, 6_190_000.0),
};

/// Nominal x of the two north-south hedgerows and y of the east-west one.
const HEDGE_X_M: [f32; 2] = [140.0, 270.0];
const HEDGE_Y_M: f32 = 150.0;
/// Random shift applied to each nominal hedgerow position.
const HEDGE_JITTER_M: f32 = 15.0;
const HEDGE_WIDTH_M: f32 = 1.6;
const GATE_WIDTH_M: f32 = 5.0;
/// Gates stay this far from the ends of their hedge segment.
const GATE_END_CLEARANCE_M: f32 = 12.0;
/// Cost of a field's surface: pasture, stubble, plough.
const FIELD_COSTS: [u8; 3] = [0, 30, 80];
/// Spacing range of trees along hedgerows.
const HEDGE_TREE_SPACING_M: (f32, f32) = (20.0, 45.0);
const HEDGE_TREE_RADIUS_M: (f32, f32) = (1.8, 3.2);
/// No hedgerow tree within this distance of a gate centre.
const GATE_TREE_CLEARANCE_M: f32 = 8.0;
const CLUMP_COUNT: usize = 4;
/// Trees per clump: at least the first value, fewer than the second.
const CLUMP_TREES: (u32, u32) = (5, 9);
/// Largest offset of a tree centre from its clump centre along each axis.
const CLUMP_SPREAD_M: f32 = 8.0;
const CLUMP_TREE_RADIUS_M: (f32, f32) = (2.0, 4.0);
/// Clump centres and the barn stay this far from any hedge.
const FEATURE_HEDGE_CLEARANCE_M: f32 = 20.0;
const BARN_SIZE_M: Vec2 = Vec2::new(24.0, 12.0);
const YARD_WIDTH_M: f32 = 6.0;
const YARD_COST: u8 = 40;
/// Chunk edge used to hand the raster to the writer; divides both farm sides in cells,
/// so the written raster is exactly the farm.
const WRITE_CHUNK_CELLS: u32 = 100;

/// A cost GeoTIFF in the system temp directory, deleted on drop.
pub struct TempMap(PathBuf);

impl TempMap {
    pub fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempMap {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

/// Generate the farm and write it to a new file in the temp directory.
pub fn write_temp() -> Result<TempMap, VoxelError> {
    static NEXT: AtomicU32 = AtomicU32::new(0);
    let path = std::env::temp_dir().join(format!(
        "farm_demo_synthetic_{}_{}.tif",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let map = TempMap(path);
    let raster = generate();
    let mut tiles = TiledGrid::new(RESOLUTION_M, WRITE_CHUNK_CELLS, COST_UNKNOWN)?;
    tiles.write_window(&raster)?;
    write_cost_geotiff(map.path(), &tiles, &GEOREFERENCE)?;
    Ok(map)
}

/// SplitMix64: a small, well-mixed generator, so the farm needs no `rand` dependency.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[lo, hi)`.
    fn range(&mut self, (lo, hi): (f32, f32)) -> f32 {
        let unit = (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32;
        lo + (hi - lo) * unit
    }

    fn pick<T: Copy>(&mut self, items: &[T]) -> T {
        items[(self.next_u64() % items.len() as u64) as usize]
    }
}

/// An axis-aligned hedge segment between two points on a line of constant x or y.
struct Hedge {
    from: Vec2,
    to: Vec2,
}

impl Hedge {
    fn length(&self) -> f32 {
        self.from.distance(self.to)
    }

    fn point_at(&self, distance: f32) -> Vec2 {
        self.from + (self.to - self.from).normalize() * distance
    }

    /// The rectangle `distance..distance + length` along the hedge, `width` across.
    fn band(&self, start: f32, length: f32, width: f32) -> (Vec2, Vec2) {
        let a = self.point_at(start);
        let b = self.point_at(start + length);
        let half = Vec2::splat(width / 2.0);
        (a.min(b) - half, a.max(b) + half)
    }
}

/// Rasterise the farm.
fn generate() -> DenseGrid<u8> {
    let size = (FARM_SIZE_M / RESOLUTION_M).round().as_uvec2();
    let mut raster = DenseGrid::filled(
        MapInfo {
            width: size.x,
            height: size.y,
            resolution: RESOLUTION_M,
            origin: Vec2::ZERO,
        },
        COST_UNKNOWN,
    );
    let mut rng = Rng(SEED);
    let inner_min = Vec2::splat(MARGIN_M);
    let inner_max = FARM_SIZE_M - MARGIN_M;
    let xs = [
        inner_min.x,
        HEDGE_X_M[0] + rng.range((-HEDGE_JITTER_M, HEDGE_JITTER_M)),
        HEDGE_X_M[1] + rng.range((-HEDGE_JITTER_M, HEDGE_JITTER_M)),
        inner_max.x,
    ];
    let ys = [
        inner_min.y,
        HEDGE_Y_M + rng.range((-HEDGE_JITTER_M, HEDGE_JITTER_M)),
        inner_max.y,
    ];

    for column in xs.windows(2) {
        for row in ys.windows(2) {
            let cost = rng.pick(&FIELD_COSTS);
            fill_rect(
                &mut raster,
                Vec2::new(column[0], row[0]),
                Vec2::new(column[1], row[1]),
                cost,
            );
        }
    }

    let mut inner = Vec::new();
    for &x in &xs[1..3] {
        for row in ys.windows(2) {
            inner.push(Hedge {
                from: Vec2::new(x, row[0]),
                to: Vec2::new(x, row[1]),
            });
        }
    }
    for column in xs.windows(2) {
        inner.push(Hedge {
            from: Vec2::new(column[0], ys[1]),
            to: Vec2::new(column[1], ys[1]),
        });
    }
    let outer = [
        Hedge {
            from: inner_min,
            to: Vec2::new(inner_max.x, inner_min.y),
        },
        Hedge {
            from: Vec2::new(inner_max.x, inner_min.y),
            to: inner_max,
        },
        Hedge {
            from: inner_max,
            to: Vec2::new(inner_min.x, inner_max.y),
        },
        Hedge {
            from: Vec2::new(inner_min.x, inner_max.y),
            to: inner_min,
        },
    ];

    let hedges: Vec<&Hedge> = inner.iter().chain(&outer).collect();
    let barn_min = place_in_field(&mut rng, xs, ys, BARN_SIZE_M);
    let yard = Vec2::splat(YARD_WIDTH_M);
    fill_rect(
        &mut raster,
        barn_min - yard,
        barn_min + BARN_SIZE_M + yard,
        YARD_COST,
    );

    for hedge in &hedges {
        let (min, max) = hedge.band(0.0, hedge.length(), HEDGE_WIDTH_M);
        fill_rect(&mut raster, min, max, COST_LETHAL);
    }
    let mut gates = Vec::new();
    for hedge in &inner {
        let along = rng.range((
            GATE_END_CLEARANCE_M,
            hedge.length() - GATE_END_CLEARANCE_M - GATE_WIDTH_M,
        ));
        let (min, max) = hedge.band(along, GATE_WIDTH_M, HEDGE_WIDTH_M);
        fill_rect(&mut raster, min, max, COST_FREE);
        gates.push(hedge.point_at(along + GATE_WIDTH_M / 2.0));
    }

    for hedge in &hedges {
        let mut along = rng.range(HEDGE_TREE_SPACING_M);
        while along < hedge.length() {
            let centre = hedge.point_at(along);
            let radius = rng.range(HEDGE_TREE_RADIUS_M);
            if gates
                .iter()
                .all(|g| g.distance(centre) > GATE_TREE_CLEARANCE_M + radius)
            {
                fill_disc(&mut raster, centre, radius, COST_LETHAL);
            }
            along += rng.range(HEDGE_TREE_SPACING_M);
        }
    }

    for _ in 0..CLUMP_COUNT {
        let spread = Vec2::splat(2.0 * CLUMP_SPREAD_M);
        let centre = place_in_field(&mut rng, xs, ys, spread) + spread / 2.0;
        let trees =
            CLUMP_TREES.0 + (rng.next_u64() % u64::from(CLUMP_TREES.1 - CLUMP_TREES.0)) as u32;
        for _ in 0..trees {
            let offset = Vec2::new(
                rng.range((-CLUMP_SPREAD_M, CLUMP_SPREAD_M)),
                rng.range((-CLUMP_SPREAD_M, CLUMP_SPREAD_M)),
            );
            fill_disc(
                &mut raster,
                centre + offset,
                rng.range(CLUMP_TREE_RADIUS_M),
                COST_LETHAL,
            );
        }
    }

    fill_rect(&mut raster, barn_min, barn_min + BARN_SIZE_M, COST_LETHAL);
    raster
}

/// Lower-left corner of a `size` rectangle inside a random field, at least
/// [`FEATURE_HEDGE_CLEARANCE_M`] from the field's hedges. `xs` and `ys` are the
/// field boundaries.
fn place_in_field(rng: &mut Rng, xs: [f32; 4], ys: [f32; 3], size: Vec2) -> Vec2 {
    let column = (rng.next_u64() % 3) as usize;
    let row = (rng.next_u64() % 2) as usize;
    let min = Vec2::new(xs[column], ys[row]) + FEATURE_HEDGE_CLEARANCE_M;
    let max = Vec2::new(xs[column + 1], ys[row + 1]) - FEATURE_HEDGE_CLEARANCE_M - size;
    Vec2::new(rng.range((min.x, max.x)), rng.range((min.y, max.y)))
}

/// Cells whose centres lie in the world rectangle `[min, max)`, clipped to the raster.
fn cells_in(raster: &DenseGrid<u8>, min: Vec2, max: Vec2) -> impl Iterator<Item = UVec2> + use<> {
    let size = UVec2::new(raster.size().x, raster.size().y).as_vec2();
    let resolution = raster.info().resolution;
    let lo = (min / resolution - 0.5)
        .ceil()
        .clamp(Vec2::ZERO, size)
        .as_uvec2();
    let hi = (max / resolution - 0.5)
        .ceil()
        .clamp(Vec2::ZERO, size)
        .as_uvec2();
    (lo.y..hi.y).flat_map(move |y| (lo.x..hi.x).map(move |x| UVec2::new(x, y)))
}

fn fill_rect(raster: &mut DenseGrid<u8>, min: Vec2, max: Vec2, cost: u8) {
    for cell in cells_in(raster, min, max) {
        raster[cell] = cost;
    }
}

fn fill_disc(raster: &mut DenseGrid<u8>, centre: Vec2, radius: f32, cost: u8) {
    let cells: Vec<UVec2> = cells_in(raster, centre - radius, centre + radius)
        .filter(|&cell| raster.map_to_world(cell).distance(centre) <= radius)
        .collect();
    for cell in cells {
        raster[cell] = cost;
    }
}
