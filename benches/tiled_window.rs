use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use glam::{IVec2, UVec2};

use costmap::TiledGrid;

const MAP_CELLS: i32 = 8192;
const CHUNK_SIZE: u32 = 256;

fn populated_grid() -> TiledGrid<u8> {
    let mut grid = TiledGrid::new(0.05, CHUNK_SIZE, 255u8).unwrap();
    let chunks = MAP_CELLS / CHUNK_SIZE as i32;
    for cy in 0..chunks {
        for cx in 0..chunks {
            let data = vec![((cx + cy) % 200) as u8; (CHUNK_SIZE * CHUNK_SIZE) as usize];
            grid.insert_chunk(IVec2::new(cx, cy), data.into()).unwrap();
        }
    }
    grid
}

fn bench_window(c: &mut Criterion) {
    let grid = populated_grid();
    let mut group = c.benchmark_group("tiled_window");
    for size in [240u32, 2000] {
        // Offset so the window straddles chunk boundaries on both axes.
        let min = IVec2::new(1000 + 17, 3000 + 91);
        group.bench_function(format!("{size}x{size}"), |b| {
            b.iter(|| black_box(grid.window(black_box(min), UVec2::splat(size))))
        });
    }
    group.finish();
}

criterion_group!(benches, bench_window);
criterion_main!(benches);
