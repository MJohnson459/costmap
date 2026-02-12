/// Felzenszwalb & Huttenlocher 2D Euclidean Distance Transform
///
/// Computes exact squared Euclidean distances from every cell to the nearest
/// obstacle cell. The algorithm decomposes the 2D problem into separable 1D
/// passes using the "marching parabolas" (lower envelope) technique.
///
/// Reference: Felzenszwalb & Huttenlocher, "Distance Transforms of Sampled
/// Functions", Theory of Computing (2012).

/// Sentinel value representing "infinite" distance (no obstacle in range).
const INF: f32 = f32::MAX / 2.0;

// ---------------------------------------------------------------------------
// 1D Distance Transform — the core building block
// ---------------------------------------------------------------------------

/// Compute the 1D squared-distance transform of `f` in-place.
///
/// `f[i]` should be 0.0 for obstacle cells and INF for free cells.
/// After this call, `f[i]` contains the minimum of
///   f_orig[j] + (i - j)^2
/// over all j.
///
/// `v` must have length >= n, `z` must have length >= n+1, `d` must have
/// length >= n, where n = f.len().
///
/// This follows the original Felzenszwalb & Huttenlocher reference
/// implementation closely, using unsafe indexing to eliminate bounds checks
/// in the hot loop.
///
/// # Safety
/// `v`, `z`, `d` must be at least as large as documented above.
fn dt_1d(f: &[f32], d: &mut [f32], v: &mut [i32], z: &mut [f32]) {
    let n = f.len();
    if n == 0 {
        return;
    }

    // SAFETY: all indices are bounded by n (for f, d, v) or n+1 (for z).
    // k is bounded by q which is bounded by n-1, and v[k] is always a
    // previously assigned q value, also bounded by n-1. The while loop
    // in the envelope construction can decrement k to -1, but z[0] = -INF
    // guarantees s > z[0] so we never read z at a negative index after the
    // first iteration — we use k >= 0 as the while guard to match the
    // reference impl safely.
    unsafe {
        let f = f.as_ptr();
        let d = d.as_mut_ptr();
        let v = v.as_mut_ptr();
        let z = z.as_mut_ptr();

        #[inline(always)]
        unsafe fn sq(x: f32) -> f32 {
            x * x
        }

        let mut k: i32 = 0;
        *v.add(0) = 0;
        *z.add(0) = -INF;
        *z.add(1) = INF;

        for q in 1..n as i32 {
            let qf = q as f32;
            let fq = *f.add(q as usize);

            let mut s = ((fq + sq(qf))
                - (*f.add(*v.add(k as usize) as usize) + sq(*v.add(k as usize) as f32)))
                / (2.0 * qf - 2.0 * *v.add(k as usize) as f32);

            while s <= *z.add(k as usize) {
                k -= 1;
                s = ((fq + sq(qf))
                    - (*f.add(*v.add(k as usize) as usize) + sq(*v.add(k as usize) as f32)))
                    / (2.0 * qf - 2.0 * *v.add(k as usize) as f32);
            }

            k += 1;
            *v.add(k as usize) = q;
            *z.add(k as usize) = s;
            *z.add(k as usize + 1) = INF;
        }

        k = 0;
        for q in 0..n as i32 {
            while *z.add(k as usize + 1) < q as f32 {
                k += 1;
            }
            let vk = *v.add(k as usize);
            *d.add(q as usize) = sq(q as f32 - vk as f32) + *f.add(vk as usize);
        }
    }
}

// ---------------------------------------------------------------------------
// 2D Distance Transform
// ---------------------------------------------------------------------------

/// A 2D grid storing squared Euclidean distances.
pub struct DistanceField {
    pub width: usize,
    pub height: usize,
    /// Row-major storage: index = y * width + x
    /// Values are squared Euclidean distances to the nearest obstacle.
    pub data: Vec<f32>,
}

impl DistanceField {
    /// Create a new distance field from a binary occupancy grid.
    ///
    /// `occupied[i]` should be `true` for obstacle cells.
    /// Returns a grid where each cell contains the squared Euclidean distance
    /// to the nearest obstacle.
    pub fn from_occupancy(width: usize, height: usize, occupied: &[bool]) -> Self {
        assert_eq!(occupied.len(), width * height);

        let n = width * height;
        let mut data = vec![0.0f32; n];

        // Initialize: 0 for obstacles, INF for free space
        for i in 0..n {
            data[i] = if occupied[i] { 0.0 } else { INF };
        }

        let max_dim = width.max(height);
        let mut v = vec![0i32; max_dim];
        let mut z = vec![0.0f32; max_dim + 1];
        let mut buf = vec![0.0f32; max_dim];

        // --- Pass 1: horizontal (along each row) ---
        for y in 0..height {
            let row_start = y * width;
            let row = &data[row_start..row_start + width];

            dt_1d(row, &mut buf[..width], &mut v, &mut z);

            data[row_start..row_start + width].copy_from_slice(&buf[..width]);
        }

        // --- Pass 2: vertical (along each column) ---
        // Transpose so columns become contiguous rows, transform, transpose back.
        let mut transposed = vec![0.0f32; n];
        transpose_blocked(&data, &mut transposed, width, height);

        for x in 0..width {
            let row_start = x * height;
            let row = &transposed[row_start..row_start + height];

            dt_1d(row, &mut buf[..height], &mut v, &mut z);

            transposed[row_start..row_start + height].copy_from_slice(&buf[..height]);
        }

        transpose_blocked(&transposed, &mut data, height, width);

        DistanceField {
            width,
            height,
            data,
        }
    }

    /// Get the squared Euclidean distance at (x, y).
    #[inline]
    pub fn get(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.width + x]
    }

    /// Get the actual Euclidean distance at (x, y).
    #[inline]
    pub fn get_distance(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.width + x].sqrt()
    }

    /// Convert to a nav2-compatible cost grid (0-254).
    ///
    /// `inscribed_radius`: distance (in cells) at which cost = 254 (lethal).
    /// `inflation_radius`: distance (in cells) at which cost drops to 0.
    /// Uses exponential decay between inscribed and inflation radii,
    /// matching nav2's inflation layer behavior.
    pub fn to_costmap(&self, inscribed_radius: f32, inflation_radius: f32) -> Vec<u8> {
        let inscribed_sq = inscribed_radius * inscribed_radius;
        let inflation_sq = inflation_radius * inflation_radius;
        // Decay factor matching nav2: cost = 253 * exp(-scale * (dist - inscribed))
        // We want cost ≈ 1 at inflation_radius, so:
        //   1 = 253 * exp(-scale * (inflation_radius - inscribed_radius))
        //   scale = ln(253) / (inflation_radius - inscribed_radius)
        let scale = if inflation_radius > inscribed_radius {
            (253.0f32).ln() / (inflation_radius - inscribed_radius)
        } else {
            0.0
        };

        self.data
            .iter()
            .map(|&sq_dist| {
                if sq_dist == 0.0 {
                    254 // Lethal (obstacle cell itself)
                } else if sq_dist <= inscribed_sq {
                    253 // Inscribed — robot center would collide
                } else if sq_dist >= inflation_sq {
                    0 // Free
                } else {
                    let dist = sq_dist.sqrt();
                    let cost = 253.0 * (-scale * (dist - inscribed_radius)).exp();
                    (cost as u8).max(1) // Clamp to [1, 253]
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Signed Distance Field (SDF) — for trajectory optimization
// ---------------------------------------------------------------------------

/// A signed distance field: negative inside obstacles, positive outside.
pub struct SignedDistanceField {
    pub width: usize,
    pub height: usize,
    /// Signed Euclidean distance (not squared). Negative inside obstacles.
    pub data: Vec<f32>,
}

impl SignedDistanceField {
    /// Compute an SDF from a binary occupancy grid.
    ///
    /// Runs the EDT twice: once for exterior distances, once for interior.
    pub fn from_occupancy(width: usize, height: usize, occupied: &[bool]) -> Self {
        // Exterior distances (from free cells to nearest obstacle)
        let exterior = DistanceField::from_occupancy(width, height, occupied);

        // Interior distances (from obstacle cells to nearest free cell)
        let inverted: Vec<bool> = occupied.iter().map(|&o| !o).collect();
        let interior = DistanceField::from_occupancy(width, height, &inverted);

        let data: Vec<f32> = exterior
            .data
            .iter()
            .zip(interior.data.iter())
            .map(|(&ext_sq, &int_sq)| {
                let ext = ext_sq.sqrt();
                let int = int_sq.sqrt();
                if ext_sq == 0.0 {
                    // On or inside an obstacle — distance is negative
                    -int
                } else {
                    ext
                }
            })
            .collect();

        SignedDistanceField {
            width,
            height,
            data,
        }
    }

    /// Get the signed distance at (x, y).
    /// Negative = inside obstacle, positive = outside, zero = on boundary.
    #[inline]
    pub fn get(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.width + x]
    }

    /// Approximate gradient at (x, y) via central differences.
    /// Returns (dx, dy) pointing away from the nearest obstacle surface.
    pub fn gradient(&self, x: usize, y: usize) -> (f32, f32) {
        let w = self.width;
        let h = self.height;

        let dx = if x == 0 {
            self.data[y * w + 1] - self.data[y * w]
        } else if x == w - 1 {
            self.data[y * w + x] - self.data[y * w + x - 1]
        } else {
            (self.data[y * w + x + 1] - self.data[y * w + x - 1]) / 2.0
        };

        let dy = if y == 0 {
            self.data[w + x] - self.data[x]
        } else if y == h - 1 {
            self.data[y * w + x] - self.data[(y - 1) * w + x]
        } else {
            (self.data[(y + 1) * w + x] - self.data[(y - 1) * w + x]) / 2.0
        };

        (dx, dy)
    }
}

// ---------------------------------------------------------------------------
// Parallel version using rayon (feature-gated in a real crate)
// ---------------------------------------------------------------------------

#[cfg(feature = "rayon")]
pub mod parallel {
    use super::*;
    use rayon::prelude::*;

    impl DistanceField {
        /// Parallel 2D EDT. Pass 1 (rows) parallelizes trivially.
        /// Pass 2 (columns) uses a transpose for cache-friendly access.
        pub fn from_occupancy_par(width: usize, height: usize, occupied: &[bool]) -> DistanceField {
            let n = width * height;
            let mut data: Vec<f32> = occupied
                .iter()
                .map(|&o| if o { 0.0 } else { INF })
                .collect();

            // Pass 1: transform each row in parallel
            data.par_chunks_mut(width).for_each(|row| {
                let mut v = vec![0i32; width];
                let mut z = vec![0.0f32; width + 1];
                let mut buf = vec![0.0f32; width];
                dt_1d(row, &mut buf, &mut v, &mut z);
                row.copy_from_slice(&buf);
            });

            // Transpose: (width × height) → (height × width)
            let mut transposed = vec![0.0f32; n];
            transpose_blocked(&data, &mut transposed, width, height);

            // Pass 2: transform each "row" of the transposed grid in parallel
            // (these are the original columns)
            transposed.par_chunks_mut(height).for_each(|col| {
                let mut v = vec![0i32; height];
                let mut z = vec![0.0f32; height + 1];
                let mut buf = vec![0.0f32; height];
                dt_1d(col, &mut buf, &mut v, &mut z);
                col.copy_from_slice(&buf);
            });

            // Transpose back
            transpose_blocked(&transposed, &mut data, height, width);

            DistanceField {
                width,
                height,
                data,
            }
        }
    }
}

fn transpose_blocked(src: &[f32], dst: &mut [f32], w: usize, h: usize) {
    const BLOCK: usize = 64; // tuned to L1 cache line count
    for by in (0..h).step_by(BLOCK) {
        for bx in (0..w).step_by(BLOCK) {
            let y_end = (by + BLOCK).min(h);
            let x_end = (bx + BLOCK).min(w);
            for y in by..y_end {
                for x in bx..x_end {
                    dst[x * h + y] = src[y * w + x];
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_obstacle_center() {
        // 5x5 grid with a single obstacle in the center
        let w = 5;
        let h = 5;
        let mut occupied = vec![false; w * h];
        occupied[2 * w + 2] = true; // (2, 2)

        let df = DistanceField::from_occupancy(w, h, &occupied);

        // Center should be 0
        assert_eq!(df.get(2, 2), 0.0);
        // Adjacent cells should be 1.0 (squared distance)
        assert_eq!(df.get(1, 2), 1.0);
        assert_eq!(df.get(3, 2), 1.0);
        assert_eq!(df.get(2, 1), 1.0);
        assert_eq!(df.get(2, 3), 1.0);
        // Diagonal cells should be 2.0 (1² + 1²)
        assert_eq!(df.get(1, 1), 2.0);
        assert_eq!(df.get(3, 3), 2.0);
        // Corner (0,0) should be 8.0 (2² + 2²)
        assert_eq!(df.get(0, 0), 8.0);
    }

    #[test]
    fn test_two_obstacles() {
        // 7x1 grid: obstacles at positions 0 and 6
        let w = 7;
        let h = 1;
        let mut occupied = vec![false; w];
        occupied[0] = true;
        occupied[6] = true;

        let df = DistanceField::from_occupancy(w, h, &occupied);

        assert_eq!(df.get(0, 0), 0.0);
        assert_eq!(df.get(1, 0), 1.0);
        assert_eq!(df.get(2, 0), 4.0);
        assert_eq!(df.get(3, 0), 9.0); // equidistant, min(9, 9) = 9
        assert_eq!(df.get(4, 0), 4.0);
        assert_eq!(df.get(5, 0), 1.0);
        assert_eq!(df.get(6, 0), 0.0);
    }

    #[test]
    fn test_sdf_signs() {
        // 5x5 grid with a 3x3 solid block in the center
        let w = 5;
        let h = 5;
        let mut occupied = vec![false; w * h];
        for y in 1..4 {
            for x in 1..4 {
                occupied[y * w + x] = true;
            }
        }

        let sdf = SignedDistanceField::from_occupancy(w, h, &occupied);

        // Center of block should be negative (inside obstacle)
        assert!(sdf.get(2, 2) < 0.0, "Center should be negative");
        // Edge of block should be negative (still inside)
        assert!(sdf.get(1, 2) < 0.0, "Edge should be negative");
        // Free cells should be positive
        assert!(sdf.get(0, 0) > 0.0, "Corner should be positive");
        assert!(sdf.get(0, 2) > 0.0, "Adjacent free should be positive");
    }

    #[test]
    fn test_costmap_conversion() {
        let w = 11;
        let h = 1;
        let mut occupied = vec![false; w];
        occupied[5] = true; // obstacle at center

        let df = DistanceField::from_occupancy(w, h, &occupied);
        let costmap = df.to_costmap(1.0, 5.0);

        // Obstacle cell = 254
        assert_eq!(costmap[5], 254);
        // Within inscribed radius = 253
        // (no cells at distance exactly 1 in a 1D case... dist 1 is at index 4 and 6)
        // At distance 1.0, sq_dist = 1.0 = inscribed_sq, so cost = 253
        assert_eq!(costmap[4], 253);
        assert_eq!(costmap[6], 253);
        // Far away cells should be 0
        assert_eq!(costmap[0], 0);
        assert_eq!(costmap[10], 0);
        // Intermediate cells should decay
        assert!(costmap[3] > 0 && costmap[3] < 253);
        assert!(
            costmap[3] > costmap[2],
            "Cost should increase toward obstacle"
        );
    }
}
