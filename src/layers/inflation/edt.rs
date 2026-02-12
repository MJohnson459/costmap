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

/// Compute the 1D squared-distance transform of `input` into `output`.
///
/// `input[i]` should be 0.0 for obstacle cells and INF for free cells.
/// After this call, `output[i]` contains the minimum of
///   input[j] + (i - j)^2
/// over all j — i.e. the squared distance to the nearest zero in `input`,
/// propagated through any non-zero costs already present.
///
/// Both slices must have the same length. Uses O(n) scratch space via the
/// `v`, `z` buffers (parabola vertices and intersection points).
fn dt_1d(input: &[f32], output: &mut [f32], v: &mut [usize], z: &mut [f32]) {
    let n = input.len();
    if n == 0 {
        return;
    }

    // v[i] = index of the i-th parabola in the lower envelope
    // z[i] = left boundary of the region where the i-th parabola is minimal
    // k    = index of the rightmost parabola in the envelope so far

    let mut k: usize = 0;
    v[0] = 0;
    z[0] = f32::NEG_INFINITY;
    z[1] = f32::INFINITY;

    for q in 1..n {
        // Intersection of parabola at q with parabola at v[k]:
        //   input[q] + (x - q)^2 = input[v[k]] + (x - v[k])^2
        // Solving for x gives:
        //   s = ((input[q] + q^2) - (input[v[k]] + v[k]^2)) / (2q - 2·v[k])
        loop {
            let vk = v[k];
            let s = ((input[q] + (q as f32) * (q as f32))
                - (input[vk] + (vk as f32) * (vk as f32)))
                / (2.0 * (q as f32) - 2.0 * (vk as f32));

            if s > z[k] {
                // New parabola extends the envelope — add it
                k += 1;
                v[k] = q;
                z[k] = s;
                z[k + 1] = f32::INFINITY;
                break;
            } else {
                // Previous parabola is entirely hidden — remove it and retry
                if k == 0 {
                    // Edge case: we're replacing the very first parabola
                    v[0] = q;
                    z[0] = f32::NEG_INFINITY;
                    z[1] = f32::INFINITY;
                    break;
                }
                k -= 1;
            }
        }
    }

    // Read off the lower envelope to fill output
    k = 0;
    for q in 0..n {
        while z[k + 1] < q as f32 {
            k += 1;
        }
        let vk = v[k];
        let dist = (q as f32) - (vk as f32);
        output[q] = input[vk] + dist * dist;
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

        // --- Pass 1: horizontal (along each row) ---
        // Each row is independent and sequential in memory.
        {
            let max_dim = width.max(height);
            let mut v = vec![0usize; max_dim];
            let mut z = vec![0.0f32; max_dim + 1];
            let mut buf = vec![0.0f32; max_dim];

            for y in 0..height {
                let row_start = y * width;
                let row = &data[row_start..row_start + width];

                dt_1d(row, &mut buf[..width], &mut v, &mut z);

                data[row_start..row_start + width].copy_from_slice(&buf[..width]);
            }

            // --- Pass 2: vertical (along each column) ---
            // We pull each column into a contiguous buffer, transform it,
            // then scatter back. For very large grids you'd transpose instead,
            // but this is simpler and the column-gather is the main cost.
            let mut col_in = vec![0.0f32; height];

            for x in 0..width {
                // Gather column
                for y in 0..height {
                    col_in[y] = data[y * width + x];
                }

                dt_1d(&col_in, &mut buf[..height], &mut v, &mut z);

                // Scatter back
                for y in 0..height {
                    data[y * width + x] = buf[y];
                }
            }
        }

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
                let mut v = vec![0usize; width];
                let mut z = vec![0.0f32; width + 1];
                let mut buf = vec![0.0f32; width];
                dt_1d(row, &mut buf, &mut v, &mut z);
                row.copy_from_slice(&buf);
            });

            // Transpose: (width × height) → (height × width)
            let mut transposed = vec![0.0f32; n];
            for y in 0..height {
                for x in 0..width {
                    transposed[x * height + y] = data[y * width + x];
                }
            }

            // Pass 2: transform each "row" of the transposed grid in parallel
            // (these are the original columns)
            transposed.par_chunks_mut(height).for_each(|col| {
                let mut v = vec![0usize; height];
                let mut z = vec![0.0f32; height + 1];
                let mut buf = vec![0.0f32; height];
                dt_1d(col, &mut buf, &mut v, &mut z);
                col.copy_from_slice(&buf);
            });

            // Transpose back
            for y in 0..height {
                for x in 0..width {
                    data[y * width + x] = transposed[x * height + y];
                }
            }

            DistanceField {
                width,
                height,
                data,
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
