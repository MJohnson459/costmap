use glam::{IVec2, UVec2, Vec2};

use crate::Grid2d;

/// Iterator over all grid cells inside a convex polygon.
///
/// Points are expected in world coordinates (meters).
pub struct PolygonIterator {
    points: Vec<Vec2>,
    y: i32,
    y_max: i32,
    x_end: i32,
    has_span: bool,
    grid_size: IVec2,
    cell: IVec2,
    poly_min_y: f32,
    poly_max_y: f32,
}

impl PolygonIterator {
    pub fn new<T>(grid: &Grid2d<T>, points: &[Vec2]) -> Option<Self> {
        if points.len() < 3 {
            return None;
        }
        let info = grid.info();
        let resolution = info.resolution;
        let origin = info.origin;
        let map_points = points.iter().map(|p| (*p - origin) / resolution).collect();
        Some(Self::new_map(map_points, info.width, info.height))
    }

    fn new_map(points: Vec<Vec2>, width: u32, height: u32) -> Self {
        let (min_y, max_y) = points
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(min_y, max_y), p| {
                (min_y.min(p.y), max_y.max(p.y))
            });
        let y_min = min_y.floor() as i32;
        let y_max = max_y.ceil() as i32;
        let grid_size = IVec2::new(width as i32, height as i32);

        Self {
            points,
            y: y_min - 1,
            y_max,
            x_end: -1,
            has_span: false,
            grid_size,
            cell: IVec2::ZERO,
            poly_min_y: min_y,
            poly_max_y: max_y,
        }
    }

    fn advance_row(&mut self) -> bool {
        while self.y <= self.y_max {
            let mut y_scan = self.y as f32 + 0.5;
            // Clamp y_scan to be within the polygon's y range to handle boundary cases
            // The +0.5 scans at the center of each row to avoid horizontal edge issues
            y_scan = y_scan.max(self.poly_min_y).min(self.poly_max_y);
            let mut xs = Vec::with_capacity(self.points.len());

            for i in 0..self.points.len() {
                let p0 = self.points[i];
                let p1 = self.points[(i + 1) % self.points.len()];
                if (p0.y - p1.y).abs() < f32::EPSILON {
                    continue;
                }
                let (ymin, ymax) = if p0.y < p1.y { (p0, p1) } else { (p1, p0) };
                if y_scan >= ymin.y && y_scan <= ymax.y {
                    let t = (y_scan - p0.y) / (p1.y - p0.y);
                    let x = p0.x + t * (p1.x - p0.x);
                    xs.push(x);
                }
            }

            if xs.len() >= 2 {
                xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let mut x_start = xs[0].ceil() as i32;
                let mut x_end = xs[xs.len() - 1].floor() as i32;

                if x_start < 0 {
                    x_start = 0;
                }
                if x_end >= self.grid_size.x {
                    x_end = self.grid_size.x - 1;
                }

                if x_start <= x_end && self.y >= 0 && self.y < self.grid_size.y {
                    self.cell = IVec2::new(x_start, self.y);
                    self.x_end = x_end;
                    self.has_span = true;
                    return true;
                }
            }

            self.y += 1;
        }

        false
    }
}

impl Iterator for PolygonIterator {
    type Item = UVec2;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.has_span && self.cell.x <= self.x_end {
                let cell = self.cell.as_uvec2();
                self.cell.x += 1;
                return Some(cell);
            }

            self.has_span = false;
            self.y += 1;
            if !self.advance_row() {
                return None;
            }
        }
    }
}

pub struct PolygonValueIterator<'a, T> {
    grid: &'a Grid2d<T>,
    iter: PolygonIterator,
}

impl<'a, T> PolygonValueIterator<'a, T> {
    pub fn new(grid: &'a Grid2d<T>, points: &[Vec2]) -> Option<Self> {
        Some(Self {
            iter: PolygonIterator::new(grid, points)?,
            grid,
        })
    }
}

impl<'a, T> Iterator for PolygonValueIterator<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|cell| {
            // SAFETY: cell is from PolygonIterator, which only yields in-bounds cells.
            unsafe { self.grid.get_unchecked(cell) }
        })
    }
}

pub struct PolygonValueMutIterator<'a, T> {
    grid: &'a mut Grid2d<T>,
    iter: PolygonIterator,
}

impl<'a, T> PolygonValueMutIterator<'a, T> {
    pub fn new(grid: &'a mut Grid2d<T>, points: &[Vec2]) -> Option<Self> {
        let iter = PolygonIterator::new(grid, points)?;
        Some(Self { iter, grid })
    }
}

impl<'a, T> Iterator for PolygonValueMutIterator<'a, T> {
    type Item = &'a mut T;

    fn next(&mut self) -> Option<Self::Item> {
        // SAFETY: cell is in-bounds (PolygonIterator only yields in-bounds cells).
        // The raw pointer cast extends the lifetime for the returned &mut T.
        self.iter
            .next()
            .map(|cell| unsafe { &mut *(self.grid.get_unchecked_mut(cell) as *mut T) })
    }
}

impl<T> Grid2d<T> {
    pub fn polygon<'a>(&'a self, points: &[Vec2]) -> Option<PolygonIterator> {
        PolygonIterator::new(self, points)
    }

    pub fn polygon_value<'a>(&'a self, points: &[Vec2]) -> Option<PolygonValueIterator<'a, T>> {
        PolygonValueIterator::new(self, points)
    }

    pub fn polygon_value_mut<'a>(
        &'a mut self,
        points: &[Vec2],
    ) -> Option<PolygonValueMutIterator<'a, T>> {
        PolygonValueMutIterator::new(self, points)
    }
}

#[cfg(test)]
mod tests {
    use glam::{UVec2, Vec2};

    use super::{PolygonIterator, PolygonValueIterator, PolygonValueMutIterator};
    use crate::Grid2d;
    use crate::types::MapInfo;

    #[test]
    fn polygon_iter_fills_rectangle() {
        let info = MapInfo {
            width: 8,
            height: 8,
            resolution: 1.0,
            ..Default::default()
        };
        let grid = Grid2d::<u8>::empty(info);
        let points = vec![
            Vec2::new(1.0, 1.0),
            Vec2::new(4.0, 1.0),
            Vec2::new(4.0, 3.0),
            Vec2::new(1.0, 3.0),
        ];
        let cells: Vec<UVec2> = PolygonIterator::new(&grid, &points).unwrap().collect();
        assert!(!cells.is_empty());
        assert!(cells.iter().any(|c| c == &glam::UVec2::new(1, 1)));
        assert!(cells.iter().any(|c| c == &glam::UVec2::new(3, 2)));
    }

    #[test]
    fn polygon_value_iterator_reads_values() {
        let info = MapInfo {
            width: 8,
            height: 8,
            resolution: 1.0,
            ..Default::default()
        };
        let mut grid = Grid2d::<u8>::empty(info);
        grid.set(UVec2::new(2, 2), 100).unwrap();

        let points = vec![
            Vec2::new(1.0, 1.0),
            Vec2::new(4.0, 1.0),
            Vec2::new(4.0, 3.0),
            Vec2::new(1.0, 3.0),
        ];
        let values: Vec<u8> = PolygonValueIterator::new(&grid, &points)
            .unwrap()
            .copied()
            .collect();

        assert!(values.contains(&100));
        assert!(!values.is_empty());
    }

    #[test]
    fn polygon_value_mut_iterator_writes_values() {
        let info = MapInfo {
            width: 8,
            height: 8,
            resolution: 1.0,
            ..Default::default()
        };
        let mut grid = Grid2d::<u8>::empty(info);
        let points = vec![
            Vec2::new(1.0, 1.0),
            Vec2::new(4.0, 1.0),
            Vec2::new(4.0, 3.0),
            Vec2::new(1.0, 3.0),
        ];

        for value in PolygonValueMutIterator::new(&mut grid, &points).unwrap() {
            *value = 50;
        }

        // Check that all cells in the polygon were set
        let cells: Vec<UVec2> = PolygonIterator::new(&grid, &points).unwrap().collect();
        for cell in cells {
            assert_eq!(grid.get(cell), Some(&50));
        }
    }
}
