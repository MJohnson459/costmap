use glam::{IVec2, UVec2, Vec2};

use crate::Grid2d;

pub struct LineIterator {
    max_t_grid: f32,
    /// Normalised step direction along each axis.
    step: IVec2,
    t_max: Vec2,
    /// Distance to the next cell boundary along each axis.
    t_delta: Vec2,
    /// Current cell being processed.
    cell: IVec2,
    emit_start: bool,

    width: u32,
    height: u32,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct LineStep {
    pub cell: UVec2,
    pub t: f32,
}

impl LineIterator {
    pub fn new<T>(grid: &Grid2d<T>, origin: &Vec2, dir: &Vec2, max_t: f32) -> Option<Self> {
        if dir.length_squared() == 0.0 {
            return None;
        }

        let info = grid.info();
        let resolution = info.resolution;
        let max_t_grid = max_t / resolution;
        let dir = dir.normalize();

        // TODO: Support starting outside of the grid.
        let start = grid.world_to_map_continuous(origin)?;

        // We use ivecs internally as the steps can be negative.
        let cell = start.floor().as_ivec2();

        let step = IVec2::new(dir.x.signum() as i32, dir.y.signum() as i32);
        let (t_delta_x, t_max_x) = axis_params(start.x, dir.x);
        let (t_delta_y, t_max_y) = axis_params(start.y, dir.y);

        let t_max = Vec2::new(t_max_x, t_max_y);
        let t_delta = Vec2::new(t_delta_x, t_delta_y);
        Some(Self {
            max_t_grid,
            step,
            t_max,
            t_delta,
            cell,
            emit_start: true,
            width: info.width,
            height: info.height,
        })
    }
}

impl Iterator for LineIterator {
    type Item = LineStep;

    fn next(&mut self) -> Option<Self::Item> {
        if self.emit_start {
            self.emit_start = false;
            return Some(LineStep {
                cell: self.cell.as_uvec2(),
                t: 0.0,
            });
        }

        let t;
        if self.t_max.x < self.t_max.y {
            t = self.t_max.x;
            self.t_max.x += self.t_delta.x;
            self.cell.x += self.step.x;
        } else {
            t = self.t_max.y;
            self.t_max.y += self.t_delta.y;
            self.cell.y += self.step.y;
        }

        if t > self.max_t_grid {
            return None;
        }

        // equivalent to (x >= 0 && x < width) for signed x
        if (self.cell.x as u32) >= self.width || (self.cell.y as u32) >= self.height {
            return None;
        }

        Some(LineStep {
            cell: self.cell.as_uvec2(),
            t,
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(self.max_t_grid as usize))
    }
}

pub struct LineValueIterator<'a, T> {
    grid: &'a Grid2d<T>,
    iter: LineIterator,
}

impl<'a, T> LineValueIterator<'a, T> {
    pub fn new(grid: &'a Grid2d<T>, origin: &Vec2, dir: &Vec2, max_t: f32) -> Option<Self> {
        Some(Self {
            iter: LineIterator::new(grid, origin, dir, max_t)?,
            grid,
        })
    }
}

impl<'a, T> Iterator for LineValueIterator<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        self.iter
            .next()
            .map(|step| self.grid.get(&step.cell).unwrap())
    }
}

pub struct LineValueMutIterator<'a, T> {
    grid: &'a mut Grid2d<T>,
    iter: LineIterator,
}

impl<'a, T> LineValueMutIterator<'a, T> {
    pub fn new(grid: &'a mut Grid2d<T>, origin: &Vec2, dir: &Vec2, max_t: f32) -> Option<Self> {
        Some(Self {
            iter: LineIterator::new(grid, origin, dir, max_t)?,
            grid,
        })
    }
}

impl<'a, T> Iterator for LineValueMutIterator<'a, T> {
    type Item = &'a mut T;

    fn next(&mut self) -> Option<Self::Item> {
        // SAFETY: We need to use unsafe here because the compiler doesn't know
        // that the LineIterator will never repeat cells.
        self.iter
            .next()
            .map(|step| unsafe { &mut *(self.grid.get_mut(&step.cell) as *mut T) })
    }
}

fn axis_params(start: f32, dir: f32) -> (f32, f32) {
    if dir == 0.0 {
        return (f32::INFINITY, f32::INFINITY);
    }

    let step = dir.signum();
    let dist_to_boundary = if step > 0.0 {
        1.0 - start.fract()
    } else {
        start.fract()
    };

    let t_delta = (1.0 / dir).abs();
    let t_max = dist_to_boundary * t_delta;
    (t_delta, t_max)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use glam::UVec2;

    use super::*;
    use crate::types::{FREE, OCCUPIED};

    fn test_grid(occupied: Option<UVec2>, resolution: f32, origin: Vec2) -> Grid2d<i8> {
        let width = 5;
        let height = 5;
        let mut data = vec![FREE; width * height];
        if let Some(cell) = occupied {
            let idx = (cell.y as usize) * width + (cell.x as usize);
            data[idx] = OCCUPIED;
        }

        let info = crate::types::MapInfo {
            width: width as u32,
            height: height as u32,
            resolution,
            origin,
        };

        Grid2d::new(info, data).expect("grid should build")
    }

    #[test]
    fn hit_positive() {
        let goal = UVec2::new(4, 1);
        let grid = test_grid(Some(goal), 1.0, Vec2::ZERO);
        let dir = goal.as_vec2().normalize();

        let hit = grid
            .raycast_dda(&Vec2::ZERO, &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 4.1231055, epsilon = 1e-4);
    }

    #[test]
    fn miss() {
        let grid = test_grid(None, 1.0, Vec2::ZERO);
        let dir = Vec2::new(6.0, 1.0).normalize();
        let hit = grid.raycast_dda(&Vec2::ZERO, &dir, 30.0);
        assert!(hit.is_none());
    }

    #[test]
    fn hit_negative_x() {
        let goal = UVec2::new(0, 1);
        let grid = test_grid(Some(goal), 1.0, Vec2::ZERO);
        let dir = Vec2::new(-3.9, 1.0).normalize();
        let hit = grid
            .raycast_dda(&Vec2::new(4.0, 0.0), &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 4.026176, epsilon = 1e-4);
    }

    #[test]
    fn resolution_with_offset() {
        let goal = UVec2::new(2, 4);
        let grid = test_grid(Some(goal), 0.05, Vec2::new(-0.18, 0.0));
        let dir = Vec2::new(-1.0, 2.0).normalize();
        let hit = grid
            .raycast_dda(&Vec2::new(0.04, 0.0), &dir, 30.0)
            .expect("hit expected");
        assert_eq!(hit.cell, goal);
        assert_relative_eq!(hit.hit_distance, 0.2236068, epsilon = 1e-4);
    }

    #[test]
    fn line_value_iterator_reads_values() {
        let mut grid = test_grid(None, 1.0, Vec2::ZERO);
        grid.set(&UVec2::new(2, 0), OCCUPIED).unwrap();

        let dir = Vec2::new(1.0, 0.0);
        let values: Vec<i8> = LineValueIterator::new(&grid, &Vec2::ZERO, &dir, 4.0)
            .unwrap()
            .copied()
            .collect();

        assert!(values.contains(&OCCUPIED));
        assert_eq!(values.len(), 5);
    }

    #[test]
    fn line_value_mut_iterator_writes_values() {
        let mut grid = test_grid(None, 1.0, Vec2::ZERO);
        let dir = Vec2::new(1.0, 0.0);

        for value in LineValueMutIterator::new(&mut grid, &Vec2::ZERO, &dir, 4.0).unwrap() {
            *value = OCCUPIED;
        }

        for x in 0..5 {
            assert_eq!(grid.get(&UVec2::new(x, 0)), Some(&OCCUPIED));
        }
    }
}
