//! PNG frames for a video, drawn without a display.
//!
//! Left: the whole map at overview resolution with the global route, the path driven,
//! the robot, the outline of every loaded tile chunk and the local window. Right: the
//! local costmap magnified, with lidar hits, the local plan and the surprise obstacles.
//! North is up in both. Under the right panel a text-free bar shows the cost palette:
//! the gradient from free to the highest non-lethal cost, inscribed, lethal, and unknown.

use costmap::{
    Costmap,
    types::{COST_INSCRIBED, COST_LETHAL, COST_UNKNOWN, Pose2},
};
use glam::{UVec2, Vec2};
use image::{Rgb, RgbImage};

pub const FRAME_SIZE: UVec2 = UVec2::new(1280, 720);
const PADDING_PX: u32 = 16;
const LOCAL_PANEL_PX: u32 = 560;
const LEGEND_GAP_PX: u32 = 14;
const LEGEND_HEIGHT_PX: u32 = 10;
/// Each special-value block of the legend is the legend width divided by this.
const LEGEND_BLOCKS_PER_WIDTH: u32 = 12;

const BACKGROUND: Rgb<u8> = Rgb([34, 36, 40]);
const FREE: Rgb<u8> = Rgb([238, 235, 226]);
/// Colour of cost 252; costs 1..=252 blend from [`FREE`] towards it.
const HIGH_COST: Rgb<u8> = Rgb([176, 132, 70]);
const INSCRIBED: Rgb<u8> = Rgb([120, 78, 40]);
const LETHAL: Rgb<u8> = Rgb([28, 28, 30]);
const UNKNOWN: Rgb<u8> = Rgb([128, 128, 128]);

const ROUTE: Rgb<u8> = Rgb([40, 100, 210]);
const DRIVEN: Rgb<u8> = Rgb([225, 90, 30]);
const LOCAL_PLAN: Rgb<u8> = Rgb([0, 170, 200]);
const ROBOT: Rgb<u8> = Rgb([210, 30, 60]);
const CHUNK: Rgb<u8> = Rgb([0, 150, 120]);
const WINDOW: Rgb<u8> = Rgb([150, 60, 180]);
const HIT: Rgb<u8> = Rgb([20, 200, 60]);
const SURPRISE: Rgb<u8> = Rgb([240, 170, 0]);

const LINE_PX: f32 = 2.0;
const THIN_LINE_PX: f32 = 1.0;
const HIT_PX: f32 = 3.0;
/// Robot marker radius in the map panel, where the true size is under a pixel.
const MAP_ROBOT_RADIUS_PX: f32 = 5.0;
const HEADING_LENGTH_PX: f32 = 12.0;

/// Everything one frame shows, in the map's local frame.
pub struct Scene<'a> {
    pub overview: &'a Costmap,
    pub route: &'a [Vec2],
    pub driven: &'a [Vec2],
    pub robot: Pose2,
    pub robot_radius_m: f32,
    /// Lower-left and upper-right corners of each loaded tile chunk.
    pub chunks: Vec<(Vec2, Vec2)>,
    pub local: &'a Costmap,
    pub hits: &'a [Vec2],
    pub local_plan: Vec<Vec2>,
    /// Corners of each surprise obstacle.
    pub surprises: Vec<[Vec2; 4]>,
}

/// Colour of a cost cell.
pub fn cost_colour(cost: u8) -> Rgb<u8> {
    match cost {
        COST_UNKNOWN => UNKNOWN,
        COST_LETHAL => LETHAL,
        COST_INSCRIBED => INSCRIBED,
        cost => {
            let t = f32::from(cost) / f32::from(COST_INSCRIBED - 1);
            Rgb(std::array::from_fn(|i| {
                (f32::from(FREE.0[i]) + t * (f32::from(HIGH_COST.0[i]) - f32::from(FREE.0[i])))
                    .round() as u8
            }))
        }
    }
}

/// A pixel rectangle, origin at the image's top-left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelRect {
    pub min: UVec2,
    pub size: UVec2,
}

impl PixelRect {
    fn contains(&self, x: i64, y: i64) -> bool {
        let max = (self.min + self.size).as_i64vec2();
        x >= i64::from(self.min.x) && y >= i64::from(self.min.y) && x < max.x && y < max.y
    }
}

/// Mapping from a world rectangle onto a pixel rectangle, north up, same scale on both axes.
#[derive(Debug, Clone, Copy)]
pub struct View {
    /// World position of the top-left corner of `pixels`.
    top_left: Vec2,
    /// Pixels per metre.
    scale: f32,
    pub pixels: PixelRect,
}

impl View {
    /// Fit the world rectangle `[min, max]` into `area`, centred, keeping its aspect ratio.
    pub fn fit(min: Vec2, max: Vec2, area: PixelRect) -> Self {
        let extent = max - min;
        let scale = (area.size.as_vec2() / extent).min_element();
        let size = (extent * scale).round().as_uvec2().min(area.size);
        Self {
            top_left: Vec2::new(min.x, max.y),
            scale,
            pixels: PixelRect {
                min: area.min + (area.size - size) / 2,
                size,
            },
        }
    }

    /// Fit the extent of `grid` into `area`.
    pub fn fit_grid(grid: &Costmap, area: PixelRect) -> Self {
        let info = grid.info();
        let max = info.origin + Vec2::new(info.world_width(), info.world_height());
        Self::fit(info.origin, max, area)
    }

    /// Image position of world point `p`; y grows downwards, so north is up.
    pub fn to_pixel(self, p: Vec2) -> Vec2 {
        let offset = (p - self.top_left) * self.scale;
        self.pixels.min.as_vec2() + Vec2::new(offset.x, -offset.y)
    }

    /// World position of the centre of image pixel `(x, y)`.
    fn to_world(self, x: u32, y: u32) -> Vec2 {
        let offset = (UVec2::new(x, y) - self.pixels.min).as_vec2() + 0.5;
        self.top_left + Vec2::new(offset.x, -offset.y) / self.scale
    }
}

/// Draw one frame.
pub fn render(scene: &Scene) -> RgbImage {
    let mut image = RgbImage::from_pixel(FRAME_SIZE.x, FRAME_SIZE.y, BACKGROUND);
    let local_area = PixelRect {
        min: UVec2::new(
            FRAME_SIZE.x - PADDING_PX - LOCAL_PANEL_PX,
            (FRAME_SIZE.y - LOCAL_PANEL_PX - LEGEND_GAP_PX - LEGEND_HEIGHT_PX) / 2,
        ),
        size: UVec2::splat(LOCAL_PANEL_PX),
    };
    let map_area = PixelRect {
        min: UVec2::splat(PADDING_PX),
        size: UVec2::new(
            local_area.min.x - 2 * PADDING_PX,
            FRAME_SIZE.y - 2 * PADDING_PX,
        ),
    };

    let map = View::fit_grid(scene.overview, map_area);
    paint_grid(&mut image, &map, scene.overview);
    for &(min, max) in &scene.chunks {
        rectangle(&mut image, &map, min, max, CHUNK, THIN_LINE_PX);
    }
    polyline(&mut image, &map, scene.route, ROUTE, LINE_PX);
    polyline(&mut image, &map, scene.driven, DRIVEN, LINE_PX);
    for corners in &scene.surprises {
        polygon(&mut image, &map, corners, SURPRISE, LINE_PX);
    }
    let local_info = scene.local.info();
    let local_max =
        local_info.origin + Vec2::new(local_info.world_width(), local_info.world_height());
    rectangle(
        &mut image,
        &map,
        local_info.origin,
        local_max,
        WINDOW,
        LINE_PX,
    );
    robot(&mut image, &map, scene.robot, MAP_ROBOT_RADIUS_PX);

    let local = View::fit_grid(scene.local, local_area);
    paint_grid(&mut image, &local, scene.local);
    polyline(&mut image, &local, scene.route, ROUTE, LINE_PX);
    polyline(&mut image, &local, scene.driven, DRIVEN, LINE_PX);
    for corners in &scene.surprises {
        polygon(&mut image, &local, corners, SURPRISE, LINE_PX);
    }
    polyline(&mut image, &local, &scene.local_plan, LOCAL_PLAN, LINE_PX);
    for &hit in scene.hits {
        disc(
            &mut image,
            &local.pixels,
            local.to_pixel(hit),
            HIT_PX / 2.0,
            HIT,
        );
    }
    robot(
        &mut image,
        &local,
        scene.robot,
        scene.robot_radius_m * local.scale,
    );

    legend(
        &mut image,
        PixelRect {
            min: UVec2::new(
                local_area.min.x,
                local_area.min.y + LOCAL_PANEL_PX + LEGEND_GAP_PX,
            ),
            size: UVec2::new(LOCAL_PANEL_PX, LEGEND_HEIGHT_PX),
        },
    );
    image
}

/// Fill the view's pixels with the colour of the grid cell under each pixel centre.
pub fn paint_grid(image: &mut RgbImage, view: &View, grid: &Costmap) {
    let PixelRect { min, size } = view.pixels;
    for y in min.y..min.y + size.y {
        for x in min.x..min.x + size.x {
            let colour = grid
                .world_to_map(view.to_world(x, y))
                .map_or(BACKGROUND, |cell| cost_colour(grid[cell]));
            image.put_pixel(x, y, colour);
        }
    }
}

/// Costs 0..=252 as a gradient, then blocks for inscribed and lethal, a gap, and unknown.
fn legend(image: &mut RgbImage, area: PixelRect) {
    let block = area.size.x / LEGEND_BLOCKS_PER_WIDTH;
    let gradient = area.size.x - 4 * block;
    for x in 0..area.size.x {
        let colour = match (x.saturating_sub(gradient) / block, x < gradient) {
            (_, true) => cost_colour((x * u32::from(COST_INSCRIBED - 1) / (gradient - 1)) as u8),
            (0, false) => INSCRIBED,
            (1, false) => LETHAL,
            (2, false) => continue,
            _ => UNKNOWN,
        };
        for y in 0..area.size.y {
            image.put_pixel(area.min.x + x, area.min.y + y, colour);
        }
    }
}

fn robot(image: &mut RgbImage, view: &View, pose: Pose2, radius_px: f32) {
    let centre = view.to_pixel(pose.position);
    disc(image, &view.pixels, centre, radius_px, ROBOT);
    let heading = Vec2::from_angle(pose.yaw) * Vec2::new(1.0, -1.0);
    let tip = centre + heading * radius_px.max(HEADING_LENGTH_PX);
    segment(image, &view.pixels, centre, tip, ROBOT, LINE_PX);
}

fn polyline(image: &mut RgbImage, view: &View, points: &[Vec2], colour: Rgb<u8>, width: f32) {
    for pair in points.windows(2) {
        let (a, b) = (view.to_pixel(pair[0]), view.to_pixel(pair[1]));
        segment(image, &view.pixels, a, b, colour, width);
    }
}

fn polygon(image: &mut RgbImage, view: &View, corners: &[Vec2], colour: Rgb<u8>, width: f32) {
    let closed: Vec<Vec2> = corners.iter().chain(corners.first()).copied().collect();
    polyline(image, view, &closed, colour, width);
}

fn rectangle(image: &mut RgbImage, view: &View, min: Vec2, max: Vec2, colour: Rgb<u8>, width: f32) {
    let corners = [min, Vec2::new(max.x, min.y), max, Vec2::new(min.x, max.y)];
    polygon(image, view, &corners, colour, width);
}

/// A line of `width` pixels from `a` to `b` (image coordinates), clipped to `clip`.
fn segment(image: &mut RgbImage, clip: &PixelRect, a: Vec2, b: Vec2, colour: Rgb<u8>, width: f32) {
    let steps = (b - a).abs().max_element().ceil().max(1.0) as u32;
    for i in 0..=steps {
        let p = a.lerp(b, i as f32 / steps as f32);
        square(image, clip, p, width, colour);
    }
}

fn square(image: &mut RgbImage, clip: &PixelRect, centre: Vec2, width: f32, colour: Rgb<u8>) {
    let first = (centre - width / 2.0).round().as_i64vec2();
    let side = width.round().max(1.0) as i64;
    for y in first.y..first.y + side {
        for x in first.x..first.x + side {
            if clip.contains(x, y) {
                image.put_pixel(x as u32, y as u32, colour);
            }
        }
    }
}

fn disc(image: &mut RgbImage, clip: &PixelRect, centre: Vec2, radius: f32, colour: Rgb<u8>) {
    let radius = radius.max(1.0);
    let first = (centre - radius).floor().as_i64vec2();
    let last = (centre + radius).ceil().as_i64vec2();
    for y in first.y..=last.y {
        for x in first.x..=last.x {
            let pixel_centre = Vec2::new(x as f32 + 0.5, y as f32 + 0.5);
            if pixel_centre.distance(centre) <= radius && clip.contains(x, y) {
                image.put_pixel(x as u32, y as u32, colour);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use costmap::{MapInfo, types::COST_FREE};

    use super::*;

    /// A 4 x 4 grid of 1 m cells with one lethal cell in the north-east corner.
    fn corner_grid() -> Costmap {
        let info = MapInfo::square(4, 1.0);
        Costmap::from_fn(info, |x, y| {
            if (x, y) == (3, 3) {
                COST_LETHAL
            } else {
                COST_FREE
            }
        })
    }

    #[test]
    fn north_east_cell_is_drawn_top_right() {
        let grid = corner_grid();
        let area = PixelRect {
            min: UVec2::new(10, 20),
            size: UVec2::splat(40),
        };
        let view = View::fit_grid(&grid, area);
        let mut image = RgbImage::from_pixel(60, 70, BACKGROUND);
        paint_grid(&mut image, &view, &grid);

        // 10 pixels per cell: the lethal cell covers panel pixels x 30..40, y 0..10.
        assert_eq!(*image.get_pixel(10 + 35, 20 + 5), LETHAL);
        assert_eq!(*image.get_pixel(10 + 35, 20 + 35), FREE);
        assert_eq!(*image.get_pixel(10 + 5, 20 + 5), FREE);
        assert_eq!(*image.get_pixel(5, 5), BACKGROUND);
    }

    #[test]
    fn north_is_up() {
        let view = View::fit(
            Vec2::ZERO,
            Vec2::new(10.0, 5.0),
            PixelRect {
                min: UVec2::ZERO,
                size: UVec2::new(100, 100),
            },
        );
        // A 10 x 5 m area in a square panel is letterboxed vertically.
        assert_eq!(view.pixels.min, UVec2::new(0, 25));
        assert_eq!(view.to_pixel(Vec2::new(0.0, 5.0)), Vec2::new(0.0, 25.0));
        assert_eq!(view.to_pixel(Vec2::new(10.0, 0.0)), Vec2::new(100.0, 75.0));
    }

    #[test]
    fn palette_orders_costs_and_marks_special_values() {
        assert_eq!(cost_colour(COST_FREE), FREE);
        assert_eq!(cost_colour(COST_INSCRIBED - 1), HIGH_COST);
        assert_eq!(cost_colour(COST_LETHAL), LETHAL);
        assert_eq!(cost_colour(COST_UNKNOWN), UNKNOWN);
        let brightness = |c: Rgb<u8>| c.0.iter().map(|&v| u32::from(v)).sum::<u32>();
        assert!(brightness(cost_colour(50)) > brightness(cost_colour(200)));
    }
}
