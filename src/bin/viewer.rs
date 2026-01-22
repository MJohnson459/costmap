use std::error::Error;
use std::sync::Arc;

use pixels::{Pixels, SurfaceTexture};
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::window::{Window, WindowAttributes};

use voxel_grid::load_occupancy_grid;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args();
    let _binary = args.next();
    let yaml_path = match args.next() {
        Some(path) => path,
        None => {
            eprintln!("usage: viewer <map.yaml>");
            return Ok(());
        }
    };

    let grid = load_occupancy_grid(&yaml_path)?;
    let event_loop = EventLoop::new()?;
    let mut app = ViewerApp::new(grid);
    event_loop.run_app(&mut app)?;

    Ok(())
}

struct ViewerApp {
    grid: voxel_grid::OccupancyGrid,
    base_frame: Vec<u8>,
    buffer_width: u32,
    buffer_height: u32,
    window: Option<Arc<Window>>,
    pixels: Option<Pixels<'static>>,
}

impl ViewerApp {
    fn new(grid: voxel_grid::OccupancyGrid) -> Self {
        let base_frame = build_base_frame(&grid);
        Self {
            grid,
            base_frame,
            buffer_width: 0,
            buffer_height: 0,
            window: None,
            pixels: None,
        }
    }
}

impl ApplicationHandler for ViewerApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window = match event_loop.create_window(
            WindowAttributes::default()
                .with_title("Voxel Grid Viewer")
                .with_inner_size(PhysicalSize::new(self.grid.width(), self.grid.height())),
        ) {
            Ok(window) => Arc::new(window),
            Err(err) => {
                eprintln!("failed to create window: {err}");
                event_loop.exit();
                return;
            }
        };

        let window_size = window.inner_size();
        let buffer_width = window_size.width.min(self.grid.width()).max(1);
        let buffer_height = window_size.height.min(self.grid.height()).max(1);
        let surface_texture =
            SurfaceTexture::new(window_size.width, window_size.height, window.clone());
        let mut pixels = match Pixels::new(buffer_width, buffer_height, surface_texture) {
            Ok(pixels) => pixels,
            Err(err) => {
                eprintln!("failed to create pixels surface: {err}");
                event_loop.exit();
                return;
            }
        };

        self.buffer_width = buffer_width;
        self.buffer_height = buffer_height;
        self.base_frame = build_base_frame_scaled(
            &self.grid,
            buffer_width,
            buffer_height,
            pixels.frame_mut().len(),
        );
        self.window = Some(window);
        self.pixels = Some(pixels);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(pixels) = self.pixels.as_mut() {
                    let buffer_width = size.width.min(self.grid.width()).max(1);
                    let buffer_height = size.height.min(self.grid.height()).max(1);

                    if buffer_width != self.buffer_width || buffer_height != self.buffer_height {
                        if pixels.resize_buffer(buffer_width, buffer_height).is_err() {
                            event_loop.exit();
                            return;
                        }
                        self.buffer_width = buffer_width;
                        self.buffer_height = buffer_height;
                        self.base_frame = build_base_frame_scaled(
                            &self.grid,
                            buffer_width,
                            buffer_height,
                            pixels.frame_mut().len(),
                        );
                    }

                    let _ = pixels.resize_surface(size.width, size.height);
                }
            }
            WindowEvent::RedrawRequested => {
                if let Some(pixels) = self.pixels.as_mut() {
                    let frame = pixels.frame_mut();
                    if frame.len() != self.base_frame.len() {
                        self.base_frame = build_base_frame_scaled(
                            &self.grid,
                            self.buffer_width,
                            self.buffer_height,
                            frame.len(),
                        );
                    }
                    frame.copy_from_slice(&self.base_frame);
                    if pixels.render().is_err() {
                        event_loop.exit();
                    }
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

fn build_base_frame(grid: &voxel_grid::OccupancyGrid) -> Vec<u8> {
    let width = grid.width();
    let height = grid.height();
    let mut frame = vec![0u8; (width as usize) * (height as usize) * 4];

    for img_y in 0..height {
        for x in 0..width {
            let grid_y = height - img_y - 1;
            let value = grid.get(x, grid_y).unwrap_or(-1);
            let [r, g, b, a] = map_occupancy_color(value);

            let idx = ((img_y * width + x) as usize) * 4;
            frame[idx] = r;
            frame[idx + 1] = g;
            frame[idx + 2] = b;
            frame[idx + 3] = a;
        }
    }

    frame
}

fn build_base_frame_scaled(
    grid: &voxel_grid::OccupancyGrid,
    buffer_width: u32,
    buffer_height: u32,
    frame_len: usize,
) -> Vec<u8> {
    let grid_width = grid.width();
    let grid_height = grid.height();
    let mut frame = vec![0u8; frame_len];

    for img_y in 0..buffer_height {
        let grid_y = grid_height - 1 - (img_y * grid_height / buffer_height);
        for img_x in 0..buffer_width {
            let grid_x = img_x * grid_width / buffer_width;
            let value = grid.get(grid_x, grid_y).unwrap_or(-1);
            let [r, g, b, a] = map_occupancy_color(value);

            let idx = ((img_y * buffer_width + img_x) as usize) * 4;
            if idx + 3 >= frame_len {
                continue;
            }
            frame[idx] = r;
            frame[idx + 1] = g;
            frame[idx + 2] = b;
            frame[idx + 3] = a;
        }
    }

    frame
}

fn map_occupancy_color(value: i8) -> [u8; 4] {
    if value < 0 {
        return [112, 137, 134, 255];
    }

    let clamped = (value as i32).clamp(0, 100) as f32;
    let shade = (255.0 - (clamped * 255.0) / 100.0) as u8;
    [shade, shade, shade, 255]
}
