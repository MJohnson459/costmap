//! Cost maps as single-band `u8` GeoTIFFs.
//!
//! The file is an ordinary GeoTIFF that GIS tools (QGIS, GDAL) open and edit directly:
//!
//! - Each pixel is a costmap cost: 0 is free, 254 is lethal, 255 is unknown.
//!   255 is declared as the GDAL nodata value.
//! - The raster is north-up: row 0 is the northern edge. [`TiledGrid`] has y pointing
//!   north, so rows are flipped between the two.
//! - The georeference is ModelPixelScale, ModelTiepoint and a GeoKeyDirectory with a
//!   projected CRS (PixelIsArea), plus the EPSG code when known.
//!
//! [`write_cost_geotiff`] writes a stripped, Deflate-compressed file followed by
//! max-pooled overviews at 1/2, 1/4, ... resolution until the larger side is at most 256
//! pixels. Each overview image carries GDAL band metadata `RESAMPLING=MAX`, the item GDAL
//! itself writes on the overviews it builds. The `tiff` encoder cannot write tiles, so the
//! file is a valid GeoTIFF but not a Cloud Optimized GeoTIFF. Convert it with
//! `gdal_translate -of COG -co COMPRESS=DEFLATE in.tif out.tif`; GDAL 3.11's COG driver
//! writes no `RESAMPLING` item on its overviews, even with
//! `-co OVERVIEWS=FORCE_USE_EXISTING`, so the converted file's overviews are not reported
//! max-pooled.
//!
//! [`CostGeoTiff`] reads any single-band 8-bit GeoTIFF, stripped or tiled, so files
//! converted or edited by GDAL load back. It decodes only the TIFF strips or tiles that
//! intersect a requested region.
//!
//! A map too large to hold at full resolution has two coarse views:
//! [`CostGeoTiff::overview`] opens one of the file's reduced-resolution images, and
//! [`CostGeoTiff::read_overview_max`] max-pools the full-resolution raster while
//! streaming it. GDAL's `gdaladdo` has no `max` resampling, so overviews it builds can
//! drop thin lethal features such as fences; [`CostGeoTiff::overview_is_max_pooled`]
//! tells the two apart.

use std::{
    collections::{HashMap, HashSet},
    fs::File,
    io::{BufReader, BufWriter, Read, Seek, Write},
    path::{Path, PathBuf},
};

use glam::{DVec2, IVec2, UVec2};
use tiff::{
    decoder::{Decoder, DecodingResult},
    encoder::{
        Compression, DeflateLevel, DirectoryEncoder, TiffEncoder, TiffKindStandard,
        colortype::Gray8,
    },
    tags::{CompressionMethod, Tag},
};

use crate::{
    tiled::{GeoReference, TiledGrid, max_known},
    types::{COST_UNKNOWN, VoxelError},
};

/// Target uncompressed strip size when writing.
const STRIP_BYTES: u32 = 64 * 1024;
/// Overviews are added until the larger raster side is at most this many pixels.
const OVERVIEW_MAX_SIDE: u32 = 256;
/// Largest drift, as a fraction of a cell, that a cost raster may accumulate between its
/// x and y pixel sizes over its longer side.
const SQUARE_CELL_DRIFT: f64 = 0.01;

const GEO_KEY_MODEL_TYPE: u16 = 1024;
const GEO_KEY_RASTER_TYPE: u16 = 1025;
const GEO_KEY_PROJECTED_CS: u16 = 3072;
const MODEL_TYPE_PROJECTED: u16 = 1;
const RASTER_PIXEL_IS_AREA: u16 = 1;
const RASTER_PIXEL_IS_POINT: u16 = 2;
/// GeoTIFF value for a CRS that is user-defined rather than an EPSG code.
const USER_DEFINED: u16 = 32767;
/// NewSubfileType bit marking a reduced-resolution image.
const SUBFILE_REDUCED_RESOLUTION: u32 = 1;
/// NewSubfileType bit marking a transparency mask.
const SUBFILE_MASK: u32 = 4;
/// GDAL's private tag holding an image's metadata as XML.
const GDAL_METADATA: Tag = Tag::Unknown(42112);
/// Band metadata written on every overview image. GDAL writes the same item, with its
/// own method name, on the overview images it builds, and reports it as the overview
/// band's `RESAMPLING` metadata.
const MAX_POOLED_OVERVIEW_METADATA: &str =
    r#"<GDALMetadata><Item name="RESAMPLING" sample="0">MAX</Item></GDALMetadata>"#;
/// Value of the `RESAMPLING` item on a max-pooled overview.
const MAX_RESAMPLING: &str = "MAX";

/// A single-band `u8` cost GeoTIFF opened for reading.
///
/// Local cell `(0, 0)` is the raster's bottom-left pixel and local x/y point east/north,
/// so cell `(x, y)` is raster pixel `(x, height - 1 - y)`.
pub struct CostGeoTiff {
    path: PathBuf,
    decoder: Decoder<BufReader<File>>,
    size: UVec2,
    resolution: f64,
    georef: GeoReference,
    /// Raster nodata value when it differs from [`COST_UNKNOWN`]; mapped to unknown on read.
    nodata: Option<u8>,
    /// Size in pixels of a full strip or tile.
    chunk: UVec2,
    /// Projected coordinates of the raster's top-left corner, kept in `f64` for overviews.
    top_left: DVec2,
    /// Pixel edge length in metres, in `f64`.
    pixel_size: f64,
    /// Reduced-resolution images in the file, finest first.
    overviews: Vec<Overview>,
    /// Grid chunks, keyed by chunk size, whose pixels were all unknown when decoded.
    /// [`load_region`](Self::load_region) skips them instead of decoding them again.
    empty_chunks: HashSet<(u32, IVec2)>,
}

/// A reduced-resolution image of the file.
#[derive(Debug, Clone)]
struct Overview {
    /// Index of the image in the file's IFD chain.
    ifd: usize,
    size: UVec2,
    /// Full-resolution pixels per overview pixel along each axis.
    factor: u32,
    /// The `RESAMPLING` item of the image's GDAL metadata, if it has one.
    resampling: Option<String>,
}

impl CostGeoTiff {
    /// Open `path` and parse its georeferencing.
    ///
    /// Fails if the raster is not a single-band unsigned 8-bit image, has no pixel scale
    /// and tiepoint, is not in a projected CRS, or has non-square pixels: cells of a
    /// [`TiledGrid`] are square, so the x and y pixel sizes may differ only until the
    /// difference accumulated over the longer raster side reaches 1 % of a pixel.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, VoxelError> {
        let path = path.as_ref().to_path_buf();
        let mut decoder = Decoder::new(BufReader::new(File::open(&path)?))?;
        check_sample_type(&mut decoder)?;
        let (width, height) = decoder.dimensions()?;
        let size = UVec2::new(width, height);
        let frame = read_frame(&mut decoder)?;
        check_square_cells(frame.pixel_size, size)?;
        let georef = GeoReference {
            epsg: frame.epsg,
            anchor: frame.top_left - DVec2::new(0.0, f64::from(height) * frame.pixel_size.y),
        };
        let nodata = read_nodata(&mut decoder)?;
        let chunk = tiff_chunk_size(&decoder, size);
        let overviews = find_overviews(&mut decoder, size)?;
        decoder.seek_to_image(0)?;
        Ok(Self {
            path,
            decoder,
            size,
            resolution: frame.pixel_size.x,
            georef,
            nodata,
            chunk,
            top_left: frame.top_left,
            pixel_size: frame.pixel_size.x,
            overviews,
            empty_chunks: HashSet::new(),
        })
    }

    /// Reduction factor of each overview in the file, finest first.
    ///
    /// Overview `i` has `factor` full-resolution pixels per pixel along each axis. Only
    /// single-band 8-bit images that reduce both axes by the same whole factor are listed.
    pub fn overview_factors(&self) -> Vec<u32> {
        self.overviews.iter().map(|o| o.factor).collect()
    }

    /// The resampling method recorded on overview `index`: the `RESAMPLING` item of the
    /// image's GDAL metadata (TIFF tag 42112), as `gdalinfo` reports it for the overview
    /// band. `None` if the index is out of range or the image records no method.
    ///
    /// [`write_cost_geotiff`] records `MAX`. GDAL 3.11 records the `-r` method of
    /// `gdaladdo` and the `RESAMPLING` creation option of the COG driver, such as
    /// `AVERAGE` or `NEAREST`; older versions and other tools may record nothing.
    pub fn overview_resampling(&self, index: usize) -> Option<&str> {
        self.overviews.get(index)?.resampling.as_deref()
    }

    /// Whether overview `index` is recorded as max-pooled: each pixel holds the highest
    /// known cost of the full-resolution pixels it covers.
    ///
    /// True only when [`overview_resampling`](Self::overview_resampling) is `MAX`
    /// (case-insensitive). A level with any other method, or none, may have dropped thin
    /// lethal features such as fences; plan on
    /// [`read_overview_max`](Self::read_overview_max) instead.
    ///
    /// The record is trusted as written. GDAL 3.11's `gdaladdo -r max` fails, as GDAL has
    /// no max overview resampling, but first records `RESAMPLING=MAX` on the levels it
    /// was asked to build, which keep their previous pixels, or none.
    pub fn overview_is_max_pooled(&self, index: usize) -> bool {
        self.overview_resampling(index)
            .is_some_and(|method| method.eq_ignore_ascii_case(MAX_RESAMPLING))
    }

    /// Open overview `index` (an index into [`overview_factors`](Self::overview_factors))
    /// as a reader of its own.
    ///
    /// The overview's pixel edge is this file's times the factor, and its top-left corner
    /// is this file's top-left corner. Its [`georeference`](Self::georeference) anchor is
    /// its own lower-left corner, which lies up to `factor - 1` full-resolution pixels
    /// south of this file's when the height is not a multiple of the factor; convert
    /// between the two local frames through projected coordinates. GDAL computes overview
    /// pixels from a fractional ratio when a side is not a multiple of the factor, so
    /// such files can be off by up to one overview pixel near the east and south edges.
    ///
    /// Values are whatever the file's writer stored: [`write_cost_geotiff`] max-pools,
    /// other tools may average or subsample; see
    /// [`overview_is_max_pooled`](Self::overview_is_max_pooled). The returned reader
    /// lists no overviews.
    pub fn overview(&self, index: usize) -> Result<CostGeoTiff, VoxelError> {
        let overview = self.overviews.get(index).ok_or_else(|| {
            VoxelError::InvalidArgument(format!(
                "overview {index} does not exist; the file has {}",
                self.overviews.len()
            ))
        })?;
        let mut decoder = Decoder::new(BufReader::new(File::open(&self.path)?))?;
        decoder.seek_to_image(overview.ifd)?;
        check_sample_type(&mut decoder)?;
        let pixel_size = self.pixel_size * f64::from(overview.factor);
        let height = f64::from(overview.size.y) * pixel_size;
        Ok(Self {
            path: self.path.clone(),
            chunk: tiff_chunk_size(&decoder, overview.size),
            decoder,
            size: overview.size,
            resolution: pixel_size,
            georef: GeoReference {
                epsg: self.georef.epsg,
                anchor: self.top_left - DVec2::new(0.0, height),
            },
            nodata: self.nodata,
            top_left: self.top_left,
            pixel_size,
            overviews: Vec::new(),
            empty_chunks: HashSet::new(),
        })
    }

    /// Raster size in pixels.
    pub fn size(&self) -> UVec2 {
        self.size
    }

    /// Pixel edge length in metres.
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Georeference whose anchor is the raster's lower-left corner in projected coordinates.
    pub fn georeference(&self) -> GeoReference {
        self.georef
    }

    /// Load the whole raster into a new grid with the given chunk size.
    pub fn read_all(&mut self, chunk_size: u32) -> Result<TiledGrid<u8>, VoxelError> {
        let mut grid = TiledGrid::new(self.resolution, chunk_size, COST_UNKNOWN)?;
        self.load_region(&mut grid, IVec2::ZERO, self.size.as_ivec2())?;
        Ok(grid)
    }

    /// Load the chunks of `grid` that intersect the cell region `[min_cell, max_cell)`
    /// and are not yet loaded, and return how many were loaded.
    ///
    /// Only the TIFF strips or tiles under those chunks are decoded. Chunks whose pixels
    /// are all unknown, or that lie outside the raster, are not allocated. `grid` must have
    /// the file's resolution and [`COST_UNKNOWN`] as its fill value, so unallocated chunks
    /// read as unknown. The reader remembers chunks it found entirely unknown and does not
    /// decode them again on later calls with the same chunk size.
    pub fn load_region(
        &mut self,
        grid: &mut TiledGrid<u8>,
        min_cell: IVec2,
        max_cell: IVec2,
    ) -> Result<usize, VoxelError> {
        self.check_grid(grid)?;
        let chunk_size = grid.chunk_size();
        let mut wanted = wanted_chunks(grid, self.size, min_cell, max_cell);
        wanted.retain(|&coord| !self.empty_chunks.contains(&(chunk_size, coord)));
        let Some(pixels) = pixel_bounds(chunk_size, self.size, &wanted) else {
            return Ok(0);
        };
        let wanted: HashSet<IVec2> = wanted.into_iter().collect();
        let mut stamper = ChunkStamper::new(grid.chunk_size(), self.size.y, self.nodata, &wanted);
        for chunk in self.tiff_chunks_in(pixels) {
            let data = self.read_chunk(&chunk)?;
            stamper.stamp(chunk.rect, &data);
        }
        let loaded = stamper.chunks;
        for &coord in &wanted {
            if !loaded.contains_key(&coord) {
                self.empty_chunks.insert((chunk_size, coord));
            }
        }
        let count = loaded.len();
        for (coord, data) in loaded {
            grid.insert_chunk(coord, data.into_boxed_slice())?;
        }
        Ok(count)
    }

    /// Max-pool the raster into a grid of `factor` x `factor` pixel blocks, decoding one
    /// TIFF strip or tile at a time.
    ///
    /// The result shares this file's local frame: block `(i, j)` covers cells
    /// `[i * factor, (i + 1) * factor)` by `[j * factor, (j + 1) * factor)`, its
    /// resolution is `resolution() * factor`, and its fill value is [`COST_UNKNOWN`].
    /// A block is unknown only if every pixel in it is, as in
    /// [`TiledGrid::overview_max`]. Memory use is the output grid plus one decoded strip
    /// or tile, so this works on rasters far larger than memory.
    pub fn read_overview_max(
        &mut self,
        factor: u32,
        chunk_size: u32,
    ) -> Result<TiledGrid<u8>, VoxelError> {
        if factor == 0 {
            return Err(VoxelError::InvalidArgument(
                "overview factor must be >= 1".into(),
            ));
        }
        let mut grid = TiledGrid::new(
            self.pixel_size * f64::from(factor),
            chunk_size,
            COST_UNKNOWN,
        )?;
        let everything = PixelRect {
            x: 0,
            y: 0,
            width: self.size.x,
            height: self.size.y,
        };
        for chunk in self.tiff_chunks_in(everything) {
            let data = self.read_chunk(&chunk)?;
            let blocks = BlockPool::new(chunk.rect, self.size.y, factor);
            let pooled = blocks.pool(chunk.rect, &data, self.nodata);
            let width = blocks.size.x as usize;
            for (i, &value) in pooled.iter().enumerate() {
                if value != COST_UNKNOWN {
                    let block = blocks.first + IVec2::new((i % width) as i32, (i / width) as i32);
                    let slot = grid.cell_mut(block);
                    *slot = max_known(*slot, value);
                }
            }
        }
        Ok(grid)
    }

    fn check_grid(&self, grid: &TiledGrid<u8>) -> Result<(), VoxelError> {
        if !grid.resolution_matches(self.resolution) {
            return Err(VoxelError::InvalidArgument(format!(
                "grid resolution {} differs from file resolution {}",
                grid.resolution(),
                self.resolution
            )));
        }
        if *grid.fill_value() != COST_UNKNOWN {
            return Err(VoxelError::InvalidArgument(
                "grid fill value must be COST_UNKNOWN".into(),
            ));
        }
        Ok(())
    }

    /// The strips or tiles intersecting `pixels`, with each one's pixel rectangle.
    fn tiff_chunks_in(&self, pixels: PixelRect) -> Vec<TiffChunk> {
        let per_row = self.size.x.div_ceil(self.chunk.x);
        let cols = pixels.x / self.chunk.x..=(pixels.x + pixels.width - 1) / self.chunk.x;
        let rows = pixels.y / self.chunk.y..=(pixels.y + pixels.height - 1) / self.chunk.y;
        let mut chunks = Vec::new();
        for row in rows {
            for col in cols.clone() {
                let (x, y) = (col * self.chunk.x, row * self.chunk.y);
                chunks.push(TiffChunk {
                    index: row * per_row + col,
                    rect: PixelRect {
                        x,
                        y,
                        width: self.chunk.x.min(self.size.x - x),
                        height: self.chunk.y.min(self.size.y - y),
                    },
                });
            }
        }
        chunks
    }

    /// Decode `chunk` and check it holds one sample per pixel of its rectangle.
    fn read_chunk(&mut self, chunk: &TiffChunk) -> Result<Vec<u8>, VoxelError> {
        let decoded = self
            .decoder
            .read_chunk(chunk.index)
            .map_err(|err| explain_decode_error(&mut self.decoder, chunk.index, err))?;
        let DecodingResult::U8(data) = decoded else {
            return Err(VoxelError::Unsupported(
                "TIFF chunk did not decode to 8-bit samples".into(),
            ));
        };
        if data.len() != chunk.rect.area() {
            return Err(VoxelError::InvalidMetadata(format!(
                "TIFF chunk {} holds {} samples, expected {}",
                chunk.index,
                data.len(),
                chunk.rect.area()
            )));
        }
        Ok(data)
    }
}

/// Replace the `tiff` crate's LZW end-of-stream failure with an actionable error.
///
/// `err` is the failure of reading chunk `index` from `decoder`. tiff 0.11's `LZWReader`
/// turns weezl's `NoProgress` status into `UnexpectedEof("no lzw end code found")` even
/// when that same call returned the last decoded bytes, so some valid LZW tiles (seen on
/// partial edge tiles of GDAL COGs) cannot be read. For such a failure on an LZW raster
/// this returns [`VoxelError::Unsupported`] naming the `COMPRESS=DEFLATE` workaround; any
/// other failure is returned unchanged.
pub fn explain_decode_error<R: Read + Seek>(
    decoder: &mut Decoder<R>,
    index: u32,
    err: tiff::TiffError,
) -> VoxelError {
    let truncated = matches!(
        &err,
        tiff::TiffError::IoError(io) if io.kind() == std::io::ErrorKind::UnexpectedEof
    );
    let lzw = decoder
        .find_tag_unsigned::<u16>(Tag::Compression)
        .is_ok_and(|method| method == Some(CompressionMethod::LZW.to_u16()));
    if truncated && lzw {
        VoxelError::Unsupported(format!(
            "LZW chunk {index} could not be decoded ({err}); the tiff crate fails on some \
             valid LZW tiles, so re-encode the file with \
             `gdal_translate -of COG -co COMPRESS=DEFLATE`"
        ))
    } else {
        err.into()
    }
}

/// A rectangle of raster pixels, origin at the top-left corner.
#[derive(Debug, Clone, Copy)]
struct PixelRect {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
}

impl PixelRect {
    fn area(&self) -> usize {
        self.width as usize * self.height as usize
    }
}

struct TiffChunk {
    index: u32,
    rect: PixelRect,
}

/// The `factor` x `factor` pixel blocks, in cell coordinates, that a raster rectangle touches.
struct BlockPool {
    first: IVec2,
    size: UVec2,
    factor: u32,
    raster_height: u32,
}

impl BlockPool {
    fn new(rect: PixelRect, raster_height: u32, factor: u32) -> Self {
        // Raster row r is cell row raster_height - 1 - r, so the rectangle's bottom row
        // holds its lowest cell row.
        let low_cell_y = raster_height - (rect.y + rect.height);
        let high_cell_y = raster_height - 1 - rect.y;
        let first = UVec2::new(rect.x / factor, low_cell_y / factor);
        let last = UVec2::new((rect.x + rect.width - 1) / factor, high_cell_y / factor);
        Self {
            first: first.as_ivec2(),
            size: last - first + 1,
            factor,
            raster_height,
        }
    }

    /// Max of the known pixels of `data` (the samples of `rect`) per block, row-major
    /// from the southern block row; [`COST_UNKNOWN`] where a block has none.
    fn pool(&self, rect: PixelRect, data: &[u8], nodata: Option<u8>) -> Vec<u8> {
        let mut pooled = vec![COST_UNKNOWN; self.size.x as usize * self.size.y as usize];
        let first = self.first.as_uvec2();
        for (row, line) in data.chunks_exact(rect.width as usize).enumerate() {
            let cell_y = self.raster_height - 1 - (rect.y + row as u32);
            let block_row = (cell_y / self.factor - first.y) as usize * self.size.x as usize;
            for (col, &value) in line.iter().enumerate() {
                if value == COST_UNKNOWN || Some(value) == nodata {
                    continue;
                }
                let block_x = ((rect.x + col as u32) / self.factor - first.x) as usize;
                let slot = &mut pooled[block_row + block_x];
                *slot = max_known(*slot, value);
            }
        }
        pooled
    }
}

/// Grid chunks that intersect `[min_cell, max_cell)` clipped to the raster and are not loaded.
fn wanted_chunks(
    grid: &TiledGrid<u8>,
    size: UVec2,
    min_cell: IVec2,
    max_cell: IVec2,
) -> Vec<IVec2> {
    let lo = min_cell.as_i64vec2().max(glam::I64Vec2::ZERO);
    let hi = max_cell.as_i64vec2().min(size.as_i64vec2());
    if lo.x >= hi.x || lo.y >= hi.y {
        return Vec::new();
    }
    let cs = i64::from(grid.chunk_size());
    let first = lo.div_euclid(glam::I64Vec2::splat(cs));
    let last = (hi - 1).div_euclid(glam::I64Vec2::splat(cs));
    let mut wanted = Vec::new();
    for y in first.y..=last.y {
        for x in first.x..=last.x {
            let coord = IVec2::new(x as i32, y as i32);
            if !grid.is_loaded(coord) {
                wanted.push(coord);
            }
        }
    }
    wanted
}

/// Raster pixels covered by the `wanted` grid chunks, clipped to the raster.
///
/// Cell row `y` is raster row `height - 1 - y`, so the cell range `[y0, y1)` is the
/// pixel range `[height - y1, height - y0)`.
fn pixel_bounds(chunk_size: u32, size: UVec2, wanted: &[IVec2]) -> Option<PixelRect> {
    let first = wanted.first()?;
    let (min, max) = wanted
        .iter()
        .fold((*first, *first), |(min, max), c| (min.min(*c), max.max(*c)));
    let cs = i64::from(chunk_size);
    let (width, height) = (i64::from(size.x), i64::from(size.y));
    let x0 = (i64::from(min.x) * cs).max(0);
    let x1 = ((i64::from(max.x) + 1) * cs).min(width);
    let y0 = (i64::from(min.y) * cs).max(0);
    let y1 = ((i64::from(max.y) + 1) * cs).min(height);
    Some(PixelRect {
        x: x0 as u32,
        y: (height - y1) as u32,
        width: (x1 - x0) as u32,
        height: (y1 - y0) as u32,
    })
}

/// Accumulates decoded TIFF chunks into grid chunks, allocating one only when it receives a known cell.
struct ChunkStamper<'a> {
    chunk_size: i64,
    raster_height: i64,
    nodata: Option<u8>,
    wanted: &'a HashSet<IVec2>,
    chunks: HashMap<IVec2, Vec<u8>>,
}

impl<'a> ChunkStamper<'a> {
    fn new(
        chunk_size: u32,
        raster_height: u32,
        nodata: Option<u8>,
        wanted: &'a HashSet<IVec2>,
    ) -> Self {
        Self {
            chunk_size: i64::from(chunk_size),
            raster_height: i64::from(raster_height),
            nodata,
            wanted,
            chunks: HashMap::new(),
        }
    }

    /// Copy the decoded `data` of the TIFF chunk covering `rect` into the wanted grid chunks.
    fn stamp(&mut self, rect: PixelRect, data: &[u8]) {
        let cs = self.chunk_size;
        let (x0, width) = (i64::from(rect.x), i64::from(rect.width));
        for row in 0..i64::from(rect.height) {
            // Raster row to cell row: the raster is north-up, cell y points north.
            let cell_y = self.raster_height - 1 - (i64::from(rect.y) + row);
            let grid_y = cell_y.div_euclid(cs);
            let local_y = cell_y.rem_euclid(cs);
            let line = &data[(row * width) as usize..((row + 1) * width) as usize];
            for grid_x in x0.div_euclid(cs)..=(x0 + width - 1).div_euclid(cs) {
                let coord = IVec2::new(grid_x as i32, grid_y as i32);
                if !self.wanted.contains(&coord) {
                    continue;
                }
                let start = x0.max(grid_x * cs);
                let end = (x0 + width).min(grid_x * cs + cs);
                let src = &line[(start - x0) as usize..(end - x0) as usize];
                let dst_start = (local_y * cs + start - grid_x * cs) as usize;
                self.stamp_run(coord, dst_start, src);
            }
        }
    }

    fn stamp_run(&mut self, coord: IVec2, dst_start: usize, src: &[u8]) {
        let nodata = self.nodata;
        let to_cost = |value: u8| {
            if Some(value) == nodata {
                COST_UNKNOWN
            } else {
                value
            }
        };
        if src.iter().all(|&value| to_cost(value) == COST_UNKNOWN) {
            return;
        }
        let len = (self.chunk_size * self.chunk_size) as usize;
        let chunk = self
            .chunks
            .entry(coord)
            .or_insert_with(|| vec![COST_UNKNOWN; len]);
        for (dst, &value) in chunk[dst_start..dst_start + src.len()].iter_mut().zip(src) {
            *dst = to_cost(value);
        }
    }
}

/// Reject a raster whose x and y pixel sizes drift apart by a visible part of a cell.
///
/// A [`TiledGrid`] has one cell size, so the raster is read with the x size and the
/// y positions are off by up to `longer side * |sx - sy|` metres at the far edge.
fn check_square_cells(pixel_size: DVec2, size: UVec2) -> Result<(), VoxelError> {
    let drift = f64::from(size.x.max(size.y)) * (pixel_size.x - pixel_size.y).abs();
    if drift <= SQUARE_CELL_DRIFT * pixel_size.x.min(pixel_size.y) {
        return Ok(());
    }
    let rounded = |v: f64| (v * 1e9).round() / 1e9;
    let (x, y) = (rounded(pixel_size.x), rounded(pixel_size.y));
    Err(VoxelError::Unsupported(format!(
        "pixels are not square ({x} x {y} m) and a cost grid has square cells; \
         resample with `gdalwarp -tr {x} {x}`"
    )))
}

/// Georeferencing read from a GeoTIFF's first image.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoFrame {
    /// Projected coordinates of the raster's top-left corner.
    pub top_left: DVec2,
    /// Pixel width (x) and height (y) in metres. They differ for anisotropic rasters.
    pub pixel_size: DVec2,
    /// EPSG code of the projected CRS, if the file names one.
    pub epsg: Option<u32>,
}

fn check_sample_type(decoder: &mut Decoder<BufReader<File>>) -> Result<(), VoxelError> {
    let color_type = decoder.colortype()?;
    let formats = decoder.find_tag_unsigned_vec::<u16>(Tag::SampleFormat)?;
    let unsigned = formats.as_deref().is_none_or(|f| f.iter().all(|&s| s == 1));
    if color_type != tiff::ColorType::Gray(8) || !unsigned {
        return Err(VoxelError::Unsupported(format!(
            "expected a single-band unsigned 8-bit GeoTIFF, found {color_type:?} with sample format {}",
            formats.map_or_else(|| "unsigned (default)".into(), |f| format!("{f:?}"))
        )));
    }
    Ok(())
}

fn find_f64s<R: Read + Seek>(
    decoder: &mut Decoder<R>,
    tag: Tag,
) -> Result<Option<Vec<f64>>, VoxelError> {
    Ok(decoder
        .find_tag(tag)?
        .map(|value| value.into_f64_vec())
        .transpose()?)
}

/// Read the pixel scale, tiepoint and CRS of the decoder's current image.
///
/// Fails if the pixel scale or tiepoint is missing or the CRS is not projected. Pixels
/// need not be square; both sizes are returned. The tiepoint is normalised to the top-left pixel corner whether the
/// file uses PixelIsArea or PixelIsPoint.
pub fn read_frame<R: Read + Seek>(decoder: &mut Decoder<R>) -> Result<GeoFrame, VoxelError> {
    let not_georeferenced = || {
        VoxelError::InvalidMetadata(
            "GeoTIFF has no ModelPixelScale and ModelTiepoint tags (rotated or unreferenced rasters are unsupported)"
                .into(),
        )
    };
    let scale = find_f64s(decoder, Tag::ModelPixelScaleTag)?.ok_or_else(not_georeferenced)?;
    let tiepoint = find_f64s(decoder, Tag::ModelTiepointTag)?.ok_or_else(not_georeferenced)?;
    let &[scale_x, scale_y, ..] = scale.as_slice() else {
        return Err(VoxelError::InvalidMetadata(
            "ModelPixelScale needs 3 values".into(),
        ));
    };
    let &[tie_i, tie_j, _, tie_x, tie_y, ..] = tiepoint.as_slice() else {
        return Err(VoxelError::InvalidMetadata(
            "ModelTiepoint needs 6 values".into(),
        ));
    };
    if !(scale_x.is_finite() && scale_y.is_finite() && scale_x > 0.0 && scale_y > 0.0) {
        return Err(VoxelError::InvalidMetadata(format!(
            "pixel scale ({scale_x}, {scale_y}) must be finite and positive"
        )));
    }
    let keys = decoder
        .find_tag_unsigned_vec::<u16>(Tag::GeoKeyDirectoryTag)?
        .map(|directory| parse_geo_keys(&directory))
        .unwrap_or_default();
    if keys.model_type.is_some_and(|m| m != MODEL_TYPE_PROJECTED) {
        return Err(VoxelError::Unsupported(
            "only projected coordinate reference systems are supported".into(),
        ));
    }
    // The tiepoint names the position of pixel corner (i, j); under PixelIsPoint
    // the pixel is a sample at its centre, which puts the corner half a pixel out.
    let half_pixel = if keys.raster_type == Some(RASTER_PIXEL_IS_POINT) {
        0.5
    } else {
        0.0
    };
    Ok(GeoFrame {
        top_left: DVec2::new(
            tie_x - (tie_i + half_pixel) * scale_x,
            tie_y + (tie_j + half_pixel) * scale_y,
        ),
        pixel_size: DVec2::new(scale_x, scale_y),
        epsg: keys.epsg,
    })
}

/// Read the GDAL nodata value of the decoder's current image.
///
/// Returns `None` if the tag is absent or its text is not a number.
pub fn read_gdal_nodata<R: Read + Seek>(
    decoder: &mut Decoder<R>,
) -> Result<Option<f64>, VoxelError> {
    let Some(text) = decoder
        .find_tag(Tag::GdalNodata)?
        .map(|value| value.into_string())
        .transpose()?
    else {
        return Ok(None);
    };
    Ok(text.trim_end_matches('\0').trim().parse::<f64>().ok())
}

fn read_nodata(decoder: &mut Decoder<BufReader<File>>) -> Result<Option<u8>, VoxelError> {
    let nodata = read_gdal_nodata(decoder)?
        .filter(|v| v.fract() == 0.0 && (0.0..=255.0).contains(v))
        .map(|v| v as u8);
    Ok(nodata.filter(|&v| v != COST_UNKNOWN))
}

/// Size in pixels of a full strip or tile of the decoder's current image.
fn tiff_chunk_size(decoder: &Decoder<BufReader<File>>, size: UVec2) -> UVec2 {
    let (width, height) = decoder.chunk_dimensions();
    UVec2::new(width.min(size.x), height.min(size.y))
}

/// Walk the IFD chain after the current (first) image and list the reduced-resolution
/// single-band 8-bit images whose sides are `size` divided by a whole factor, rounded up.
///
/// Overviews are optional, so the walk stops at the first image the decoder cannot
/// parse instead of failing. Leaves the decoder on the last image visited.
fn find_overviews(
    decoder: &mut Decoder<BufReader<File>>,
    size: UVec2,
) -> Result<Vec<Overview>, VoxelError> {
    let mut overviews = Vec::new();
    let mut ifd = 0;
    while decoder.more_images() && decoder.next_image().is_ok() {
        ifd += 1;
        let subfile = decoder
            .find_tag_unsigned::<u32>(Tag::NewSubfileType)?
            .unwrap_or(0);
        if subfile & SUBFILE_REDUCED_RESOLUTION == 0
            || subfile & SUBFILE_MASK != 0
            || check_sample_type(decoder).is_err()
        {
            continue;
        }
        let (width, height) = decoder.dimensions()?;
        let reduced = UVec2::new(width, height);
        if let Some(factor) = overview_factor(size, reduced) {
            overviews.push(Overview {
                ifd,
                size: reduced,
                factor,
                resampling: read_resampling(decoder)?,
            });
        }
    }
    overviews.sort_by_key(|o| o.factor);
    Ok(overviews)
}

/// The `RESAMPLING` item of the decoder's current image's GDAL metadata, if any.
fn read_resampling<R: Read + Seek>(decoder: &mut Decoder<R>) -> Result<Option<String>, VoxelError> {
    let Some(xml) = decoder
        .find_tag(GDAL_METADATA)?
        .map(|value| value.into_string())
        .transpose()?
    else {
        return Ok(None);
    };
    Ok(metadata_item(&xml, "RESAMPLING").map(str::to_owned))
}

/// Text of the first `<Item name="{name}" ...>text</Item>` element of GDAL metadata XML.
///
/// GDAL writes plain words for the items read here, so entities are not decoded.
fn metadata_item<'a>(xml: &'a str, name: &str) -> Option<&'a str> {
    let wanted = format!("name=\"{name}\"");
    xml.split("<Item").skip(1).find_map(|item| {
        let (attributes, rest) = item.split_once('>')?;
        let (text, _) = rest.split_once("</Item>")?;
        attributes
            .split_whitespace()
            .any(|attribute| attribute == wanted)
            .then(|| text.trim())
    })
}

/// The whole factor `f >= 2` that reduces `size` to `reduced` with GDAL's rule
/// `reduced = ceil(size / f)` on both axes.
///
/// Several factors can satisfy both axes when the reduced sides are small; a power of
/// two among them is preferred, as GDAL and [`write_cost_geotiff`] build power-of-two
/// levels by default. Returns `None` if no factor fits both axes, or if several fit and
/// none of them is a power of two.
fn overview_factor(size: UVec2, reduced: UVec2) -> Option<u32> {
    let (x_lo, x_hi) = factor_range(size.x, reduced.x)?;
    let (y_lo, y_hi) = factor_range(size.y, reduced.y)?;
    let (lo, hi) = (x_lo.max(y_lo).max(2), x_hi.min(y_hi));
    if lo > hi {
        return None;
    }
    let power_of_two = lo.checked_next_power_of_two().filter(|&p| p <= hi);
    power_of_two.or((lo == hi).then_some(lo))
}

/// Inclusive range of the factors `f` with `ceil(side / f) == reduced`, or `None` if
/// there are none. The range is unbounded above (`u32::MAX`) when `reduced` is 1.
fn factor_range(side: u32, reduced: u32) -> Option<(u32, u32)> {
    if reduced == 0 || reduced > side {
        return None;
    }
    // ceil(side / f) == reduced  <=>  (reduced - 1) * f < side <= reduced * f.
    let lo = side.div_ceil(reduced);
    let hi = if reduced == 1 {
        u32::MAX
    } else {
        (side - 1) / (reduced - 1)
    };
    (lo <= hi).then_some((lo, hi))
}

/// The GeoKeys this module reads.
#[derive(Default)]
struct GeoKeys {
    model_type: Option<u16>,
    raster_type: Option<u16>,
    epsg: Option<u32>,
}

/// Parse a GeoKeyDirectory: a 4-short header `[version, revision, minor, key_count]`
/// followed by `[key, tag_location, count, value]` entries. Only keys stored inline
/// (`tag_location == 0`) are read.
fn parse_geo_keys(directory: &[u16]) -> GeoKeys {
    let mut keys = GeoKeys::default();
    let Some((header, entries)) = directory.split_first_chunk::<4>() else {
        return keys;
    };
    let (entries, _) = entries.as_chunks::<4>();
    for &[key, location, _, value] in entries.iter().take(usize::from(header[3])) {
        if location != 0 {
            continue;
        }
        match key {
            GEO_KEY_MODEL_TYPE => keys.model_type = Some(value),
            GEO_KEY_RASTER_TYPE => keys.raster_type = Some(value),
            GEO_KEY_PROJECTED_CS if value != USER_DEFINED && value != 0 => {
                keys.epsg = Some(u32::from(value));
            }
            _ => {}
        }
    }
    keys
}

/// One raster level, rows north to south.
struct Level {
    data: Vec<u8>,
    size: UVec2,
}

/// Write the loaded chunks of `grid` as a GeoTIFF at `path`.
///
/// The raster covers the bounding box of the loaded chunks, with unloaded cells inside it
/// written as unknown. The tiepoint places it at the same projected coordinates as the
/// grid has under `georef`, so after re-opening, local cell indices are shifted by the
/// bounding box's minimum cell but [`GeoReference::to_projected`] of every cell centre is
/// unchanged. See the module documentation for the file layout.
///
/// Fails if the grid has no loaded chunks or the bounding box exceeds `u32` pixels.
pub fn write_cost_geotiff(
    path: impl AsRef<Path>,
    grid: &TiledGrid<u8>,
    georef: &GeoReference,
) -> Result<(), VoxelError> {
    let (min_cell, max_cell) = grid.loaded_cell_bounds().ok_or_else(|| {
        VoxelError::InvalidArgument(
            "cannot write a GeoTIFF for a grid with no loaded chunks".into(),
        )
    })?;
    write_cost_geotiff_region(path, grid, georef, min_cell, max_cell)
}

/// Write the cells `[min_cell, max_cell)` of `grid` as a GeoTIFF at `path`.
///
/// Like [`write_cost_geotiff`], but the raster covers exactly the given cell region rather
/// than the bounding box of the loaded chunks, so a map whose size is not a multiple of the
/// chunk size is not padded. Cells in unloaded chunks are written as unknown.
///
/// Fails if the region is empty or exceeds `u32` pixels.
pub fn write_cost_geotiff_region(
    path: impl AsRef<Path>,
    grid: &TiledGrid<u8>,
    georef: &GeoReference,
    min_cell: IVec2,
    max_cell: IVec2,
) -> Result<(), VoxelError> {
    let size = raster_size(min_cell, max_cell)?;
    if size.x == 0 || size.y == 0 {
        return Err(VoxelError::InvalidArgument(
            "cannot write a GeoTIFF for an empty region".into(),
        ));
    }
    let levels = build_levels(raster_from_grid(grid, min_cell, size), size);
    let resolution = grid.resolution();
    // The lower-left corner is computed from the cell index so the tiepoint
    // does not pick up rounding at large offsets.
    let lower_left = georef.anchor + min_cell.as_dvec2() * resolution;
    let top_left = lower_left + DVec2::new(0.0, f64::from(size.y) * resolution);

    let mut file = BufWriter::new(File::create(path)?);
    let mut encoder =
        TiffEncoder::new(&mut file)?.with_compression(Compression::Deflate(DeflateLevel::Balanced));
    for (index, level) in levels.iter().enumerate() {
        let mut image = encoder.new_image::<Gray8>(level.size.x, level.size.y)?;
        image.rows_per_strip(rows_per_strip(level.size))?;
        if index == 0 {
            write_geo_tags(image.encoder(), resolution, top_left, georef.epsg)?;
        } else {
            image
                .encoder()
                .write_tag(Tag::NewSubfileType, SUBFILE_REDUCED_RESOLUTION)?;
            image
                .encoder()
                .write_tag(GDAL_METADATA, MAX_POOLED_OVERVIEW_METADATA)?;
        }
        image.write_data(&level.data)?;
    }
    file.flush()?;
    Ok(())
}

fn raster_size(min_cell: IVec2, max_cell: IVec2) -> Result<UVec2, VoxelError> {
    let extent = max_cell.as_i64vec2() - min_cell.as_i64vec2();
    let too_large = || {
        VoxelError::InvalidArgument(format!(
            "loaded extent {}x{} cells exceeds the TIFF pixel limit",
            extent.x, extent.y
        ))
    };
    Ok(UVec2::new(
        u32::try_from(extent.x).map_err(|_| too_large())?,
        u32::try_from(extent.y).map_err(|_| too_large())?,
    ))
}

/// Dense raster of `[min_cell, min_cell + size)` with rows ordered north to south.
///
/// Grid rows run south to north, so each is copied into the mirrored raster row.
fn raster_from_grid(grid: &TiledGrid<u8>, min_cell: IVec2, size: UVec2) -> Vec<u8> {
    let width = size.x as usize;
    let mut raster = vec![COST_UNKNOWN; width * size.y as usize];
    if width == 0 {
        return raster;
    }
    let row_size = UVec2::new(size.x, 1);
    for (north, row) in raster.chunks_exact_mut(width).enumerate() {
        let y = min_cell.y + (size.y as usize - 1 - north) as i32;
        grid.copy_out(IVec2::new(min_cell.x, y), row_size, row);
    }
    raster
}

/// The full-resolution level followed by 2x max-pooled levels.
fn build_levels(raster: Vec<u8>, size: UVec2) -> Vec<Level> {
    let mut levels = vec![Level { data: raster, size }];
    while levels
        .last()
        .is_some_and(|l| l.size.max_element() > OVERVIEW_MAX_SIDE)
    {
        let Some(previous) = levels.last() else { break };
        levels.push(pool_2x2(previous));
    }
    levels
}

/// Max-pool 2x2 blocks anchored at the top-left pixel, as GDAL does for overviews.
///
/// An odd trailing row or column forms a smaller block. A block is unknown only if
/// every cell in it is unknown.
fn pool_2x2(level: &Level) -> Level {
    let size = (level.size + 1) / 2;
    let width = level.size.x as usize;
    let mut data = Vec::with_capacity(size.x as usize * size.y as usize);
    for rows in level.data.chunks(2 * width) {
        let (top, bottom) = rows.split_at(width.min(rows.len()));
        for x in 0..size.x as usize {
            let columns = 2 * x..(2 * x + 2).min(width);
            let block = top[columns.clone()]
                .iter()
                .chain(bottom.get(columns).unwrap_or_default());
            data.push(block.fold(COST_UNKNOWN, |acc, &v| max_known(acc, v)));
        }
    }
    Level { data, size }
}

/// Rows per strip that keeps an uncompressed strip near [`STRIP_BYTES`].
fn rows_per_strip(size: UVec2) -> u32 {
    (STRIP_BYTES / size.x).clamp(1, size.y)
}

fn write_geo_tags<W: Write + Seek>(
    directory: &mut DirectoryEncoder<'_, W, TiffKindStandard>,
    resolution: f64,
    top_left: DVec2,
    epsg: Option<u32>,
) -> Result<(), VoxelError> {
    directory.write_tag(Tag::ModelPixelScaleTag, &[resolution, resolution, 0.0][..])?;
    directory.write_tag(
        Tag::ModelTiepointTag,
        &[0.0, 0.0, 0.0, top_left.x, top_left.y, 0.0][..],
    )?;
    directory.write_tag(Tag::GeoKeyDirectoryTag, &geo_key_directory(epsg)?[..])?;
    directory.write_tag(Tag::GdalNodata, "255")?;
    Ok(())
}

/// GeoKeyDirectory for a projected, PixelIsArea raster, with the EPSG code if known.
///
/// Fails if the EPSG code does not fit the 16-bit key value.
fn geo_key_directory(epsg: Option<u32>) -> Result<Vec<u16>, VoxelError> {
    let mut keys = vec![
        (GEO_KEY_MODEL_TYPE, MODEL_TYPE_PROJECTED),
        (GEO_KEY_RASTER_TYPE, RASTER_PIXEL_IS_AREA),
    ];
    if let Some(code) = epsg {
        let code = u16::try_from(code).map_err(|_| {
            VoxelError::Unsupported(format!("EPSG code {code} does not fit a GeoTIFF key"))
        })?;
        keys.push((GEO_KEY_PROJECTED_CS, code));
    }
    let mut directory = vec![1, 1, 0, keys.len() as u16];
    for (key, value) in keys {
        directory.extend([key, 0, 1, value]);
    }
    Ok(directory)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overview_factor_fits_both_axes_and_prefers_powers_of_two() {
        let size = UVec2::new(1000, 789);
        assert_eq!(overview_factor(size, UVec2::new(500, 395)), Some(2));
        assert_eq!(overview_factor(size, UVec2::new(334, 263)), Some(3));
        assert_eq!(overview_factor(size, UVec2::new(16, 13)), Some(64));
        assert_eq!(overview_factor(size, UVec2::new(1, 1)), Some(1024));
        assert_eq!(overview_factor(size, UVec2::new(16, 20)), None);
        assert_eq!(overview_factor(size, size), None);
        // 10 to 14 all reduce 30 to 3, and none is a power of two.
        assert_eq!(overview_factor(UVec2::splat(30), UVec2::splat(3)), None);
    }

    #[test]
    fn metadata_item_reads_gdal_overview_metadata() {
        let gdal = "<GDALMetadata>\n  <Item name=\"RESAMPLING\" sample=\"0\">AVERAGE</Item>\n</GDALMetadata>";
        assert_eq!(metadata_item(gdal, "RESAMPLING"), Some("AVERAGE"));
        assert_eq!(
            metadata_item(MAX_POOLED_OVERVIEW_METADATA, "RESAMPLING"),
            Some(MAX_RESAMPLING)
        );
        let other = r#"<GDALMetadata><Item name="OVERVIEW_RESAMPLING">MAX</Item></GDALMetadata>"#;
        assert_eq!(metadata_item(other, "RESAMPLING"), None);
        assert_eq!(
            metadata_item("<Item name=\"RESAMPLING\">", "RESAMPLING"),
            None
        );
    }
}
