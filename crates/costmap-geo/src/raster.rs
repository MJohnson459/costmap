//! Georeferenced TIFF rasters sampled by projected coordinate.
//!
//! Chunks (tiles or strips) are decoded when a sample needs them and the most recent
//! ones are kept, so memory depends on the chunk size and cache length, not the raster size.

use std::{fs::File, io::BufReader, path::Path};

use costmap::loaders::geotiff::{GeoFrame, explain_decode_error, read_frame, read_gdal_nodata};
use glam::{DVec2, UVec2};
use tiff::{decoder::Decoder, decoder::DecodingResult, tags::Tag};

use crate::error::GeoError;

/// Chunks each raster keeps decoded unless a wider sample row needs more (see
/// `cache_for_row_span`). Where input pixels are about the size of output cells, a
/// 1024-cell block with its slope halo spans at most 4 x 4 chunks of 512 pixels.
pub const DEFAULT_CACHE_CHUNKS: usize = 32;

/// Cache length that lets a row-by-row scan `span_pixels` wide decode each chunk
/// once: the chunks one row crosses, for the two chunk rows a scan straddles.
fn chunks_for_row_span(span_pixels: f64, chunk_width: u32) -> usize {
    let across = (span_pixels / f64::from(chunk_width)).ceil() as usize + 1;
    2 * across
}

/// `PlanarConfiguration` value for samples stored interleaved per pixel.
const PLANAR_CHUNKY: u16 = 1;
/// `SampleFormat` value for IEEE floating point samples.
const SAMPLE_FORMAT_FLOAT: u16 = 3;

/// A sample type a raster can be decoded to.
trait Sample: Copy {
    fn unpack(result: DecodingResult) -> Option<Vec<Self>>;
}

impl Sample for f32 {
    fn unpack(result: DecodingResult) -> Option<Vec<Self>> {
        match result {
            DecodingResult::F32(data) => Some(data),
            _ => None,
        }
    }
}

impl Sample for u8 {
    fn unpack(result: DecodingResult) -> Option<Vec<Self>> {
        match result {
            DecodingResult::U8(data) => Some(data),
            _ => None,
        }
    }
}

/// Pixel access to a chunked TIFF through a least-recently-used chunk cache.
struct ChunkReader<S> {
    decoder: Decoder<BufReader<File>>,
    size: UVec2,
    /// Pixels in a full chunk.
    chunk: UVec2,
    bands: usize,
    /// Decoded chunks as `(index, samples)`, most recently used first.
    cache: Vec<(u32, Vec<S>)>,
    capacity: usize,
}

impl<S: Sample> ChunkReader<S> {
    fn new(
        mut decoder: Decoder<BufReader<File>>,
        bands: usize,
        capacity: usize,
    ) -> Result<Self, GeoError> {
        let (width, height) = decoder.dimensions()?;
        let (chunk_width, chunk_height) = decoder.chunk_dimensions();
        Ok(Self {
            decoder,
            size: UVec2::new(width, height),
            chunk: UVec2::new(chunk_width.min(width), chunk_height.min(height)),
            bands,
            cache: Vec::new(),
            capacity: capacity.max(1),
        })
    }

    /// Grow the cache so that scanning rows `span_pixels` wide does not evict a
    /// chunk the next row needs.
    fn cache_for_row_span(&mut self, span_pixels: f64) {
        let span = span_pixels.min(f64::from(self.size.x));
        self.capacity = self.capacity.max(chunks_for_row_span(span, self.chunk.x));
    }

    /// The band values of the pixel at `pixel`, which must lie inside the raster.
    fn pixel(&mut self, pixel: UVec2) -> Result<&[S], GeoError> {
        let per_row = self.size.x.div_ceil(self.chunk.x);
        let index = pixel.y / self.chunk.y * per_row + pixel.x / self.chunk.x;
        let chunk_origin = pixel / self.chunk * self.chunk;
        let width = self.chunk.x.min(self.size.x - chunk_origin.x);
        let local = pixel - chunk_origin;
        let offset = (local.y as usize * width as usize + local.x as usize) * self.bands;
        let bands = self.bands;
        let samples = self.chunk_samples(index, chunk_origin)?;
        Ok(&samples[offset..offset + bands])
    }

    /// Decoded samples of chunk `index`, which starts at pixel `origin`.
    fn chunk_samples(&mut self, index: u32, origin: UVec2) -> Result<&[S], GeoError> {
        match self.cache.iter().position(|(i, _)| *i == index) {
            Some(0) => {}
            Some(position) => self.cache[..=position].rotate_right(1),
            None => {
                let samples = self.decode(index, origin)?;
                self.cache.insert(0, (index, samples));
                self.cache.truncate(self.capacity);
            }
        }
        Ok(&self.cache[0].1)
    }

    fn decode(&mut self, index: u32, origin: UVec2) -> Result<Vec<S>, GeoError> {
        let extent = self.chunk.min(self.size - origin);
        let expected = extent.x as usize * extent.y as usize * self.bands;
        let decoded = self
            .decoder
            .read_chunk(index)
            .map_err(|err| GeoError::Core(explain_decode_error(&mut self.decoder, index, err)))?;
        let samples = S::unpack(decoded).ok_or_else(|| {
            GeoError::Raster(format!(
                "chunk {index} did not decode to the expected sample type"
            ))
        })?;
        if samples.len() != expected {
            return Err(GeoError::Raster(format!(
                "chunk {index} holds {} samples, expected {expected}",
                samples.len()
            )));
        }
        Ok(samples)
    }
}

/// The pixel of a raster that contains `projected`, if it lies inside the raster.
fn pixel_at(frame: &GeoFrame, size: UVec2, projected: DVec2) -> Option<UVec2> {
    let offset = (projected - frame.top_left) / frame.pixel_size;
    let (column, row) = (offset.x.floor(), (-offset.y).floor());
    let inside =
        column >= 0.0 && row >= 0.0 && column < f64::from(size.x) && row < f64::from(size.y);
    inside.then(|| UVec2::new(column as u32, row as u32))
}

fn open_decoder(path: &Path) -> Result<(Decoder<BufReader<File>>, GeoFrame), GeoError> {
    let mut decoder = Decoder::new(BufReader::new(File::open(path)?))?;
    let planar = decoder.find_tag_unsigned::<u16>(Tag::PlanarConfiguration)?;
    if planar.is_some_and(|p| p != PLANAR_CHUNKY) {
        return Err(GeoError::Raster(
            "only pixel-interleaved (PlanarConfiguration 1) rasters are supported".into(),
        ));
    }
    let frame = read_frame(&mut decoder)?;
    Ok((decoder, frame))
}

/// Extent of the raster in projected coordinates as `(lower-left, upper-right)`.
fn extent(frame: &GeoFrame, size: UVec2) -> (DVec2, DVec2) {
    let span = size.as_dvec2() * frame.pixel_size;
    (
        DVec2::new(frame.top_left.x, frame.top_left.y - span.y),
        DVec2::new(frame.top_left.x + span.x, frame.top_left.y),
    )
}

/// A single-band Float32 raster such as a DSM or DTM.
pub struct DemRaster {
    reader: ChunkReader<f32>,
    frame: GeoFrame,
    nodata: Option<f64>,
}

impl DemRaster {
    /// Open a single-band Float32 GeoTIFF with the default chunk cache.
    pub fn open(path: &Path) -> Result<Self, GeoError> {
        Self::open_with_cache(path, DEFAULT_CACHE_CHUNKS)
    }

    /// Open with a cache of `cache_chunks` decoded chunks.
    pub fn open_with_cache(path: &Path, cache_chunks: usize) -> Result<Self, GeoError> {
        Self::try_open(path, cache_chunks).map_err(|e| e.at(path))
    }

    fn try_open(path: &Path, cache_chunks: usize) -> Result<Self, GeoError> {
        let (mut decoder, frame) = open_decoder(path)?;
        let color = decoder.colortype()?;
        let formats = decoder.find_tag_unsigned_vec::<u16>(Tag::SampleFormat)?;
        let float = formats
            .as_deref()
            .is_some_and(|f| f == [SAMPLE_FORMAT_FLOAT]);
        if color != tiff::ColorType::Gray(32) || !float {
            return Err(GeoError::Raster(format!(
                "expected a single-band Float32 raster, found {color:?}"
            )));
        }
        let nodata = read_gdal_nodata(&mut decoder)?;
        Ok(Self {
            reader: ChunkReader::new(decoder, 1, cache_chunks)?,
            frame,
            nodata,
        })
    }

    /// Grow the chunk cache for sampling rows `span_m` metres wide, one row at a time.
    pub fn cache_for_row_span(&mut self, span_m: f64) {
        self.reader
            .cache_for_row_span(span_m / self.frame.pixel_size.x.abs());
    }

    pub fn frame(&self) -> &GeoFrame {
        &self.frame
    }

    /// Raster size in pixels.
    pub fn size(&self) -> UVec2 {
        self.reader.size
    }

    /// Extent in projected coordinates as `(lower-left, upper-right)`.
    pub fn extent(&self) -> (DVec2, DVec2) {
        extent(&self.frame, self.reader.size)
    }

    /// Value of the pixel containing `projected`, or `None` outside the raster, at nodata
    /// or at a non-finite value.
    pub fn sample(&mut self, projected: DVec2) -> Result<Option<f32>, GeoError> {
        let Some(pixel) = pixel_at(&self.frame, self.reader.size, projected) else {
            return Ok(None);
        };
        let value = self.reader.pixel(pixel)?[0];
        let is_nodata = self.nodata.is_some_and(|n| f64::from(value) == n);
        Ok((value.is_finite() && !is_nodata).then_some(value))
    }
}

/// A 3-band RGB or 4-band RGBA 8-bit raster such as an ODM orthophoto.
pub struct OrthoRaster {
    reader: ChunkReader<u8>,
    frame: GeoFrame,
}

impl OrthoRaster {
    /// Open an 8-bit RGB or RGBA GeoTIFF with the default chunk cache.
    pub fn open(path: &Path) -> Result<Self, GeoError> {
        Self::open_with_cache(path, DEFAULT_CACHE_CHUNKS)
    }

    /// Open with a cache of `cache_chunks` decoded chunks.
    pub fn open_with_cache(path: &Path, cache_chunks: usize) -> Result<Self, GeoError> {
        Self::try_open(path, cache_chunks).map_err(|e| e.at(path))
    }

    fn try_open(path: &Path, cache_chunks: usize) -> Result<Self, GeoError> {
        let (mut decoder, frame) = open_decoder(path)?;
        let bands = match decoder.colortype()? {
            tiff::ColorType::RGB(8) => 3,
            tiff::ColorType::RGBA(8) => 4,
            other => {
                return Err(GeoError::Raster(format!(
                    "expected an 8-bit RGB or RGBA raster, found {other:?}"
                )));
            }
        };
        Ok(Self {
            reader: ChunkReader::new(decoder, bands, cache_chunks)?,
            frame,
        })
    }

    /// Grow the chunk cache for sampling rows `span_m` metres wide, one row at a time.
    pub fn cache_for_row_span(&mut self, span_m: f64) {
        self.reader
            .cache_for_row_span(span_m / self.frame.pixel_size.x.abs());
    }

    pub fn frame(&self) -> &GeoFrame {
        &self.frame
    }

    /// Colour of the pixel containing `projected`, or `None` outside the raster or where
    /// an alpha band is 0.
    pub fn sample(&mut self, projected: DVec2) -> Result<Option<[u8; 3]>, GeoError> {
        let Some(pixel) = pixel_at(&self.frame, self.reader.size, projected) else {
            return Ok(None);
        };
        let bands = self.reader.pixel(pixel)?;
        let transparent = bands.get(3) == Some(&0);
        Ok((!transparent).then(|| [bands[0], bands[1], bands[2]]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_holds_the_chunks_a_sample_row_crosses() {
        // 1024 cells of 0.5 m over 0.02 m pixels: 25,600 pixels, 50 tiles of 512.
        assert_eq!(chunks_for_row_span(25_600.0, 512), 102);
        assert!(chunks_for_row_span(1024.0, 512) <= DEFAULT_CACHE_CHUNKS);
    }
}
