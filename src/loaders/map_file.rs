//! A map file opened by its extension: a ROS map or a cost GeoTIFF.

use std::path::Path;

#[cfg(feature = "geotiff")]
use crate::GeoTiffWindows;
use crate::{
    CostWindows, Costmap, RosMapLoader, costmap_from_occupancy,
    types::{CellRegion, CostmapError, Lattice},
};

/// A distance beyond a window, in cells or in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Margin {
    Cells(u32),
    /// Rounded up to whole cells at the raster's resolution.
    Metres(f64),
}

impl Margin {
    /// This margin in cells of `resolution` metres.
    pub fn cells(self, resolution: f64) -> u32 {
        match self {
            Self::Cells(cells) => cells,
            Self::Metres(metres) => (metres / resolution).ceil() as u32,
        }
    }
}

/// How a cost GeoTIFF is read: its chunk edge and the margins of
/// `GeoTiffWindows::with_margins`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoTiffReading {
    /// Edge of a held chunk, in cells.
    pub chunk_cells: u32,
    /// Distance loaded beyond each window.
    pub load_margin: Margin,
    /// Distance beyond the loaded region that chunks are kept.
    pub keep_margin: Margin,
}

#[cfg(feature = "geotiff")]
impl GeoTiffReading {
    /// Open the GeoTIFF at `path` with these chunks and margins. No cells are
    /// read yet.
    pub fn open(&self, path: impl AsRef<Path>) -> Result<GeoTiffWindows, CostmapError> {
        let windows = GeoTiffWindows::open(path, self.chunk_cells)?;
        let resolution = windows.file().resolution();
        Ok(windows.with_margins(
            self.load_margin.cells(resolution),
            self.keep_margin.cells(resolution),
        ))
    }
}

/// A map file, chosen by extension.
#[cfg_attr(feature = "geotiff", allow(clippy::large_enum_variant))]
pub enum MapFile {
    /// A ROS map YAML and its image, held whole in memory as costs.
    Ros(Costmap),
    /// A cost GeoTIFF, of which only the chunks near the last window are held.
    #[cfg(feature = "geotiff")]
    GeoTiff(GeoTiffWindows),
}

impl MapFile {
    /// Extensions [`open`](Self::open) accepts.
    #[cfg(feature = "geotiff")]
    pub const EXTENSIONS: &str = ".yaml, .yml (ROS map), .tif, .tiff (cost GeoTIFF)";
    /// Extensions [`open`](Self::open) accepts.
    #[cfg(not(feature = "geotiff"))]
    pub const EXTENSIONS: &str = ".yaml, .yml (ROS map); .tif, .tiff need the geotiff feature";

    /// Open the map at `path`.
    ///
    /// `.yaml` and `.yml` are a ROS map, loaded whole and converted to costs.
    /// `.tif` and `.tiff` are a cost GeoTIFF, opened as `geotiff` says, and
    /// need the `geotiff` feature. Case is ignored. Any other extension is
    /// [`CostmapError::InvalidArgument`].
    #[cfg_attr(not(feature = "geotiff"), allow(unused_variables))]
    pub fn open(path: impl AsRef<Path>, geotiff: GeoTiffReading) -> Result<Self, CostmapError> {
        let path = path.as_ref();
        let extension = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase);
        match extension.as_deref() {
            Some("yaml" | "yml") => {
                let occupancy = RosMapLoader::load_from_yaml(path)?;
                Ok(Self::Ros(costmap_from_occupancy(&occupancy)?))
            }
            #[cfg(feature = "geotiff")]
            Some("tif" | "tiff") => Ok(Self::GeoTiff(geotiff.open(path)?)),
            _ => Err(CostmapError::InvalidArgument(format!(
                "{}: unsupported extension; accepted: {}",
                path.display(),
                Self::EXTENSIONS
            ))),
        }
    }

    /// The map's cells.
    pub fn cells(&self) -> CellRegion {
        match self {
            Self::Ros(costs) => costs.cells(),
            #[cfg(feature = "geotiff")]
            Self::GeoTiff(windows) => windows.cells(),
        }
    }
}

impl CostWindows for MapFile {
    fn lattice(&self) -> Lattice {
        match self {
            Self::Ros(costs) => costs.lattice(),
            #[cfg(feature = "geotiff")]
            Self::GeoTiff(windows) => windows.lattice(),
        }
    }

    fn window(&mut self, cells: CellRegion) -> Result<Costmap, CostmapError> {
        match self {
            Self::Ros(costs) => (&*costs).window(cells),
            #[cfg(feature = "geotiff")]
            Self::GeoTiff(windows) => windows.window(cells),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const READING: GeoTiffReading = GeoTiffReading {
        chunk_cells: 64,
        load_margin: Margin::Cells(0),
        keep_margin: Margin::Metres(1.0),
    };

    #[test]
    fn an_unknown_extension_is_an_error_naming_those_accepted() {
        #[cfg(not(feature = "geotiff"))]
        let paths = ["site.png", "site", "site.tif"];
        #[cfg(feature = "geotiff")]
        let paths = ["site.png", "site"];
        for path in paths {
            let Err(CostmapError::InvalidArgument(message)) = MapFile::open(path, READING) else {
                panic!("{path} opened");
            };
            assert!(message.contains(MapFile::EXTENSIONS), "{message}");
        }
    }

    #[test]
    fn a_margin_in_metres_rounds_up_to_whole_cells() {
        assert_eq!(Margin::Metres(5.0).cells(0.3), 17);
        assert_eq!(Margin::Metres(5.0).cells(0.5), 10);
        assert_eq!(Margin::Cells(7).cells(0.5), 7);
    }
}
