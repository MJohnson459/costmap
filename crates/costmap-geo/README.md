# costmap-geo

`costmap-from-ortho` turns the rasters OpenDroneMap (ODM) produces from a drone survey into a single-band `u8` cost GeoTIFF. The `costmap` crate's tiled store loads the file, and QGIS and GDAL open it directly.

## Process

1. **Fly and process with ODM.** Enable the surface and terrain models:

   ```
   odm --project-path /data project --dsm --dtm
   ```

   ODM writes three files, which this tool reads:

   - `odm_dem/dsm.tif`: surface height, including trees and buildings.
   - `odm_dem/dtm.tif`: bare-ground height.
   - `odm_orthophoto/odm_orthophoto.tif`: RGBA image. Alpha 0 marks pixels without data.

2. **Build the cost map.**

   ```
   costmap-from-ortho \
     --dsm /data/odm_dem/dsm.tif \
     --dtm /data/odm_dem/dtm.tif \
     --ortho /data/odm_orthophoto/odm_orthophoto.tif \
     --out cost.tif
   ```

   The command prints the output size, resolution, EPSG code, the share of cells per class (free, graded, lethal or inscribed (253-254), unknown) and the elapsed time. Errors go to stderr with a non-zero exit status.

3. **Inspect in QGIS.** Load `cost.tif` and the orthophoto. To mark areas by hand, create a polygon layer in the raster's CRS, add an integer field named `cost`, and draw. Export it as GeoJSON without changing the CRS (Export, Save Features As, format GeoJSON, CRS the same as the raster). Typical values: 254 for a keep-out zone, 0 to force a track open.

4. **Re-run with the overrides.**

   ```
   costmap-from-ortho --dsm ... --dtm ... --ortho ... --overrides zones.geojson --out cost.tif
   ```

5. **Optionally convert to a Cloud Optimized GeoTIFF.**

   ```
   gdal_translate -of COG -co COMPRESS=DEFLATE cost.tif cost_cog.tif
   ```

   Use `COMPRESS=DEFLATE`. GDAL's default LZW COG fails to load in `costmap::CostGeoTiff` on the Aukerman dataset ("no lzw end code found" from the `tiff` crate); the DEFLATE file loads.

   `cost.tif` records `RESAMPLING=MAX` on each of its overviews, and `CostGeoTiff::overview_is_max_pooled` reports them max-pooled. The COG driver of GDAL 3.11 writes no such record, whether it resamples the overviews or copies them with `-co OVERVIEWS=FORCE_USE_EXISTING`, so the converted file's overviews are not reported max-pooled and the farm demo max-pools the raster while streaming it instead.

## Cost values

| Cost | Meaning |
| --- | --- |
| 0 | Free |
| 1 to 252 | Graded: slope or vegetation cost, higher is worse |
| 253 | Inscribed (not produced by this tool; an override may set it) |
| 254 | Lethal: obstacles, steep slopes, and vegetation at `--vegetation-cost 254` |
| 255 | Unknown; declared as the GeoTIFF nodata value |

## Rules

The output grid is north-up, covers the DTM's extent and starts at the DTM's lower-left corner. Each cell is evaluated at its centre. The three rasters need not share a pixel grid: each is sampled by projected coordinate, taking the pixel that contains the point.

Rules apply in this order. A cell's cost is set by the first rule that matches, except that rules 4 and 5 modify a cost.

| # | Rule | Option | Default |
| --- | --- | --- | --- |
| 1 | Unknown (255) if the DSM or DTM has no value at the cell, or the orthophoto has alpha 0 or does not cover it. | | |
| 2 | Lethal (254) if DSM minus DTM is at least the obstacle height. | `--obstacle-height` | 0.3 m |
| 3 | Otherwise slope cost: 0 up to the free angle, linear to 252 at the lethal angle, 254 at or above it. | `--slope-free-deg`, `--slope-lethal-deg` | 10, 25 degrees |
| 4 | Vegetation: a cell is vegetation when at least `--vegetation-min-fraction` of the known orthophoto pixels in the box around it have an excess-green index `(2G - R - B) / (R + G + B)` above the threshold (see Vegetation). Its cost becomes `max(cost, vegetation_cost)`, 0 to 254. Needs `--ortho`. Never lowers a cost, never changes unknown cells. | `--vegetation-cost`, `--exg-threshold`, `--vegetation-window-m`, `--vegetation-min-fraction` | 0 (off), 0.1, 0.15 m, 0.5 |
| 5 | Overrides: every cell whose centre is inside a polygon takes the polygon's cost. Features apply in file order, so later ones win. | `--overrides` | none |

Other options:

| Option | Meaning | Default |
| --- | --- | --- |
| `--resolution` | Output cell size in metres. | The DTM's pixel size |
| `--slope-window-m` | Largest edge of the box the DTM is averaged over before the slope is taken (see Slope). | 1.0 m |

Rule 5 runs last, so an override sets the cell even where the rasters say lethal or unknown.

### Slope

The DTM is averaged over a square box, which keeps centimetre noise in the DTM from reading as steep ground. The box is the largest odd number of cells that fits in `--slope-window-m`, and at least 3 cells: at 0.05 m cells a 1 m window gives a 19-cell, 0.95 m box; at 0.6 m cells it gives a 3-cell, 1.8 m box. The slope at a cell is the central difference of two box means, one box radius (`(cells - 1) / 2` cells) either side of the cell, in each of x and y, so the baseline is the box edge minus one cell. The tool prints the box and baseline it used. Cells without DTM data are left out of the mean. If fewer than half of the box's cells have data, that axis contributes no gradient and the cell reads as flat.

### Vegetation

The box is the largest odd number of cells that fits in `--vegetation-window-m`, and at least 1 cell, so a window under 2 cells is the per-pixel test. At 0.03 m cells the default 0.15 m window gives a 5-cell, 0.15 m box; at 0.1 m cells it gives a 1-cell box. The tool prints the box it used. Pixels with alpha 0 or outside the orthophoto are left out of both the green count and the pixel count; a cell with no known pixels in its box is not vegetation. A fraction of exactly `--vegetation-min-fraction` counts.

Valid values for `--vegetation-cost` are 0 to 254. 254 is lethal; 255 (unknown) is rejected. `--vegetation-min-fraction` must be above 0 and at most 1; `--vegetation-window-m` must not be negative.

### Thin crops

ODM's default dense reconstruction can miss canopies a few tens of centimetres wide, so the height rule does not see them. In a vineyard survey at 0.03 m (vines 1.5 m tall, canopies about 0.3 m wide, rows 2.3 m apart), 0.6 % of the point-cloud points in a sample were 0.3 m or more above the terrain model, and the cost map showed the vineyard as free ground. Check by comparing the cost map with the orthophoto.

ODM settings did not change this for that survey (322 nadir images at 30 m, DJI Phantom 3 Pro, ODM 3.6.2). The share of vine-coloured pixels standing 0.3 m or more above the terrain model was 2 % at the defaults, 1.6 % with `--pc-quality high` and 1.4 % with `--pc-quality high --pc-filter 0`. Oblique images were not tried.

The colour rule can mark such rows as obstacles:

```
costmap-from-ortho --dsm ... --dtm ... --ortho ... --vegetation-cost 254 --exg-threshold 0.15 --out cost.tif
```

In that survey the excess-green index was 0.3 to 0.5 on canopy and below about 0.1 on the dry ground between rows, and `--exg-threshold 0.15` selected 24.8 % of the pixels. That selection draws the rows as continuous bands plus single-pixel specks in the lanes; the neighbourhood test removes the single-pixel specks.

Limits:

- It works only where the crop is green and the ground between rows is not. It fails with green cover crops or grass lanes.
- It marks every green thing, including weeds and field margins.
- Canopy gaps and trellis wires are not detected. Where the canopy is sparse, a row can break into separate segments.
- The map is a prior. The robot's own sensors must confirm the lane.

### Overrides file

A GeoJSON FeatureCollection of `Polygon` and `MultiPolygon` features in the same projected CRS as the rasters. The tool does not read the file's `crs` member and does not reproject. Each feature needs an integer property `cost` from 0 to 255:

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": { "cost": 254 },
      "geometry": {
        "type": "Polygon",
        "coordinates": [[[436900, 4572800], [436920, 4572800], [436920, 4572820], [436900, 4572800]]]
      }
    }
  ]
}
```

Inside a polygon follows the even-odd rule, so interior rings are holes. Concave polygons are filled correctly. Polygons may extend beyond the raster. A missing `cost`, a non-integer or out-of-range `cost`, a geometry other than Polygon or MultiPolygon, or a ring with fewer than 3 positions stops the run with an error naming the feature's index (counting from 0).

## Memory

The DEMs and the orthophoto are read one TIFF chunk at a time through a cache of the 32 most recent chunks per raster, and the output is processed in blocks of 1024 by 1024 cells plus the slope halo (DTM) and the vegetation halo (orthophoto). Input memory therefore depends on the chunk size, not the raster size. A file stored as a single strip is decoded whole.

The output raster is assembled in memory before it is written, at one byte per cell plus overviews. The Aukerman dataset (7528 by 5939 cells) peaks at about 250 MB.

## Limitations

- An aerial map is a prior. The DTM under tree canopy is interpolated, and the DSM there is the canopy, so every cell under trees is lethal. Paths under trees are not visible.
- The survey ages. Crops grow, vehicles move, and ground floods. Re-fly or override before relying on it.
- Anything shorter than `--obstacle-height` is invisible. Rocks, ditches and kerbs below that height do not register, apart from their effect on slope.
- Slope is measured at the scale of the window. Steps and ditches narrower than the window are averaged out.
- Slope next to a gap in the DTM is biased, because the mean covers only the cells that have data.
- No reprojection. All inputs, and the overrides file, must be in the same projected CRS. Rasters without an EPSG code, with rotation, or in a geographic CRS are rejected, as are differing EPSG codes.
- Orthophoto cells outside the orthophoto's extent are unknown. The orthophoto's extent usually differs slightly from the DEMs', so a thin strip along some edges is unknown.
- `--resolution` coarser than the DTM samples the nearest DTM pixel and does not average, so features thinner than the cell may be missed or kept at random.
- The cost map's pixel size is stored through the core crate's `f32` resolution. A 0.05 m cell is written as 0.0500000007 m, which moves the far edge of the Aukerman raster by about 4 micrometres in y.
