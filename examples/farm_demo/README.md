# Farm demo

This example drives a simulated robot across a cost GeoTIFF without ever holding the whole map:

- The route is planned on a 0.4 m overview of the whole map. The overview is read from one of the file's overview images if the file records it as max-pooled (`RESAMPLING=MAX`, which `write_cost_geotiff` writes). Otherwise it is max-pooled while streaming the raster, and the program prints why. `gdaladdo` and the COG driver write averaged or unrecorded overviews, which can drop fences. For maps with cells coarser than 0.25 m the overview is the full map.
- Tile chunks are loaded from the file near the robot and evicted beyond a larger radius. The full-resolution raster is held only as these chunks.
- A 30 m rolling costmap combines the paged map, simulated lidar and inflation. Lidar beams stop at the map's no-data cells, so unknown ground stays unknown and the robot does not enter it.
- Two obstacles that the map does not contain are placed on the route, and the local planner goes round them. If the robot can reach no point of the route, it marks the obstacles it has seen on the overview and replans the route. It stops, blocked, with the reason, if that fails.
- Safety is measured against ground truth: the robot's 0.4 m footprint, every half cell along its path, against the map's lethal cells and the surprise obstacles. The summary reports the steps with an overlap and the minimum clearance.
- The program prints a summary and exits non-zero unless the goal is reached with no safety violations.

Commands run from the repository root. Run it on a generated 400 m × 300 m farm. No data is needed: the farm is written to a temporary GeoTIFF and read back through the same path as a real map.

```bash
cargo run --release --example farm_demo --features rerun,geotiff
```

Run it on your own cost GeoTIFF. Start and goal are in metres east and north of the map's lower-left corner; without them the demo picks two free cells far apart.

```bash
cargo run --release --example farm_demo --features rerun,geotiff -- \
    --map cost.tif --start 300,80 --goal 20,170
```

Both commands open the [Rerun](https://rerun.io) viewer, which must be installed (`cargo install rerun-cli`). Other outputs:

| Flag | Effect |
| --- | --- |
| `--save out.rrd` | Write a Rerun recording instead of opening the viewer. |
| `--headless` | Log nothing; print the summary only. |
| `--frames DIR` | Also write PNG frames, drawn by the demo itself. |
| `--speed MPS` | Robot speed in m/s, 0.1 to 10. The default is 1.5. |
| `--overview-m M` | Largest cell size of the route-planning overview in metres, 0.05 to 5. The default is 0.5. The route can only pass gaps a few overview cells wide. |

Each flag may be given once.

Record a video without a display:

```bash
cargo run --release --example farm_demo --features rerun,geotiff -- --headless --frames frames
ffmpeg -framerate 30 -i frames/frame_%05d.png -pix_fmt yuv420p farm_demo.mp4
```

## Row crops

The process was run end to end on a vineyard survey ([UAV AgroTwin Vineyard Dataset 2024](https://zenodo.org/records/12168516), 322 nadir images, rows 2.3 m apart). Two settings differ from the defaults:

- ODM did not reconstruct the vine canopy, so the height rule marked almost none of the rows. The colour rule marks them from the orthophoto. See "Thin crops" in [`crates/costmap-geo`](../../crates/costmap-geo) for the measurements and the limits of that rule.
- At the default 0.5 m overview the lanes between rows close up and no route enters them. `--overview-m 0.12` keeps them open.

```bash
cargo run --release -p costmap-geo --bin costmap-from-ortho -- \
    --dsm odm_dem/dsm.tif --dtm odm_dem/dtm.tif --ortho odm_orthophoto/odm_orthophoto.tif \
    --vegetation-cost 254 --exg-threshold 0.15 --vegetation-window-m 0.21 --out cost.tif

cargo run --release --example farm_demo --features rerun,geotiff -- \
    --map cost.tif --overview-m 0.12 --start 53.5,30 --goal 100.1,80
```
