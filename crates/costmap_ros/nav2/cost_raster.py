"""Cost GeoTIFFs and ROS maps read with Pillow, independently of the Rust
readers.

Shared by the integration test, the drive demo and the benchmark.
"""

import pathlib

import numpy as np
import yaml
from PIL import Image

FREE, LETHAL, UNKNOWN = 0, 254, 255

TIFF_PIXEL_SCALE = 33550
TIFF_TIEPOINT = 33922


def geotiff_georef(tif_path):
    """(resolution, (rows, cols)) of a cost GeoTIFF, from its TIFF tags only.

    The map frame is local metres from the raster's lower-left corner, so the
    tiepoint is used only to confirm that raster pixel (0, 0) is the anchor.
    """
    img = Image.open(tif_path)
    tags = img.tag_v2
    if TIFF_PIXEL_SCALE not in tags or TIFF_TIEPOINT not in tags:
        raise SystemExit(f"{tif_path}: no ModelPixelScale/ModelTiepoint tags")
    scale = tags[TIFF_PIXEL_SCALE]
    tie = tags[TIFF_TIEPOINT]
    if abs(scale[0] - scale[1]) > 1e-12 * scale[0]:
        raise SystemExit(f"{tif_path}: non-square pixels {scale[:2]}")
    if tuple(tie[:2]) != (0.0, 0.0):
        raise SystemExit(f"{tif_path}: tiepoint raster position {tie[:2]} is not pixel (0, 0)")
    return float(scale[0]), (img.height, img.width)


def occupancy_from_cost(cost):
    """Occupancy value the bridge is documented to send for each cost.

    README "Cost and occupancy": 255 -> -1, 254 -> 100, 253 -> 99, 0 -> 0,
    1..=252 -> ceil(cost * 100 / 254) capped at 99.
    """
    cost = cost.astype(np.int64)
    occ = -((-cost * 100) // 254)
    occ = np.minimum(occ, 99)
    occ[cost == 0] = 0
    occ[cost == 253] = 99
    occ[cost == 254] = 100
    occ[cost == 255] = -1
    return occ


def interpret_value(occ):
    """Nav2 StaticLayer::interpretValue for trinary_costmap false,
    track_unknown_space true, lethal threshold 100, unknown 255, inscribed 99.

    The occupancy byte is read unsigned, so -1 is 255.
    """
    value = (occ & 0xFF).astype(np.int64)
    out = (value / 100.0 * 254.0).astype(np.int64)
    out[value >= 100] = 254
    out[value == 99] = 253
    out[value == 255] = 255
    return out.astype(np.uint8)


def load_geotiff(tif_path):
    """Expected Nav2 cost of every raster cell, row 0 at the bottom.

    Reads the raster with Pillow and its pixel size from the TIFF tags,
    independently of the Rust GeoTIFF reader. The map frame origin is the
    raster's lower-left corner, x east, y north.
    """
    resolution, _ = geotiff_georef(tif_path)
    img = Image.open(tif_path)
    if img.mode != "L":
        raise SystemExit(f"{tif_path}: expected an 8-bit single-band raster, got mode {img.mode}")
    cost = np.asarray(img, dtype=np.uint8)
    klass = np.full(cost.shape, 1, dtype=np.uint8)
    klass[cost == 0] = 0
    klass[(cost == 253) | (cost == 254)] = 2
    klass[cost == 255] = 3
    return {
        "expected": np.flipud(interpret_value(occupancy_from_cost(cost))),
        "klass": np.flipud(klass),
        "classes": ["free", "graded", "inscribed/lethal", "unknown"],
        "required": [["free"], ["graded", "inscribed/lethal"]],
        "resolution": resolution,
        "origin": (0.0, 0.0),
    }


def load_ros_map(yaml_path):
    """Expected Nav2 cost of every cell of a ROS map, row 0 at the bottom.

    Follows nav2_map_server's trinary rule with the YAML's thresholds:
    occupancy = (255 - grey) / 255 for an un-negated image, lethal above
    occupied_thresh, free below free_thresh, unknown otherwise.
    """
    meta = yaml.safe_load(pathlib.Path(yaml_path).read_text())
    if meta.get("mode", "trinary") != "trinary":
        raise SystemExit(f"{yaml_path}: only trinary maps are supported by this check")
    image_path = pathlib.Path(yaml_path).parent / meta["image"]
    grey = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64)
    occupancy = grey / 255.0 if meta.get("negate", 0) else (255.0 - grey) / 255.0
    costs = np.full(grey.shape, UNKNOWN, dtype=np.uint8)
    costs[occupancy > meta.get("occupied_thresh", 0.65)] = LETHAL
    costs[occupancy < meta.get("free_thresh", 0.196)] = FREE
    origin = meta["origin"]
    if origin[2] != 0:
        raise SystemExit(f"{yaml_path}: rotated map origins are not supported")
    classes = {FREE: 1, LETHAL: 0, UNKNOWN: 2}
    return {
        "expected": np.flipud(costs),
        "klass": np.flipud(np.vectorize(classes.get, otypes=[np.uint8])(costs)),
        "classes": ["lethal", "free", "unknown"],
        "required": [["lethal"], ["free"]],
        "resolution": float(meta["resolution"]),
        "origin": (float(origin[0]), float(origin[1])),
    }


def load_map(path):
    """`load_ros_map` for a .yaml or .yml file, `load_geotiff` for any other."""
    if pathlib.Path(path).suffix.lower() in (".yaml", ".yml"):
        return load_ros_map(path)
    return load_geotiff(path)
