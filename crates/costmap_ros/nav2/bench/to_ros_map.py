"""Writes a cost GeoTIFF as a ROS map (PGM and YAML) for nav2_map_server.

Lethal and inscribed costs become occupied, unknown stays unknown and every
other cost becomes free, which is what a trinary ROS map can hold.
"""

import argparse
import pathlib
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from cost_raster import geotiff_georef  # noqa: E402

Image.MAX_IMAGE_PIXELS = None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True, help="cost GeoTIFF")
    parser.add_argument("--out", required=True, help="directory for map.pgm and map.yaml")
    args = parser.parse_args()

    resolution, _ = geotiff_georef(args.map)
    cost = np.asarray(Image.open(args.map), dtype=np.uint8)
    pixels = np.full(cost.shape, 254, dtype=np.uint8)
    pixels[cost >= 253] = 0
    pixels[cost == 255] = 205

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(out / "map.pgm")
    (out / "map.yaml").write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {resolution!r}\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    print(out / "map.yaml")


if __name__ == "__main__":
    main()
