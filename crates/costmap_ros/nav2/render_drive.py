"""Render the recording of demo_drive.sh as a video.

Usage: python render_drive.py --map cost.tif --recording recording.npz --out drive.mp4

Left: the whole raster, dimmed, read from the file. Drawn over it in colour
is Nav2's global costmap as Nav2 published it, the outline of the window
costmap_window_server last published, Nav2's plan, the waypoints and the
track driven. Right: Nav2's local costmap around the robot.

Needs ffmpeg on PATH.
"""

import argparse
import math
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from check_costmap import load_geotiff

WIDTH, HEIGHT = 1280, 720
PANEL = 440
OCC_UNKNOWN = 255
BACKGROUND = (24, 26, 30)
TEXT = (228, 230, 235)
MUTED = (150, 156, 166)
WINDOW_OUTLINE = (255, 196, 0)
GLOBAL_OUTLINE = (90, 200, 255)
PLAN = (40, 110, 255)
TRACK = (20, 160, 70)
ROBOT = (20, 20, 20)
WAYPOINT = (140, 60, 200)


def occupancy_palette():
    """RGB for each occupancy byte Nav2's costmap publisher sends."""
    palette = np.zeros((256, 3), np.uint8)
    palette[:] = (176, 30, 30)
    ramp = np.linspace(0.0, 1.0, 99)[:, None]
    palette[0:99] = (1 - ramp) * np.array([250, 250, 246]) + ramp * np.array([244, 150, 60])
    palette[99] = (226, 96, 40)
    palette[100] = (176, 30, 30)
    palette[OCC_UNKNOWN] = (104, 116, 134)
    return palette


def cost_palette(dim):
    """RGB for each raster cost, blended towards the background by `dim`."""
    palette = np.zeros((256, 3), np.float64)
    ramp = np.linspace(0.0, 1.0, 253)[:, None]
    palette[0:253] = (1 - ramp) * np.array([250, 250, 246]) + ramp * np.array([244, 150, 60])
    palette[253] = (226, 96, 40)
    palette[254] = (176, 30, 30)
    palette[255] = (104, 116, 134)
    return ((1 - dim) * palette + dim * np.array(BACKGROUND)).astype(np.uint8)


def latest(times, t):
    """Index of the last entry at or before `t`, or None."""
    i = int(np.searchsorted(times, t, side="right")) - 1
    return i if i >= 0 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True)
    parser.add_argument("--recording", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--speed", type=float, default=4.0, help="playback speed-up")
    args = parser.parse_args()

    rec = np.load(args.recording)
    poses, windows, waypoints = rec["poses"], rec["windows"], rec["waypoints"]
    goal_events = rec["goal_events"]
    plan_ends = np.cumsum(rec["plan_len"])
    raster = load_geotiff(args.map)
    res = raster["resolution"]
    cost = raster["expected"]
    map_h_m, map_w_m = cost.shape[0] * res, cost.shape[1] * res

    # Left panel: the whole map, y up, scaled to the frame height.
    scale = HEIGHT / map_h_m
    left_w = int(round(map_w_m * scale))
    step = max(int(1.0 / (scale * res)), 1)
    h, w = (cost.shape[0] // step) * step, (cost.shape[1] // step) * step
    blocks = cost[:h, :w].reshape(h // step, step, w // step, step)
    coarse = np.where(blocks == 255, 0, blocks).max(axis=(1, 3))
    coarse[(blocks == 255).all(axis=(1, 3))] = 255
    base = Image.fromarray(cost_palette(0.72)[np.flipud(coarse)]).resize(
        (left_w, HEIGHT), Image.BILINEAR
    )

    def to_px(x, y):
        return x * scale, HEIGHT - y * scale

    occ_colours = occupancy_palette()
    font = ImageFont.load_default(size=18)
    small = ImageFont.load_default(size=14)
    panel_x = left_w + (WIDTH - left_w - PANEL) // 2
    panel_y = 40

    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{WIDTH}x{HEIGHT}", "-r", str(args.fps), "-i", "-",
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", args.out,
        ],
        stdin=subprocess.PIPE,
    )

    travelled = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(poses[:, 1]), np.diff(poses[:, 2])))])
    reached = goal_events[goal_events[:, 2] == 4]
    t_end = poses[-1, 0]
    frame_count = int(t_end * args.fps / args.speed) + args.fps
    for k in range(frame_count):
        t = min(k * args.speed / args.fps, t_end)
        pi = latest(poses[:, 0], t) or 0
        _, x, y, yaw = poses[pi]

        frame = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
        frame.paste(base, (0, 0))
        draw = ImageDraw.Draw(frame)

        gi = latest(rec["global_meta"][:, 0], t)
        if gi is not None:
            _, ox, oy, gres = rec["global_meta"][gi]
            grid = rec["global_grid"][gi]
            x0, y1 = to_px(ox, oy)
            x1, y0 = to_px(ox + grid.shape[1] * gres, oy + grid.shape[0] * gres)
            size = (int(round(x1 - x0)), int(round(y1 - y0)))
            image = Image.fromarray(occ_colours[np.flipud(grid)]).resize(size, Image.BOX)
            frame.paste(image, (int(round(x0)), int(round(y0))))
            draw.rectangle([x0, y0, x1, y1], outline=GLOBAL_OUTLINE, width=2)

        wi = latest(windows[:, 0], t)
        if wi is not None:
            _, ox, oy, ww, wh = windows[wi]
            x0, y1 = to_px(ox, oy)
            x1, y0 = to_px(ox + ww, oy + wh)
            draw.rectangle([x0, y0, x1, y1], outline=WINDOW_OUTLINE, width=2)

        plan = None
        qi = latest(rec["plan_t"], t)
        if qi is not None:
            plan = rec["plan_xy"][plan_ends[qi] - rec["plan_len"][qi] : plan_ends[qi]]
            draw.line([to_px(px, py) for px, py in plan[::5]], fill=PLAN, width=2)
        draw.line([to_px(px, py) for _, px, py, _ in poses[: pi + 1 : 3]] or [to_px(x, y)] * 2,
                  fill=TRACK, width=3)
        done = int((reached[:, 0] <= t).sum())
        for index, (wx, wy) in enumerate(waypoints):
            cx, cy = to_px(wx, wy)
            fill = WAYPOINT if index >= done else TRACK
            draw.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=fill, outline=(255, 255, 255))
        cx, cy = to_px(x, y)
        draw.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=ROBOT, outline=(255, 255, 255), width=2)

        li = latest(rec["local_meta"][:, 0], t)
        if li is not None:
            _, ox, oy, lres = rec["local_meta"][li]
            grid = rec["local_grid"][li]
            zoom = PANEL / (grid.shape[1] * lres)
            image = Image.fromarray(occ_colours[np.flipud(grid)]).resize((PANEL, PANEL), Image.NEAREST)
            frame.paste(image, (panel_x, panel_y))

            def local_px(wx, wy):
                return panel_x + (wx - ox) * zoom, panel_y + PANEL - (wy - oy) * zoom

            if plan is not None:
                near = plan[(np.abs(plan[:, 0] - x) < 3.0) & (np.abs(plan[:, 1] - y) < 3.0)]
                inside = [local_px(px, py) for px, py in near
                          if ox <= px <= ox + PANEL / zoom and oy <= py <= oy + PANEL / zoom]
                if len(inside) > 1:
                    draw.line(inside, fill=PLAN, width=3)
            rx, ry = local_px(x, y)
            radius = 0.3 * zoom
            draw.ellipse([rx - radius, ry - radius, rx + radius, ry + radius], outline=ROBOT, width=3)
            draw.line([rx, ry, rx + radius * math.cos(yaw), ry - radius * math.sin(yaw)], fill=ROBOT, width=3)
            draw.rectangle([panel_x, panel_y, panel_x + PANEL, panel_y + PANEL], outline=MUTED)
        draw.text((panel_x, panel_y - 26), "Nav2 local costmap, 6 m", font=font, fill=TEXT)

        text_y = panel_y + PANEL + 18
        lines = [
            (f"t {t:5.0f} s    driven {travelled[pi]:5.1f} m    waypoint {min(done + 1, len(waypoints))}/{len(waypoints)}", TEXT),
            (f"map {map_w_m:.0f} x {map_h_m:.0f} m, {cost.size / 1e6:.0f} M cells at {res:.2f} m", MUTED),
            (f"windows published: {0 if wi is None else wi + 1}", WINDOW_OUTLINE),
            ("yellow: window last published by costmap_window_server", WINDOW_OUTLINE),
            ("blue: Nav2 global costmap, as Nav2 published it", GLOBAL_OUTLINE),
            ("dimmed: the file outside what Nav2 holds now", MUTED),
        ]
        for line, colour in lines:
            draw.text((panel_x, text_y), line, font=small if colour != TEXT else font, fill=colour)
            text_y += 26
        ffmpeg.stdin.write(frame.tobytes())

    ffmpeg.stdin.close()
    if ffmpeg.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"wrote {args.out}: {frame_count} frames, {frame_count / args.fps:.0f} s at {args.speed:g}x")


if __name__ == "__main__":
    main()
