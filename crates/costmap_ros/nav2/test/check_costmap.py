"""Check a live Nav2 costmap against the map costmap_window_server serves.

The process is also the robot: it publishes map -> base_link on /tf from
start-up, so Nav2 and the window server can resolve the robot pose. Once
Nav2's costmap arrives it compares every cell, by world position, with the
map read straight from its file: a ROS map YAML and image (trinary rule), or
a cost GeoTIFF (the occupancy the bridge sends for each cost, through Nav2's
StaticLayer::interpretValue). It then moves the robot to each pose (by
default POSES, every move 65 m, well past the published window; or --poses),
times how long Nav2 takes to show the new surroundings, and checks them.

Exit status 0 means every comparison passed.
"""

import argparse
import math
import pathlib
import statistics
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from cost_raster import UNKNOWN, geotiff_georef, load_map  # noqa: E402

# (x, y, yaw) in the map frame. Each step is 65 m, so the Nav2 window after a
# move never overlaps the window published before it, for windows up to 100 m.
POSES = [
    (60.0, 70.0, 0.0),
    (125.0, 100.0, 0.7),
    (190.0, 70.0, -1.2),
    (255.0, 100.0, 3.0),
    (320.0, 70.0, 0.0),
    (385.0, 100.0, 1.5),
]

TF_RATE_HZ = 20.0
MAX_REPORTED_MISMATCHES = 5


def expected_costs(fixture, costmap):
    """Expected cost at the centre of every cell of a nav2_msgs/Costmap."""
    meta = costmap.metadata
    res = meta.resolution
    xs = meta.origin.position.x + (np.arange(meta.size_x) + 0.5) * res
    ys = meta.origin.position.y + (np.arange(meta.size_y) + 0.5) * res
    wx, wy = np.meshgrid(xs, ys)
    fx = np.floor((wx - fixture["origin"][0]) / fixture["resolution"]).astype(np.int64)
    fy = np.floor((wy - fixture["origin"][1]) / fixture["resolution"]).astype(np.int64)
    rows, cols = fixture["expected"].shape
    inside = (fx >= 0) & (fx < cols) & (fy >= 0) & (fy < rows)
    expected = np.full(wx.shape, UNKNOWN, dtype=np.uint8)
    expected[inside] = fixture["expected"][fy[inside], fx[inside]]
    klass = np.full(wx.shape, fixture["classes"].index("unknown"), dtype=np.uint8)
    klass[inside] = fixture["klass"][fy[inside], fx[inside]]
    return expected, klass, wx, wy


def compare(fixture, costmap):
    """Per-class counts of cells that match, and a few mismatches."""
    meta = costmap.metadata
    actual = np.asarray(costmap.data, dtype=np.uint8).reshape(meta.size_y, meta.size_x)
    expected, klass, wx, wy = expected_costs(fixture, costmap)
    names = fixture["classes"]
    result = {"mismatches": [], "classes": names}
    for index, name in enumerate(names):
        cells = klass == index
        result[name] = (
            int(np.count_nonzero(actual[cells] == expected[cells])),
            int(np.count_nonzero(cells)),
        )
    for j, i in zip(*np.nonzero(actual != expected)):
        if len(result["mismatches"]) == MAX_REPORTED_MISMATCHES:
            break
        result["mismatches"].append(
            f"({wx[j, i]:.3f}, {wy[j, i]:.3f}) {names[klass[j, i]]} "
            f"expected {expected[j, i]} got {actual[j, i]}"
        )
    # A window missing a required class would pass vacuously.
    result["ok"] = not np.any(actual != expected) and all(
        sum(result[name][1] for name in alternatives) > 0 for alternatives in fixture["required"]
    )
    return result


def window_centre(costmap):
    meta = costmap.metadata
    return (
        meta.origin.position.x + 0.5 * meta.size_x * meta.resolution,
        meta.origin.position.y + 0.5 * meta.size_y * meta.resolution,
    )


def centred_on(costmap, pose):
    """Whether the rolling window has been moved to `pose` (within two cells)."""
    cx, cy = window_centre(costmap)
    return math.hypot(cx - pose[0], cy - pose[1]) <= 2 * costmap.metadata.resolution * math.sqrt(2)


def summary(result):
    return ", ".join(f"{k} {result[k][0]}/{result[k][1]}" for k in result["classes"])


class Robot(Node):
    """Publishes the robot pose on /tf and keeps every costmap received."""

    def __init__(self, args, poses):
        super().__init__("costmap_e2e_check")
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.pose = poses[0]
        self.received = []
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_timer(1.0 / TF_RATE_HZ, self.publish_pose)
        self.create_subscription(
            Costmap,
            args.costmap_topic,
            lambda msg: self.received.append((time.monotonic(), msg)),
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

    def publish_pose(self):
        x, y, yaw = self.pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def move_to(self, pose):
        """Moves the robot and returns the monotonic time of the move."""
        self.pose = pose
        self.received.clear()
        moved_at = time.monotonic()
        self.publish_pose()
        return moved_at


def wait_for_match(robot, fixture, pose, timeout):
    """First costmap centred on `pose` that matches the map.

    Returns (receive time, comparison, costmap), with a receive time of None
    and the last comparison made if nothing matched before the timeout.
    """
    deadline = time.monotonic() + timeout
    checked = 0
    last = None
    while time.monotonic() < deadline:
        rclpy.spin_once(robot, timeout_sec=0.05)
        while checked < len(robot.received):
            received_at, costmap = robot.received[checked]
            checked += 1
            if not centred_on(costmap, pose):
                continue
            last = compare(fixture, costmap)
            if last["ok"]:
                return received_at, last, costmap
    return None, last, None


def report_failure(label, pose, result):
    print(f"FAIL {label}: no Nav2 costmap matched the map around {pose[:2]}")
    if result is None:
        print("  no costmap centred on the pose arrived")
        return
    print(f"  last costmap: {summary(result)}")
    for line in result["mismatches"]:
        print(f"  mismatch at {line}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", required=True, help="ROS map YAML (fixture) or cost GeoTIFF (.tif)")
    ap.add_argument(
        "--poses",
        help='robot poses "x,y x,y ..." in the local map frame; default: the fixture poses',
    )
    ap.add_argument(
        "--print-resolution",
        action="store_true",
        help="print the map's resolution in metres and exit",
    )
    ap.add_argument("--window-m", type=float, help="published window size")
    ap.add_argument("--costmap-topic", default="/costmap_raw")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--first-timeout", type=float, default=60.0)
    ap.add_argument("--move-timeout", type=float, default=10.0)
    args = ap.parse_args()

    if args.print_resolution:
        if args.map.lower().endswith((".yaml", ".yml")):
            print(f"{load_map(args.map)['resolution']:.9g}")
        else:
            print(f"{geotiff_georef(args.map)[0]:.9g}")
        return 0
    if args.window_m is None:
        ap.error("--window-m is required")
    poses = POSES
    if args.poses:
        poses = [(float(x), float(y), 0.0) for x, y in (p.split(",") for p in args.poses.split())]
    fixture = load_map(args.map)
    rclpy.init()
    robot = Robot(args, poses)
    robot.publish_pose()
    print(f"tf: publishing {args.map_frame} -> {args.base_frame} at {poses[0][:2]}", flush=True)

    failures = 0
    received_at, result, costmap = wait_for_match(robot, fixture, poses[0], args.first_timeout)
    if received_at is None:
        report_failure("start", poses[0], result)
        failures += 1
    else:
        meta = costmap.metadata
        nav2_m = meta.size_x * meta.resolution
        print(f"Nav2 window {nav2_m:.2f} m, published window {args.window_m:.2f} m")
        if nav2_m >= args.window_m:
            print("FAIL: the Nav2 window must be smaller than the published window")
            failures += 1
        print(f"PASS start {poses[0][:2]}: {summary(result)}", flush=True)

    latencies = []
    for pose in poses[1:]:
        if failures:
            break
        moved_at = robot.move_to(pose)
        received_at, result, _ = wait_for_match(robot, fixture, pose, args.move_timeout)
        if received_at is None:
            report_failure("move", pose, result)
            failures += 1
            continue
        latency_ms = (received_at - moved_at) * 1000.0
        latencies.append(latency_ms)
        print(f"PASS move to {pose[:2]}: {summary(result)}, {latency_ms:.0f} ms", flush=True)

    if latencies:
        print(
            f"latency window_m={args.window_m:g} moves={len(latencies)} "
            f"min={min(latencies):.0f} ms median={statistics.median(latencies):.0f} ms "
            f"max={max(latencies):.0f} ms"
        )
    verdict = "PASS" if failures == 0 and len(latencies) == len(poses) - 1 else "FAIL"
    print(f"{verdict}: window_m={args.window_m:g}, {len(latencies)}/{len(poses) - 1} moves matched")
    robot.destroy_node()
    rclpy.shutdown()
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
