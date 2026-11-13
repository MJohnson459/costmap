"""Drive a robot across a cost GeoTIFF with Nav2 and record what Nav2 saw.

Sends each waypoint to Nav2 as a NavigateToPose goal and waits for Nav2 to
report success with the robot within 0.5 m of it. Meanwhile it records the
robot pose, every plan, Nav2's local costmap (and its global costmap, if it
has one) and the extent of every window tiled_map_server publishes, and
writes them to <out>/recording.npz for render.py.

Afterwards it checks the driven track against the raster read straight from
its file: no cell under the robot's disc may be lethal.

Exit status 0 means every waypoint was reached and the track is clear.
"""

import argparse
import math
import pathlib
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from cost_raster import load_map  # noqa: E402

LETHAL = 254
POSE_RATE_HZ = 10.0
# Nav2's global costmap is stored at one cell per this many cells, by maximum.
GLOBAL_DOWNSAMPLE = 4
# Occupancy values Nav2's costmap publisher uses, as stored bytes.
OCC_UNKNOWN = 255
SERVER_TIMEOUT_S = 120.0
# Nav2 reporting success is not enough: the robot must be this close to the waypoint.
ARRIVAL_TOLERANCE_M = 0.5
# Distance around the robot searched for the nearest lethal cell.
CLEARANCE_SEARCH_M = 3.0


def latched():
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def grid_array(msg):
    """Occupancy as bytes, row 0 at the bottom, unknown as OCC_UNKNOWN."""
    data = np.frombuffer(msg.data, dtype=np.int8).astype(np.uint8)
    return data.reshape(msg.info.height, msg.info.width)


def downsample_max(grid, factor):
    """Block maximum over known cells; a block is unknown only if all of it is."""
    h, w = (grid.shape[0] // factor) * factor, (grid.shape[1] // factor) * factor
    blocks = grid[:h, :w].reshape(h // factor, factor, w // factor, factor)
    known = np.where(blocks == OCC_UNKNOWN, 0, blocks).max(axis=(1, 3))
    all_unknown = (blocks == OCC_UNKNOWN).all(axis=(1, 3))
    return np.where(all_unknown, OCC_UNKNOWN, known).astype(np.uint8)


class Mission(Node):
    def __init__(self):
        super().__init__("drive_mission")
        self.start = time.monotonic()
        self.pose = None
        self.poses = []
        self.windows = []
        self.plans = []
        self.routes = []
        self.global_frames = []
        self.local_frames = []
        self.goal_events = []
        self.last_pose_at = -math.inf

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(Path, "/route", self.on_route, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_window, latched())
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_global, latched()
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local, latched()
        )
        self.navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def now(self):
        return time.monotonic() - self.start

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, 2.0 * math.atan2(q.z, q.w))
        t = self.now()
        if t - self.last_pose_at >= 1.0 / POSE_RATE_HZ:
            self.last_pose_at = t
            self.poses.append((t, *self.pose))

    def on_plan(self, msg):
        xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        if len(xy):
            self.plans.append((self.now(), xy))

    def on_route(self, msg):
        xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        if len(xy):
            self.routes.append((self.now(), xy))

    def on_window(self, msg):
        info = msg.info
        self.windows.append(
            (
                self.now(),
                info.origin.position.x,
                info.origin.position.y,
                info.width * info.resolution,
                info.height * info.resolution,
            )
        )

    def on_global(self, msg):
        info = msg.info
        grid = downsample_max(grid_array(msg), GLOBAL_DOWNSAMPLE)
        meta = (
            self.now(),
            info.origin.position.x,
            info.origin.position.y,
            info.resolution * GLOBAL_DOWNSAMPLE,
        )
        self.global_frames.append((meta, grid))

    def on_local(self, msg):
        info = msg.info
        meta = (self.now(), info.origin.position.x, info.origin.position.y, info.resolution)
        self.local_frames.append((meta, grid_array(msg)))

    def spin_until(self, done, timeout):
        """Spins until `done()` is true. Returns False on timeout."""
        deadline = time.monotonic() + timeout
        while not done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def drive_to(self, index, x, y, yaw, timeout):
        """Sends one goal and waits for its result. Returns whether it succeeded."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        sent_at = self.now()
        self.goal_events.append((sent_at, index, -1))
        send = self.navigate.send_goal_async(goal)
        if not self.spin_until(send.done, 30.0) or not send.result().accepted:
            print(f"FAIL waypoint {index} ({x:.1f}, {y:.1f}): goal not accepted", flush=True)
            return False
        result = send.result().get_result_async()
        if not self.spin_until(result.done, timeout):
            print(f"FAIL waypoint {index} ({x:.1f}, {y:.1f}): no result in {timeout:.0f} s", flush=True)
            return False
        status = result.result().status
        self.goal_events.append((self.now(), index, status))
        px, py, _ = self.pose
        miss = math.hypot(px - x, py - y)
        ok = status == GoalStatus.STATUS_SUCCEEDED and miss <= ARRIVAL_TOLERANCE_M
        print(
            f"{'PASS' if ok else 'FAIL'} waypoint {index} ({x:.1f}, {y:.1f}): status {status}, "
            f"{self.now() - sent_at:.1f} s, robot at ({px:.2f}, {py:.2f}), {miss:.2f} m away",
            flush=True,
        )
        return ok

    def save(self, path, waypoints):
        plan_xy = [xy for _, xy in self.plans]
        route_xy = [xy for _, xy in self.routes]
        np.savez_compressed(
            path,
            poses=np.array(self.poses),
            windows=np.array(self.windows),
            waypoints=np.array(waypoints),
            goal_events=np.array(self.goal_events),
            plan_t=np.array([t for t, _ in self.plans]),
            plan_len=np.array([len(xy) for xy in plan_xy]),
            plan_xy=np.concatenate(plan_xy) if plan_xy else np.zeros((0, 2)),
            route_t=np.array([t for t, _ in self.routes]),
            route_len=np.array([len(xy) for xy in route_xy]),
            route_xy=np.concatenate(route_xy) if route_xy else np.zeros((0, 2)),
            global_meta=np.array([m for m, _ in self.global_frames]),
            global_grid=np.array([g for _, g in self.global_frames]),
            local_meta=np.array([m for m, _ in self.local_frames]),
            local_grid=np.array([g for _, g in self.local_frames]),
        )


def track_clearance(raster, poses, radius):
    """(cells under the robot's disc that are lethal, least distance to a lethal cell).

    The distance is from the robot centre to the nearest lethal cell centre and
    is capped at CLEARANCE_SEARCH_M.
    """
    cost, res = raster["expected"], raster["resolution"]
    origin_x, origin_y = raster["origin"]
    reach = int(math.ceil(CLEARANCE_SEARCH_M / res))
    height, width = cost.shape
    violations = 0
    least = CLEARANCE_SEARCH_M
    for _, x, y, _ in poses:
        x, y = x - origin_x, y - origin_y
        cx, cy = int(x / res), int(y / res)
        x0, x1 = max(cx - reach, 0), min(cx + reach + 1, width)
        y0, y1 = max(cy - reach, 0), min(cy + reach + 1, height)
        ys, xs = np.nonzero(cost[y0:y1, x0:x1] == LETHAL)
        if not len(xs):
            continue
        dist = np.hypot((xs + x0 + 0.5) * res - x, (ys + y0 + 0.5) * res - y)
        violations += int((dist < radius).sum())
        least = min(least, float(dist.min()))
    return violations, least


def parse_waypoints(text):
    return [tuple(float(v) for v in item.split(",")) for item in text.split()]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True, help="map the servers serve")
    parser.add_argument("--waypoints", required=True, help='"x,y x,y ..." in map metres')
    parser.add_argument("--out", required=True, help="directory for recording.npz")
    parser.add_argument("--robot-radius", type=float, default=0.3)
    parser.add_argument("--leg-timeout", type=float, default=180.0)
    args = parser.parse_args()
    waypoints = parse_waypoints(args.waypoints)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    mission = Mission()
    if not mission.navigate.wait_for_server(timeout_sec=SERVER_TIMEOUT_S):
        print(f"FAIL: no navigate_to_pose action server within {SERVER_TIMEOUT_S:.0f} s")
        return 1
    if not mission.spin_until(lambda: mission.pose is not None, 30.0):
        print("FAIL: no /odom within 30 s")
        return 1
    print(f"mission: robot at ({mission.pose[0]:.2f}, {mission.pose[1]:.2f})", flush=True)

    reached = 0
    for index, (x, y) in enumerate(waypoints):
        nx, ny = waypoints[min(index + 1, len(waypoints) - 1)]
        yaw = math.atan2(ny - y, nx - x) if (nx, ny) != (x, y) else 0.0
        if not mission.drive_to(index, x, y, yaw, args.leg_timeout):
            break
        reached += 1
    # One more second of costmaps after the last goal.
    mission.spin_until(lambda: False, 1.0)
    mission.save(out / "recording.npz", waypoints)

    poses = np.array(mission.poses)
    steps = np.hypot(np.diff(poses[:, 1]), np.diff(poses[:, 2]))
    violations, least = track_clearance(load_map(args.map), poses, args.robot_radius)
    print(
        f"drove {steps.sum():.1f} m in {poses[-1, 0] - poses[0, 0]:.0f} s; "
        f"{len(mission.windows)} windows published; {len(mission.routes)} routes; {len(mission.plans)} plans"
    )
    print(
        f"track: {violations} lethal cells under the {args.robot_radius} m robot disc, "
        f"least distance to a lethal cell {least:.2f} m"
    )
    ok = reached == len(waypoints) and violations == 0
    print(f"{'PASS' if ok else 'FAIL'}: {reached}/{len(waypoints)} waypoints reached")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
