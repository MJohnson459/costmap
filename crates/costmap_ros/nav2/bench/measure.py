"""Measures planning latency, memory and CPU of running navigation processes.

Sends ComputePathToPose goals to one action server for the whole trip and to
another for the first NEAR_M metres of it, then reads the resident memory and
CPU time of the named process groups from /proc. Writes one JSON object.
"""

import argparse
import json
import math
import os
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Distance along the trip between the starts plans are asked from.
START_SPACING_M = 5.0
STARTS = 10
NEAR_REPEATS = 5
SERVER_TIMEOUT_S = 120.0
PLAN_TIMEOUT_S = 300.0
READY_TIMEOUT_S = 120.0
CLOCK_TICKS = os.sysconf("SC_CLK_TCK")
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024


def pose(x, y):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.w = 1.0
    return msg


def group_processes(pgid):
    """Process ids in process group `pgid`."""
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/stat") as stat:
                # The command name is in parentheses and may hold spaces.
                fields = stat.read().rsplit(")", 1)[1].split()
        except OSError:
            continue
        if int(fields[2]) == pgid:
            found.append(int(name))
    return found


def group_usage(pgid):
    """(resident MB now, peak resident MB, CPU seconds) summed over a group."""
    rss_kb = peak_kb = ticks = 0
    for pid in group_processes(pgid):
        try:
            with open(f"/proc/{pid}/status") as status:
                for line in status:
                    if line.startswith("VmRSS:"):
                        rss_kb += int(line.split()[1])
                    elif line.startswith("VmHWM:"):
                        peak_kb += int(line.split()[1])
            with open(f"/proc/{pid}/stat") as stat:
                fields = stat.read().rsplit(")", 1)[1].split()
            ticks += int(fields[11]) + int(fields[12])
        except OSError:
            continue
    return rss_kb / 1024.0, peak_kb / 1024.0, ticks / CLOCK_TICKS


class NoPath(Exception):
    pass


class Planner:
    def __init__(self, node, action):
        self.node = node
        self.action = action
        self.client = ActionClient(node, ComputePathToPose, action)
        if not self.client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S):
            raise SystemExit(f"action {action} did not appear in {SERVER_TIMEOUT_S} s")

    def plan(self, start, goal):
        """(path points, wall milliseconds, planner-reported milliseconds)."""
        request = ComputePathToPose.Goal()
        request.start = pose(*start)
        request.goal = pose(*goal)
        request.use_start = True
        sent = time.monotonic()
        handle_future = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.node, handle_future, timeout_sec=PLAN_TIMEOUT_S)
        handle = handle_future.result()
        if handle is None or not handle.accepted:
            raise SystemExit(f"{self.action}: goal {start} -> {goal} was not accepted")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=PLAN_TIMEOUT_S)
        wall_ms = (time.monotonic() - sent) * 1e3
        wrapped = result_future.result()
        if wrapped is None:
            raise SystemExit(f"{self.action}: no result in {PLAN_TIMEOUT_S} s")
        result = wrapped.result
        if not result.path.poses:
            raise NoPath(
                f"{self.action}: no path {start} -> {goal}: "
                f"error {result.error_code} {result.error_msg}"
            )
        points = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
        reported = result.planning_time
        return points, wall_ms, reported.sec * 1e3 + reported.nanosec / 1e6


def plan_when_ready(planner, start, goal):
    """The first plan that succeeds. A costmap that has not yet taken in its
    map fails the goals sent before it has."""
    ready_by = time.monotonic() + READY_TIMEOUT_S
    while True:
        try:
            return planner.plan(start, goal)
        except NoPath as error:
            if time.monotonic() > ready_by:
                raise SystemExit(str(error))
            time.sleep(1.0)


def length_m(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def points_along(points, spacing_m, count):
    """Up to `count` points of the path, `spacing_m` apart, starting at its first."""
    picked, travelled, wanted = [points[0]], 0.0, spacing_m
    for a, b in zip(points, points[1:]):
        travelled += math.dist(a, b)
        if travelled >= wanted and len(picked) < count:
            picked.append(b)
            wanted += spacing_m
    return picked


def summary(values):
    return {
        "median_ms": round(statistics.median(values), 2),
        "max_ms": round(max(values), 2),
        "count": len(values),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trip-action", required=True, help="action that plans the whole trip")
    parser.add_argument("--near-action", required=True, help="action that plans the stretch ahead")
    parser.add_argument("--near-m", type=float, default=15.0)
    parser.add_argument("--start", required=True, help="x,y in the map frame")
    parser.add_argument("--goal", required=True, help="x,y in the map frame")
    parser.add_argument("--group", action="append", default=[], help="name=process group id")
    parser.add_argument("--idle-s", type=float, default=20.0)
    parser.add_argument("--out", required=True, help="file the JSON report is written to")
    args = parser.parse_args()
    start = tuple(float(v) for v in args.start.split(","))
    goal = tuple(float(v) for v in args.goal.split(","))
    groups = dict((name, int(pgid)) for name, pgid in (g.split("=") for g in args.group))

    rclpy.init()
    node = Node("costmap_bench")
    trip = Planner(node, args.trip_action)
    near = trip if args.near_action == args.trip_action else Planner(node, args.near_action)

    cpu_before = {name: group_usage(pgid)[2] for name, pgid in groups.items()}
    planning_started = time.monotonic()

    points, first_wall, first_reported = plan_when_ready(trip, start, goal)
    ready_s = time.monotonic() - planning_started
    starts = points_along(points, START_SPACING_M, STARTS)
    replans = [trip.plan(s, goal) for s in starts]
    near_goal = points_along(points, args.near_m, 2)[-1]
    plan_when_ready(near, start, near_goal)
    nears = [near.plan(start, near_goal) for _ in range(NEAR_REPEATS)]

    planning_s = time.monotonic() - planning_started
    cpu_planning = {name: group_usage(pgid)[2] for name, pgid in groups.items()}
    time.sleep(args.idle_s)
    usage = {name: group_usage(pgid) for name, pgid in groups.items()}

    report = {
        "trip_action": args.trip_action,
        "trip_length_m": round(length_m(points), 1),
        "ready_after_s": round(ready_s - first_wall / 1e3, 1),
        "trip_first": {"wall_ms": round(first_wall, 1), "reported_ms": round(first_reported, 1)},
        "trip_replans_wall": summary([wall for _, wall, _ in replans]),
        "trip_replans_reported": summary([reported for _, _, reported in replans]),
        "near_action": args.near_action,
        "near_length_m": round(length_m(nears[0][0]), 1),
        "near_wall": summary([wall for _, wall, _ in nears]),
        "near_reported": summary([reported for _, _, reported in nears]),
        "planning_window_s": round(planning_s, 1),
        "processes": {
            name: {
                "resident_mb": round(usage[name][0], 1),
                "peak_resident_mb": round(usage[name][1], 1),
                "cpu_s_while_planning": round(cpu_planning[name] - cpu_before[name], 2),
                "idle_cpu_percent": round(
                    100.0 * (usage[name][2] - cpu_planning[name]) / args.idle_s, 2
                ),
            }
            for name in groups
        },
    }
    text = json.dumps(report, indent=2)
    with open(args.out, "w") as out:
        out.write(text + "\n")
    print(text)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
