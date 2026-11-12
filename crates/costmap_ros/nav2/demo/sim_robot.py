"""Kinematic differential-drive robot for the Nav2 drive demo.

Integrates the velocity Nav2's controller publishes on /cmd_vel and publishes
the resulting pose as odom -> base_link on /tf and as /odom. map -> odom is a
static identity transform: the robot knows where it is, with no drift.
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

RATE_HZ = 50.0
# A command older than this stops the robot.
CMD_TIMEOUT_S = 0.5


def transform(parent, child, stamp, x, y, yaw):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


class SimRobot(Node):
    def __init__(self, x, y, yaw):
        super().__init__("sim_robot")
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.cmd_at = None
        self.last = self.get_clock().now()

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        static_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        stamp = self.last.to_msg()
        self.static_pub.publish(
            TFMessage(transforms=[transform("map", "odom", stamp, 0.0, 0.0, 0.0)])
        )
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / RATE_HZ, self.step)
        self.get_logger().info(f"sim: robot at ({x:.2f}, {y:.2f}), yaw {yaw:.2f}")

    def on_cmd(self, msg):
        self.v = msg.twist.linear.x
        self.w = msg.twist.angular.z
        self.cmd_at = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if self.cmd_at is None or (now - self.cmd_at).nanoseconds * 1e-9 > CMD_TIMEOUT_S:
            self.v = self.w = 0.0
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        stamp = now.to_msg()
        tf = transform("odom", "base_link", stamp, self.x, self.y, self.yaw)
        self.tf_pub.publish(TFMessage(transforms=[tf]))

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    args = parser.parse_args()
    rclpy.init()
    node = SimRobot(args.x, args.y, args.yaw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
