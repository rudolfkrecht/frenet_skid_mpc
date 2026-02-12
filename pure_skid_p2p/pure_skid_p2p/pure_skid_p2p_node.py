#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class PureSkidP2P(Node):
    """
    PURE baseline P2P:

      d = ||p_w - p||
      theta_d = atan2(dy, dx)
      dtheta = wrap(theta_d - yaw)

      v = v_nom (constant forward speed)
      omega = k_w * dtheta

    Waypoint:
      if d < reach_radius -> idx++

    No:
      - curvature clipping
      - yaw-rate cap
      - slew limits
      - ALIGN/GOTO FSM
      - behind-skip
      - slowdown
    """

    def __init__(self):
        super().__init__("pure_skid_p2p")

        # Topics / frames
        self.declare_parameter("path_topic", "/micro_waypoints_path")
        self.declare_parameter("cmdvel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")

        # Rate
        self.declare_parameter("rate_hz", 20.0)

        # Pure P2P control
        self.declare_parameter("v_nom", 0.22)
        self.declare_parameter("k_w", 1.8)
        self.declare_parameter("reach_radius", 0.25)
        self.declare_parameter("stop_at_end", True)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path
        self.have_path = False
        self.xy: Optional[np.ndarray] = None
        self.path_frame: str = "odom"
        self.idx = 0

        # ROS I/O
        self.sub_path = self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.cb_path, 10
        )
        self.pub_cmd = self.create_publisher(
            Twist, str(self.get_parameter("cmdvel_topic").value), 10
        )
        self.pub_dbg = self.create_publisher(String, "/pure_skid_p2p_state", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.step)

        self.get_logger().info("PureSkidP2P started.")

    def cb_path(self, msg: Path):
        n = len(msg.poses)
        if n < 2:
            self.have_path = False
            self.get_logger().warn("micro_waypoints_path has <2 poses.")
            return

        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.path_frame
        self.xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)

        self.have_path = True
        self.idx = 0

        self.get_logger().info(f"Received micro path N={n} frame={self.path_frame}")

    def get_pose(self) -> Optional[Pose2D]:
        base = str(self.get_parameter("base_frame").value)
        try:
            tf = self.tf_buffer.lookup_transform(self.path_frame, base, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({self.path_frame}<-{base}): {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return Pose2D(float(t.x), float(t.y), float(yaw))

    def step(self):
        if not self.have_path or self.xy is None:
            return

        if self.idx >= len(self.xy):
            self.pub_cmd.publish(Twist())
            return

        pose = self.get_pose()
        if pose is None:
            return

        reach = float(self.get_parameter("reach_radius").value)

        # stop at end
        if bool(self.get_parameter("stop_at_end").value) and self.idx >= len(self.xy) - 1:
            w_last = self.xy[-1]
            d_last = math.hypot(float(w_last[0] - pose.x), float(w_last[1] - pose.y))
            if d_last < reach:
                self.pub_cmd.publish(Twist())
                return

        # current waypoint
        wpt = self.xy[self.idx]
        dx = float(wpt[0] - pose.x)
        dy = float(wpt[1] - pose.y)
        d = math.hypot(dx, dy)

        # reach -> advance (single-step advance, intentionally simple)
        if d < reach and self.idx < len(self.xy) - 1:
            self.idx += 1
            wpt = self.xy[self.idx]
            dx = float(wpt[0] - pose.x)
            dy = float(wpt[1] - pose.y)
            d = math.hypot(dx, dy)

        theta_d = math.atan2(dy, dx)
        dtheta = wrap_to_pi(theta_d - pose.yaw)

        v = float(self.get_parameter("v_nom").value)
        k_w = float(self.get_parameter("k_w").value)
        omega = k_w * dtheta

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

        dbg = String()
        dbg.data = f"idx={self.idx}/{len(self.xy)} d={d:.2f} dth={dtheta:.2f} v={v:.2f} w={omega:.2f}"
        self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = PureSkidP2P()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
