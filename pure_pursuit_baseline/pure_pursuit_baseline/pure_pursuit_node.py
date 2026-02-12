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


def quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class PurePursuitBaseline(Node):
    """
    Very pure Pure Pursuit baseline (constant-speed).

    - Lookahead point is chosen along the path as the first point with
      distance >= Ld from current position (starting from current idx).
    - Curvature: kappa = 2*y_b / Ld^2  (where (x_b, y_b) is lookahead point in base frame)
    - Commands:
        v = v_nom
        omega = v * kappa

    No:
      - curvature/yaw-rate clipping
      - slew limits
      - slowdown
      - FSM
    """

    def __init__(self):
        super().__init__("pure_pursuit_baseline")

        # Topics / frames
        self.declare_parameter("path_topic", "/micro_waypoints_path")
        self.declare_parameter("cmdvel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")

        # Rate
        self.declare_parameter("rate_hz", 20.0)

        # Pure Pursuit params
        self.declare_parameter("v_nom", 0.22)
        self.declare_parameter("lookahead", 0.60)      # Ld [m]
        self.declare_parameter("reach_radius", 0.25)   # for advancing idx and stop_at_end
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
        self.pub_dbg = self.create_publisher(String, "/pure_pursuit_state", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.step)

        self.get_logger().info("PurePursuitBaseline started.")

    def cb_path(self, msg: Path):
        n = len(msg.poses)
        if n < 2:
            self.have_path = False
            self.get_logger().warn("Path has <2 poses.")
            return

        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.path_frame
        self.xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)

        self.have_path = True
        self.idx = 0
        self.get_logger().info(f"Received path N={n} frame={self.path_frame}")

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

    def to_base_frame(self, pose: Pose2D, wpt: np.ndarray) -> tuple[float, float]:
        """World -> base coordinates of waypoint."""
        dx = float(wpt[0] - pose.x)
        dy = float(wpt[1] - pose.y)
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        # Rotate by -yaw (world->base)
        x_b = c * dx + s * dy
        y_b = -s * dx + c * dy
        return x_b, y_b

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

        # stop at end (pure baseline)
        if bool(self.get_parameter("stop_at_end").value) and self.idx >= len(self.xy) - 1:
            w_last = self.xy[-1]
            d_last = math.hypot(float(w_last[0] - pose.x), float(w_last[1] - pose.y))
            if d_last < reach:
                self.pub_cmd.publish(Twist())
                return

        # advance idx if current point reached
        w_cur = self.xy[self.idx]
        d_cur = math.hypot(float(w_cur[0] - pose.x), float(w_cur[1] - pose.y))
        if d_cur < reach and self.idx < len(self.xy) - 1:
            self.idx += 1

        # choose lookahead point: first path point with distance >= Ld
        Ld = float(self.get_parameter("lookahead").value)
        Ld = max(1e-3, Ld)

        j = self.idx
        chosen = self.xy[-1]
        while j < len(self.xy):
            w = self.xy[j]
            d = math.hypot(float(w[0] - pose.x), float(w[1] - pose.y))
            if d >= Ld:
                chosen = w
                break
            j += 1

        x_b, y_b = self.to_base_frame(pose, chosen)

        # pure pursuit curvature (geometric)
        # kappa = 2*y_b / Ld^2
        kappa = 2.0 * y_b / (Ld * Ld)

        v = float(self.get_parameter("v_nom").value)
        omega = v * kappa  # NO clipping

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

        dbg = String()
        dbg.data = (
            f"idx={self.idx}/{len(self.xy)} j={j} "
            f"Ld={Ld:.2f} xb={x_b:.2f} yb={y_b:.2f} "
            f"kappa={kappa:.2f} v={v:.2f} w={omega:.2f}"
        )
        self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitBaseline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
