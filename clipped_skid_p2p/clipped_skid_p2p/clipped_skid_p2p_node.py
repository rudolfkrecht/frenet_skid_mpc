#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional, Tuple

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


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class ClippedSkidP2P(Node):
    """
    Simple curvature-clipped P2P follower with NO in-place rotation.

    Control:
      d = ||p_w - p||
      theta_d = atan2(dy,dx)
      dtheta = wrap(theta_d - theta)
      kappa = dtheta / max(d, eps_dist)
      kappa <- clip(kappa, -kappa_max, +kappa_max)

    Curvature limit:
      kappa_max = curv_scale * (1/R_min)

    omega = clip(v * kappa, -r_max, +r_max)

    Safeguard:
      - if target waypoint is behind in base frame (x_b < behind_x_thresh),
        advance idx until it's ahead (bounded skips per cycle).
    """

    def __init__(self):
        super().__init__("clipped_skid_p2p")

        # Topics / frames
        self.declare_parameter("path_topic", "/micro_waypoints_path")
        self.declare_parameter("cmdvel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")

        # Rate
        self.declare_parameter("rate_hz", 20.0)

        # P2P control
        self.declare_parameter("v_nom", 0.22)
        self.declare_parameter("v_align", 0.12)
        self.declare_parameter("eps_theta", 0.25)       # ALIGN->GOTO
        self.declare_parameter("theta_realign", 0.90)   # if GOTO and |dtheta| grows -> back to ALIGN
        self.declare_parameter("eps_dist", 0.40)
        self.declare_parameter("reach_radius", 0.25)

        # No in-place rotation
        self.declare_parameter("v_stop_thresh", 0.05)

        # Curvature + yaw bounds
        self.declare_parameter("R_min", 0.70)           # nominal minimum turning radius
        self.declare_parameter("curv_scale", 1.0)       # tight tracks: 2..4
        self.declare_parameter("r_max", 1.2)            # yaw-rate cap

        # Slew limits
        self.declare_parameter("dv_max", 0.6)           # m/s^2 (speed-up)
        self.declare_parameter("dv_min", 1.0)           # m/s^2 (slow-down)
        self.declare_parameter("dw_max", 1.2)           # rad/s^2

        # Behind-skip safeguard
        self.declare_parameter("behind_x_thresh", 0.05) # [m]
        self.declare_parameter("max_skip_per_step", 6)

        # Stop at end
        self.declare_parameter("stop_at_end", True)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path
        self.have_path = False
        self.xy: Optional[np.ndarray] = None
        self.path_frame: str = "odom"
        self.idx = 0

        # State
        self.state = "ALIGN"

        # Slew memory
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.t_prev = self.get_clock().now().nanoseconds * 1e-9

        # ROS I/O
        self.sub_path = self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.cb_path, 10
        )
        self.pub_cmd = self.create_publisher(
            Twist, str(self.get_parameter("cmdvel_topic").value), 10
        )
        self.pub_dbg = self.create_publisher(String, "/clipped_skid_p2p_state", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.step)

        self.get_logger().info("ClippedSkidP2P started.")

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
        self.state = "ALIGN"

        self.v_prev = 0.0
        self.w_prev = 0.0
        self.t_prev = self.get_clock().now().nanoseconds * 1e-9

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

    def x_in_base(self, pose: Pose2D, w: np.ndarray) -> float:
        dx = float(w[0] - pose.x)
        dy = float(w[1] - pose.y)
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        return c * dx + s * dy

    def slew(self, v_des: float, w_des: float) -> Tuple[float, float]:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = max(1e-3, now - self.t_prev)
        self.t_prev = now

        dv_up = float(self.get_parameter("dv_max").value) * dt
        dv_dn = float(self.get_parameter("dv_min").value) * dt
        dw = float(self.get_parameter("dw_max").value) * dt

        v_out = self.v_prev + clip(v_des - self.v_prev, -dv_dn, dv_up)
        w_out = self.w_prev + clip(w_des - self.w_prev, -dw, dw)

        self.v_prev = v_out
        self.w_prev = w_out
        return v_out, w_out

    def step(self):
        if not self.have_path or self.xy is None:
            return

        if self.idx >= len(self.xy):
            self.pub_cmd.publish(Twist())
            return

        pose = self.get_pose()
        if pose is None:
            return

        # Stop at end
        if bool(self.get_parameter("stop_at_end").value) and self.idx >= len(self.xy) - 1:
            w_last = self.xy[-1]
            d_last = math.hypot(float(w_last[0] - pose.x), float(w_last[1] - pose.y))
            if d_last < float(self.get_parameter("reach_radius").value):
                self.pub_cmd.publish(Twist())
                return

        # ---- behind-skip safeguard ----
        behind_x = float(self.get_parameter("behind_x_thresh").value)
        max_skip = int(self.get_parameter("max_skip_per_step").value)

        skips = 0
        while self.idx < len(self.xy) - 1 and skips < max_skip:
            x_b = self.x_in_base(pose, self.xy[self.idx])
            if x_b >= behind_x:
                break
            self.idx += 1
            self.state = "ALIGN"
            skips += 1

        wpt = self.xy[self.idx]

        # geometry
        dx = float(wpt[0] - pose.x)
        dy = float(wpt[1] - pose.y)
        d = math.hypot(dx, dy)
        theta_d = math.atan2(dy, dx)
        dtheta = wrap_to_pi(theta_d - pose.yaw)

        # reach -> advance
        reach = float(self.get_parameter("reach_radius").value)
        if d < reach and self.idx < len(self.xy) - 1:
            self.idx += 1
            wpt = self.xy[self.idx]
            dx = float(wpt[0] - pose.x)
            dy = float(wpt[1] - pose.y)
            d = math.hypot(dx, dy)
            theta_d = math.atan2(dy, dx)
            dtheta = wrap_to_pi(theta_d - pose.yaw)

        # speed FSM
        v_nom = float(self.get_parameter("v_nom").value)
        v_align = float(self.get_parameter("v_align").value)
        eps_theta = float(self.get_parameter("eps_theta").value)
        theta_realign = float(self.get_parameter("theta_realign").value)

        if self.state == "ALIGN":
            v = v_align
            if abs(dtheta) < eps_theta:
                self.state = "GOTO"
        else:
            if abs(dtheta) > theta_realign:
                self.state = "ALIGN"
                v = v_align
            else:
                v = v_nom

        # curvature clipped P2P
        eps_dist = float(self.get_parameter("eps_dist").value)
        R_min = max(1e-3, float(self.get_parameter("R_min").value))
        curv_scale = max(0.1, float(self.get_parameter("curv_scale").value))
        kappa_max = curv_scale * (1.0 / R_min)

        kappa = dtheta / max(d, eps_dist)
        kappa = clip(kappa, -kappa_max, +kappa_max)

        r_max = float(self.get_parameter("r_max").value)
        omega = clip(v * kappa, -r_max, +r_max)

        # NO in-place
        v_stop = float(self.get_parameter("v_stop_thresh").value)
        if v < v_stop:
            omega = 0.0

        # slew-limit
        v_cmd, w_cmd = self.slew(v, omega)

        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.pub_cmd.publish(cmd)

        dbg = String()
        dbg.data = (
            f"state={self.state} idx={self.idx}/{len(self.xy)} "
            f"d={d:.2f} dth={dtheta:.2f} kappa={kappa:.2f} "
            f"v={v_cmd:.2f} w={w_cmd:.2f} "
            f"kmax={kappa_max:.2f} x_b={self.x_in_base(pose, wpt):.2f} skips={skips}"
        )
        self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ClippedSkidP2P()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
