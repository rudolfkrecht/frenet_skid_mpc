#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, String

from tf2_ros import Buffer, TransformListener
from scipy.optimize import minimize


# -------------------------
# Helpers
# -------------------------
def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def headings_from_points(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    d = np.diff(points, axis=0)
    th = np.zeros(len(points), dtype=float)
    th[:-1] = np.arctan2(d[:, 1], d[:, 0])
    th[-1] = th[-2]
    return np.unwrap(th)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class FrenetSkidMPC(Node):
    """
    Frenet MPC over a micro waypoint path (Twist output).

    Reads all parameters from YAML (ros__parameters) via declare_parameter defaults.
    """

    def __init__(self):
        super().__init__("frenet_skid_mpc")

        # Topics / frames
        self.declare_parameter("path_topic", "/micro_waypoints_path")
        self.declare_parameter("vref_topic", "/v_ref")  # optional
        self.declare_parameter("cmdvel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")

        # Rate / horizon
        self.declare_parameter("rate_hz", 6.0)
        self.declare_parameter("dt", 0.12)
        self.declare_parameter("N", 10)

        # Progress / robustness
        self.declare_parameter("lockon_dist", 2.0)
        self.declare_parameter("window", 300)
        self.declare_parameter("max_progress_step", 3)

        # Reference speed fallback
        self.declare_parameter("v_nom", 0.18)

        # Bounds / invariants
        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("v_min_move", 0.10)
        self.declare_parameter("r_max", 1.6)

        # Curvature policy
        self.declare_parameter("R_min", 0.50)
        self.declare_parameter("curv_scale", 3.5)

        # No in-place behavior
        self.declare_parameter("v_stop_thresh", 0.05)
        self.declare_parameter("turn_at_low_v_pen", 80.0)

        # End behavior
        self.declare_parameter("stop_at_end", True)
        self.declare_parameter("reach_radius", 0.12)
        self.declare_parameter("end_stop_points", 6)

        # Weights
        self.declare_parameter("w_cte", 60.0)
        self.declare_parameter("w_epsi", 8.0)
        self.declare_parameter("w_v", 3.0)
        self.declare_parameter("w_u", 0.3)
        self.declare_parameter("w_du", 15.0)

        # Soft constraint penalties
        self.declare_parameter("pen_curv", 800.0)
        self.declare_parameter("pen_rmax", 600.0)
        self.declare_parameter("pen_vmin", 300.0)

        # Solver
        self.declare_parameter("maxiter", 20)
        self.declare_parameter("ftol", 5e-3)
        self.declare_parameter("time_budget", 0.35)  # informational only

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path storage
        self.have_path = False
        self.frame_id: str = "odom"
        self.xy: Optional[np.ndarray] = None
        self.yaw_path: Optional[np.ndarray] = None
        self.ds_mean: float = 0.2
        self.v_ref: Optional[np.ndarray] = None  # optional

        # Progress
        self.progress_idx: int = 0
        self.locked: bool = False

        # Warm-start
        self.last_u_seq = None  # shape (N,2)

        # ROS I/O
        self.sub_path = self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.cb_path, 10
        )
        self.sub_vref = self.create_subscription(
            Float32MultiArray, str(self.get_parameter("vref_topic").value), self.cb_vref, 10
        )

        self.pub_cmd = self.create_publisher(Twist, str(self.get_parameter("cmdvel_topic").value), 10)
        self.pub_dbg = self.create_publisher(String, "/frenet_skid_mpc_debug", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.step)

        self.get_logger().info("Frenet Skid MPC started.")

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.have_path = False
            self.get_logger().warn("Path too short.")
            return

        self.frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id

        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)

        # basic duplicate filtering
        filtered = [xy[0]]
        for i in range(1, len(xy)):
            if np.linalg.norm(xy[i] - filtered[-1]) > 1e-4:
                filtered.append(xy[i])
        xy = np.asarray(filtered, dtype=float)

        if len(xy) < 2:
            self.have_path = False
            self.get_logger().warn("Path collapsed after filtering.")
            return

        yaw = headings_from_points(xy)
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        self.ds_mean = float(np.mean(seg)) if seg.size > 0 else 0.2
        self.ds_mean = max(self.ds_mean, 1e-3)

        self.xy = xy
        self.yaw_path = yaw
        self.have_path = True

        self.progress_idx = 0
        self.locked = False
        self.last_u_seq = None

        self.get_logger().info(
            f"Received path N={len(xy)} frame={self.frame_id} ds_mean={self.ds_mean:.3f}"
        )

        if self.v_ref is None or len(self.v_ref) != len(xy):
            self.v_ref = self.build_fallback_vref(len(xy))

    def cb_vref(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=float)
        self.v_ref = arr

        if self.have_path and self.xy is not None and len(arr) != len(self.xy):
            self.get_logger().warn("v_ref length mismatch vs path; using fallback.")
            self.v_ref = self.build_fallback_vref(len(self.xy))
        else:
            self.get_logger().info("Received external v_ref (will use if length matches path).")

    def build_fallback_vref(self, n: int) -> np.ndarray:
        v_nom = float(self.get_parameter("v_nom").value)
        end_stop_points = int(self.get_parameter("end_stop_points").value)
        v = np.full(n, v_nom, dtype=float)
        if end_stop_points > 0:
            v[max(0, n - end_stop_points):] = 0.0
        return v

    # -------------------------
    # TF pose
    # -------------------------
    def get_pose(self) -> Optional[Pose2D]:
        base = str(self.get_parameter("base_frame").value)
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, base, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({self.frame_id}<-{base}): {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return Pose2D(float(t.x), float(t.y), float(yaw))

    # -------------------------
    # Path utilities
    # -------------------------
    def nearest_index_global(self, x: float, y: float) -> int:
        assert self.xy is not None
        d2 = np.sum((self.xy - np.array([x, y])) ** 2, axis=1)
        return int(np.argmin(d2))

    def nearest_index_window(self, x: float, y: float, start_idx: int, window: int) -> int:
        assert self.xy is not None
        n = len(self.xy)
        i0 = int(np.clip(start_idx, 0, n - 1))
        i1 = int(np.clip(i0 + window, 0, n))
        chunk = self.xy[i0:i1]
        d2 = np.sum((chunk - np.array([x, y])) ** 2, axis=1)
        return int(i0 + np.argmin(d2))

    def distance_to_idx(self, pose: Pose2D, idx: int) -> float:
        assert self.xy is not None
        p = self.xy[int(np.clip(idx, 0, len(self.xy) - 1))]
        return float(math.hypot(float(p[0] - pose.x), float(p[1] - pose.y)))

    def frenet_errors(self, pose: Pose2D, idx: int) -> Tuple[float, float, float]:
        assert self.xy is not None and self.yaw_path is not None and self.v_ref is not None
        idx = int(np.clip(idx, 0, len(self.xy) - 1))
        xr, yr = self.xy[idx]
        psir = float(self.yaw_path[idx])
        vref = float(self.v_ref[idx])

        dx = pose.x - float(xr)
        dy = pose.y - float(yr)

        # left normal of tangent
        nx = -math.sin(psir)
        ny = math.cos(psir)

        cte = dx * nx + dy * ny
        epsi = wrap_to_pi(pose.yaw - psir)
        return float(cte), float(epsi), float(vref)

    # -------------------------
    # MPC model
    # -------------------------
    def step_model(self, state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        x, y, psi = state
        v, w = u
        x2 = x + dt * v * math.cos(psi)
        y2 = y + dt * v * math.sin(psi)
        psi2 = wrap_to_pi(psi + dt * w)
        return np.array([x2, y2, psi2], dtype=float)

    def solve_mpc(self, pose: Pose2D) -> Tuple[float, float, bool, float]:
        assert self.xy is not None and self.v_ref is not None

        dt = float(self.get_parameter("dt").value)
        N = int(self.get_parameter("N").value)

        v_max = float(self.get_parameter("v_max").value)
        r_max = float(self.get_parameter("r_max").value)
        v_min_move = float(self.get_parameter("v_min_move").value)

        R_min = max(1e-3, float(self.get_parameter("R_min").value))
        curv_scale = max(0.1, float(self.get_parameter("curv_scale").value))
        kappa_max = curv_scale * (1.0 / R_min)

        # weights
        w_cte = float(self.get_parameter("w_cte").value)
        w_epsi = float(self.get_parameter("w_epsi").value)
        w_v = float(self.get_parameter("w_v").value)
        w_u = float(self.get_parameter("w_u").value)
        w_du = float(self.get_parameter("w_du").value)

        pen_curv = float(self.get_parameter("pen_curv").value)
        pen_rmax = float(self.get_parameter("pen_rmax").value)
        pen_vmin = float(self.get_parameter("pen_vmin").value)
        lowv_turn_pen = float(self.get_parameter("turn_at_low_v_pen").value)

        maxiter = int(self.get_parameter("maxiter").value)
        ftol = float(self.get_parameter("ftol").value)

        # Freeze reference index during horizon
        ref0 = int(np.clip(self.progress_idx, 0, len(self.xy) - 1))

        # Warm start
        if self.last_u_seq is None or self.last_u_seq.shape != (N, 2):
            u0 = np.zeros((N, 2), dtype=float)
            _, _, vref0 = self.frenet_errors(pose, ref0)
            u0[:, 0] = clip(
                vref0 if vref0 > 0.0 else float(self.get_parameter("v_nom").value),
                0.0,
                v_max,
            )
            u0[:, 1] = 0.0
        else:
            u0 = np.vstack([self.last_u_seq[1:], self.last_u_seq[-1:]]).copy()

        u0_flat = u0.reshape(-1)

        bounds = []
        for _ in range(N):
            bounds.append((0.0, v_max))
            bounds.append((-r_max, r_max))

        x_init = np.array([pose.x, pose.y, pose.yaw], dtype=float)

        t_start = time.perf_counter()

        def objective(u_flat: np.ndarray) -> float:
            u = u_flat.reshape(N, 2)
            s = x_init.copy()
            J = 0.0
            prev = u0[0].copy()

            for k in range(N):
                vk = float(u[k, 0])
                wk = float(u[k, 1])

                s = self.step_model(s, np.array([vk, wk]), dt)
                xk, yk, psik = s

                cte, epsi, vref = self.frenet_errors(Pose2D(xk, yk, psik), ref0)

                J += w_cte * (cte ** 2)
                J += w_epsi * (epsi ** 2)
                J += w_v * ((vk - vref) ** 2)

                J += w_u * (vk ** 2 + wk ** 2)

                dv = vk - float(prev[0])
                dw = wk - float(prev[1])
                J += w_du * (dv ** 2 + dw ** 2)
                prev = u[k]

                # soft curvature: |w| <= v*kappa_max
                curv_excess = max(0.0, abs(wk) - (vk * kappa_max))
                J += pen_curv * (curv_excess ** 2)

                # soft yaw cap
                r_excess = max(0.0, abs(wk) - r_max)
                J += pen_rmax * (r_excess ** 2)

                # discourage turning while slow
                if vk < v_min_move:
                    J += lowv_turn_pen * (wk ** 2)
                    if vref > 1e-3:
                        J += pen_vmin * ((v_min_move - vk) ** 2)

            return float(J)

        res = minimize(
            objective,
            u0_flat,
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": maxiter, "ftol": ftol, "disp": False},
        )

        solve_s = time.perf_counter() - t_start
        ok = bool(res.success)

        if ok:
            u_star = res.x.reshape(N, 2)
            self.last_u_seq = u_star.copy()
            v_cmd = float(u_star[0, 0])
            w_cmd = float(u_star[0, 1])
        else:
            # fallback: do NOT stop; use warm-start first control
            v_cmd = float(u0[0, 0])
            w_cmd = float(u0[0, 1])

        # Hard output constraints
        v_cmd = clip(v_cmd, 0.0, v_max)
        w_cmd = clip(w_cmd, -r_max, r_max)

        # Hard curvature output: |w| <= v*kappa_max
        w_lim = v_cmd * kappa_max
        if w_lim >= 1e-6:
            w_cmd = clip(w_cmd, -w_lim, w_lim)

        # no in-place
        v_stop = float(self.get_parameter("v_stop_thresh").value)
        if v_cmd < v_stop:
            w_cmd = 0.0

        return float(v_cmd), float(w_cmd), ok, float(1000.0 * solve_s)

    # -------------------------
    # Main loop
    # -------------------------
    def step(self):
        if not self.have_path or self.xy is None:
            return

        pose = self.get_pose()
        if pose is None:
            return

        # Stop at end
        if bool(self.get_parameter("stop_at_end").value):
            if self.progress_idx >= len(self.xy) - 1:
                d_last = self.distance_to_idx(pose, len(self.xy) - 1)
                if d_last < float(self.get_parameter("reach_radius").value):
                    self.pub_cmd.publish(Twist())
                    return

        lockon_dist = float(self.get_parameter("lockon_dist").value)
        window = int(self.get_parameter("window").value)

        prev_idx = self.progress_idx

        if not self.locked:
            self.progress_idx = self.nearest_index_global(pose.x, pose.y)
            self.locked = True
        else:
            d_prog = self.distance_to_idx(pose, self.progress_idx)
            if d_prog > lockon_dist:
                self.progress_idx = self.nearest_index_global(pose.x, pose.y)
            else:
                nearest = self.nearest_index_window(pose.x, pose.y, self.progress_idx, window)
                self.progress_idx = max(self.progress_idx, nearest)

        # limit how fast progress can jump forward
        max_step = int(self.get_parameter("max_progress_step").value)
        self.progress_idx = min(self.progress_idx, prev_idx + max_step)

        v_cmd, w_cmd, ok, solve_ms = self.solve_mpc(pose)

        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.pub_cmd.publish(msg)

        cte, epsi, vref = self.frenet_errors(pose, self.progress_idx)

        dbg = String()
        dbg.data = (
            f"ok={ok} solve={solve_ms:.1f}ms "
            f"idx={self.progress_idx}/{len(self.xy)} "
            f"cte={cte:+.2f} epsi={epsi:+.2f} vref={vref:.2f} "
            f"cmd(v={v_cmd:.2f}, w={w_cmd:.2f}) frame={self.frame_id}"
        )
        self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetSkidMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
