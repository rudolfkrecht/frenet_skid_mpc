#!/usr/bin/env python3
"""
dynamic_uniform_spline_publisher_node.py (ROS 2 Humble)

RViz-friendly version:
- Publishes Path + Marker topics with TRANSIENT_LOCAL durability ("latched" in ROS2),
  so RViz can start later and still see the last published markers/path.
- Optional: publishes Marker.DELETEALL before adding markers.
"""

import math
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float32MultiArray, ColorRGBA


# ----------------------------
# Basic math helpers
# ----------------------------
def norm2(v: np.ndarray) -> float:
    return float(np.hypot(v[0], v[1]))


def unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = norm2(v)
    return v / n if n > eps else np.zeros_like(v)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def cumulative_arc_length(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.zeros(len(points), dtype=float)
    s[1:] = np.cumsum(ds)
    return s


def headings_from_points(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    d = np.diff(points, axis=0)
    th = np.zeros(len(points), dtype=float)
    th[:-1] = np.arctan2(d[:, 1], d[:, 0])
    th[-1] = th[-2]
    return np.unwrap(th)


def curvature_discrete(points: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    kappa = np.zeros(n, dtype=float)
    if n < 3:
        return kappa

    for i in range(1, n - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        a = norm2(p1 - p0)
        b = norm2(p2 - p1)
        c = norm2(p2 - p0)
        denom = a * b * c + eps

        cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
        area2 = abs(cross)
        k = 2.0 * area2 / denom
        kappa[i] = k * np.sign(cross)

    return kappa


# ----------------------------
# Identified yaw model -> r_max
# ----------------------------
def yaw_rate_max_from_identified(a_r: float, b_r: float, tau_delta_max: float, eta: float) -> float:
    if abs(a_r) < 1e-9:
        return eta * b_r * tau_delta_max
    return eta * (b_r / a_r) * tau_delta_max


# ----------------------------
# Hermite spline pieces
# ----------------------------
def compute_bisector_tangents(P: np.ndarray, stop_flags: np.ndarray, base_gain: float) -> np.ndarray:
    n = len(P)
    m = np.zeros((n, 2), dtype=float)

    for i in range(n):
        if stop_flags[i] > 0.5:
            m[i] = 0.0
            continue

        if i == 0:
            d = unit(P[1] - P[0])
            spacing = norm2(P[1] - P[0])
            m[i] = base_gain * spacing * d
        elif i == n - 1:
            d = unit(P[-1] - P[-2])
            spacing = norm2(P[-1] - P[-2])
            m[i] = base_gain * spacing * d
        else:
            d_prev = unit(P[i] - P[i - 1])
            d_next = unit(P[i + 1] - P[i])
            spacing = min(norm2(P[i] - P[i - 1]), norm2(P[i + 1] - P[i]))

            b = d_prev + d_next
            if norm2(b) < 1e-9:
                b = d_next

            d = unit(b)
            m[i] = base_gain * spacing * d

    return m


def hermite_segment(P0, P1, m0, m1, u: np.ndarray) -> np.ndarray:
    u2 = u * u
    u3 = u2 * u
    h00 = 2 * u3 - 3 * u2 + 1
    h10 = u3 - 2 * u2 + u
    h01 = -2 * u3 + 3 * u2
    h11 = u3 - u2
    return (
        (h00[:, None] * P0)
        + (h10[:, None] * m0)
        + (h01[:, None] * P1)
        + (h11[:, None] * m1)
    )


def sample_spline(P: np.ndarray, m: np.ndarray, samples_per_meter: float):
    dense = []
    seg_id = []

    for i in range(len(P) - 1):
        seg_len = norm2(P[i + 1] - P[i])
        n = max(int(math.ceil(seg_len * samples_per_meter)), 10)
        u = np.linspace(0.0, 1.0, n, endpoint=False)
        pts = hermite_segment(P[i], P[i + 1], m[i], m[i + 1], u)
        dense.append(pts)
        seg_id.extend([i] * len(pts))

    dense = np.vstack(dense)
    dense = np.vstack([dense, P[-1]])
    seg_id.append(len(P) - 2)
    return dense, np.array(seg_id, dtype=int)


def pick_global_tangent_scale(
    P: np.ndarray,
    stop_flags: np.ndarray,
    m_base: np.ndarray,
    samples_per_meter: float,
    kappa_abs: float,
    max_iter: int = 20,
):
    def scaled(s: float):
        m = m_base.copy()
        for i in range(len(m)):
            if stop_flags[i] > 0.5:
                m[i] = 0.0
            else:
                m[i] *= s

        pts, seg_id = sample_spline(P, m, samples_per_meter=samples_per_meter)
        kappa = curvature_discrete(pts)
        kappa_peak = float(np.max(np.abs(kappa))) if len(kappa) else 0.0
        feasible = (kappa_peak <= kappa_abs + 1e-6)
        return feasible, kappa_peak, pts, seg_id, kappa, m

    feas1, _, pts1, seg_id1, kappa1, m1 = scaled(1.0)
    if feas1:
        return 1.0, pts1, seg_id1, kappa1, m1

    feas0, _, pts0, seg_id0, kappa0, m0 = scaled(0.0)
    if not feas0:
        return 0.0, pts0, seg_id0, kappa0, m0

    lo, hi = 0.0, 1.0
    best = (0.0, pts0, seg_id0, kappa0, m0)

    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        feas, _, pts, seg_id, kappa, m = scaled(mid)
        if feas:
            lo = mid
            best = (mid, pts, seg_id, kappa, m)
        else:
            hi = mid

    return best


def build_speed_profile(
    points: np.ndarray,
    waypoints: np.ndarray,
    stop_flags: np.ndarray,
    kappa: np.ndarray,
    r_max: float,
    v_min: float,
    v_max: float,
    a_long_max: float,
):
    s = cumulative_arc_length(points)
    n = len(points)

    v = np.full(n, v_max, dtype=float)
    for i in range(n):
        kk = abs(kappa[i])
        v[i] = min(v_max, (r_max / kk) if kk > 1e-6 else v_max)

    v = np.clip(v, v_min, v_max)

    stop_indices = []
    for i in range(len(waypoints)):
        if stop_flags[i] > 0.5:
            d = np.linalg.norm(points - waypoints[i], axis=1)
            idx = int(np.argmin(d))
            stop_indices.append(idx)
            v[idx] = 0.0

    for i in range(1, n):
        ds = max(s[i] - s[i - 1], 1e-6)
        v[i] = min(v[i], math.sqrt(max(v[i - 1] ** 2 + 2 * a_long_max * ds, 0.0)))

    for i in range(n - 2, -1, -1):
        ds = max(s[i + 1] - s[i], 1e-6)
        v[i] = min(v[i], math.sqrt(max(v[i + 1] ** 2 + 2 * a_long_max * ds, 0.0)))

    stop_set = set(stop_indices)
    for i in range(n):
        if i in stop_set:
            continue
        v[i] = max(v[i], v_min)

    return v


# ----------------------------
# ROS 2 Node
# ----------------------------
class DynamicUniformSplinePublisher(Node):
    def __init__(self):
        super().__init__("dynamic_uniform_spline_publisher")

        # Params
        self.declare_parameter("waypoints_topic", "/waypoints")
        self.declare_parameter("path_topic", "/generated_path")
        self.declare_parameter("path_marker_topic", "/generated_path_marker")
        self.declare_parameter("wp_marker_topic", "/waypoints_marker")
        self.declare_parameter("feas_marker_topic", "/feasibility_marker")
        self.declare_parameter("vref_topic", "/v_ref")
        self.declare_parameter("frame_id", "map")

        self.declare_parameter("samples_per_meter", 60.0)
        self.declare_parameter("base_tangent_gain", 1.0)
        self.declare_parameter("line_width", 0.06)

        self.declare_parameter("a_r", 1.2)
        self.declare_parameter("b_r", 0.3)
        self.declare_parameter("tau_delta_max", 80.0)
        self.declare_parameter("eta", 0.7)

        self.declare_parameter("v_max", 1.0)
        self.declare_parameter("v_min", 0.5)
        self.declare_parameter("a_long_max", 0.4)

        self.declare_parameter("publish_deleteall", True)

        # QoS: latch-like for RViz
        self.qos_latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Sub: normal is fine
        self.sub = self.create_subscription(
            PoseArray,
            str(self.get_parameter("waypoints_topic").value),
            self.cb_waypoints,
            10,
        )

        # Pubs: latched
        self.pub_path = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), self.qos_latched
        )
        self.pub_path_marker = self.create_publisher(
            Marker, str(self.get_parameter("path_marker_topic").value), self.qos_latched
        )
        self.pub_wp_marker = self.create_publisher(
            Marker, str(self.get_parameter("wp_marker_topic").value), self.qos_latched
        )
        self.pub_feas_marker = self.create_publisher(
            Marker, str(self.get_parameter("feas_marker_topic").value), self.qos_latched
        )
        self.pub_vref = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("vref_topic").value), self.qos_latched
        )

        self.get_logger().info(
            "DynamicUniformSplinePublisher started.\n"
            "NOTE: Path/Markers are TRANSIENT_LOCAL so RViz can see them even if started later."
        )

    def _publish_deleteall(self, header: Header):
        mk = Marker()
        mk.header = header
        mk.action = Marker.DELETEALL
        mk.ns = ""
        mk.id = 0
        self.pub_path_marker.publish(mk)
        self.pub_wp_marker.publish(mk)
        self.pub_feas_marker.publish(mk)

    def cb_waypoints(self, msg: PoseArray):
        if len(msg.poses) < 2:
            self.get_logger().warn("Need at least 2 waypoints.")
            return

        P = np.array([[p.position.x, p.position.y] for p in msg.poses], dtype=float)
        stop_flags = np.array([1.0 if p.position.z >= 0.5 else 0.0 for p in msg.poses], dtype=float)

        frame_id = str(self.get_parameter("frame_id").value)
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        base_gain = float(self.get_parameter("base_tangent_gain").value)
        line_width = float(self.get_parameter("line_width").value)

        a_r = float(self.get_parameter("a_r").value)
        b_r = float(self.get_parameter("b_r").value)
        tau_delta_max = float(self.get_parameter("tau_delta_max").value)
        eta = float(self.get_parameter("eta").value)

        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        a_long_max = float(self.get_parameter("a_long_max").value)

        r_max = yaw_rate_max_from_identified(a_r, b_r, tau_delta_max, eta)
        kappa_abs = r_max / max(v_min, 1e-9)

        m_base = compute_bisector_tangents(P, stop_flags, base_gain=base_gain)
        scale, pts, _seg_id, kappa, _m_used = pick_global_tangent_scale(
            P=P,
            stop_flags=stop_flags,
            m_base=m_base,
            samples_per_meter=samples_per_meter,
            kappa_abs=kappa_abs,
            max_iter=22,
        )

        theta = headings_from_points(pts)
        kappa_peak = float(np.max(np.abs(kappa))) if len(kappa) else 0.0
        feasible = (kappa_peak <= kappa_abs + 1e-6)

        self.get_logger().info(
            f"N_wp={len(P)} N_pts={len(pts)} | "
            f"v_min={v_min:.3f} r_max={r_max:.3f} -> kappa_abs={kappa_abs:.3f} | "
            f"kappa_peak={kappa_peak:.3f} | scale={scale:.3f} | feasible={feasible}"
        )

        v_ref = build_speed_profile(
            points=pts,
            waypoints=P,
            stop_flags=stop_flags,
            kappa=kappa,
            r_max=r_max,
            v_min=v_min,
            v_max=v_max,
            a_long_max=a_long_max,
        )

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=frame_id)

        if bool(self.get_parameter("publish_deleteall").value):
            self._publish_deleteall(header)

        # Path
        path_msg = Path()
        path_msg.header = header
        for i, p in enumerate(pts):
            ps = PoseStamped()
            ps.header = header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(float(theta[i]))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)
        self.pub_path.publish(path_msg)

        # Waypoints marker
        wp_mk = Marker()
        wp_mk.header = header
        wp_mk.ns = "waypoints"
        wp_mk.id = 0
        wp_mk.type = Marker.SPHERE_LIST
        wp_mk.action = Marker.ADD
        wp_mk.pose.orientation.w = 1.0
        wp_mk.scale.x = 0.12
        wp_mk.scale.y = 0.12
        wp_mk.scale.z = 0.12
        wp_mk.color.a = 1.0
        wp_mk.color.r = 1.0
        wp_mk.color.g = 0.4
        wp_mk.color.b = 0.0
        wp_mk.points = []
        for p in P:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.0
            wp_mk.points.append(pt)
        self.pub_wp_marker.publish(wp_mk)

        # Path marker
        mk = Marker()
        mk.header = header
        mk.ns = "generated_spline"
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = line_width
        mk.color.a = 1.0
        mk.color.r = 0.0
        mk.color.g = 1.0
        mk.color.b = 0.0
        mk.points = []
        for p in pts:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.0
            mk.points.append(pt)
        self.pub_path_marker.publish(mk)

        # Feasibility marker (colored line)
        feas_mk = Marker()
        feas_mk.header = header
        feas_mk.ns = "feasibility"
        feas_mk.id = 0
        feas_mk.type = Marker.LINE_STRIP
        feas_mk.action = Marker.ADD
        feas_mk.pose.orientation.w = 1.0
        feas_mk.scale.x = line_width * 1.25
        feas_mk.points = []
        feas_mk.colors = []
        feas_mk.color.a = 1.0

        for i, p in enumerate(pts):
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.0
            feas_mk.points.append(pt)

            c = ColorRGBA()
            c.a = 1.0
            if abs(kappa[i]) <= kappa_abs:
                c.r, c.g, c.b = 0.1, 0.9, 0.1
            else:
                c.r, c.g, c.b = 0.95, 0.1, 0.1
            feas_mk.colors.append(c)

        self.pub_feas_marker.publish(feas_mk)

        # v_ref
        vmsg = Float32MultiArray()
        vmsg.data = [float(v) for v in v_ref]
        self.pub_vref.publish(vmsg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicUniformSplinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
