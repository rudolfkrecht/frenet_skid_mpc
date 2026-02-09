#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header


# -------------------------
# Helpers
# -------------------------
def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def arc_length(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points))
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


def resample_polyline(points: np.ndarray, ds_target: float) -> np.ndarray:
    """
    Resample a polyline to uniform spacing ds_target along arc length.
    Returns resampled points (including first and last).
    """
    if len(points) < 2:
        return points.copy()

    s = arc_length(points)
    total = float(s[-1])
    if total < 1e-9:
        return points[:1].copy()

    n = max(int(math.floor(total / ds_target)) + 1, 2)
    s_new = np.linspace(0.0, total, n)

    x_new = np.interp(s_new, s, points[:, 0])
    y_new = np.interp(s_new, s, points[:, 1])
    return np.column_stack([x_new, y_new])


# -------------------------
# Node
# -------------------------
class PathToMicroWaypoints(Node):
    def __init__(self):
        super().__init__("path_to_micro_waypoints")

        # Parameters
        self.declare_parameter("input_path_topic", "/generated_path")
        self.declare_parameter("spacing", 0.2)
        self.declare_parameter("micro_waypoints_topic", "/micro_waypoints")
        self.declare_parameter("micro_waypoints_path_topic", "/micro_waypoints_path")
        self.declare_parameter("marker_topic", "/micro_waypoints_marker")
        self.declare_parameter("marker_scale", 0.08)

        in_topic = str(self.get_parameter("input_path_topic").value)

        # Subscriptions (keep default QoS; Path usually published continuously upstream)
        self.sub = self.create_subscription(Path, in_topic, self.cb_path, 10)

        # "Latched" QoS (RViz can start later and still get last message)
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pub_posearray = self.create_publisher(
            PoseArray, str(self.get_parameter("micro_waypoints_topic").value), qos_latched
        )
        self.pub_path = self.create_publisher(
            Path, str(self.get_parameter("micro_waypoints_path_topic").value), qos_latched
        )
        self.pub_marker = self.create_publisher(
            Marker, str(self.get_parameter("marker_topic").value), qos_latched
        )

        self.get_logger().info(
            f"PathToMicroWaypoints started. Subscribing: {in_topic} (outputs are TRANSIENT_LOCAL for RViz)"
        )

    def cb_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn("Input path has <2 poses.")
            return

        spacing = float(self.get_parameter("spacing").value)
        frame_id = msg.header.frame_id if msg.header.frame_id else "map"

        # Extract XY
        P = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses], dtype=float)

        # Remove consecutive duplicates / tiny steps
        filtered = [P[0]]
        for i in range(1, len(P)):
            if np.linalg.norm(P[i] - filtered[-1]) > 1e-4:
                filtered.append(P[i])
        P = np.asarray(filtered, dtype=float)

        if len(P) < 2:
            self.get_logger().warn("Path collapsed after filtering duplicates.")
            return

        # Resample
        Pm = resample_polyline(P, spacing)
        yaw = headings_from_points(Pm)

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=frame_id)

        # PoseArray
        pa = PoseArray()
        pa.header = header
        for i in range(len(Pm)):
            pose = Pose()
            pose.position.x = float(Pm[i, 0])
            pose.position.y = float(Pm[i, 1])
            pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(float(yaw[i]))
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pa.poses.append(pose)
        self.pub_posearray.publish(pa)

        # Path
        out_path = Path()
        out_path.header = header
        for pose in pa.poses:
            ps = PoseStamped()
            ps.header = header
            ps.pose = pose
            out_path.poses.append(ps)
        self.pub_path.publish(out_path)

        # Optional: clear old markers (helps during dev / when paths shrink)
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        self.pub_marker.publish(clear)

        # Marker (points)
        mk = Marker()
        mk.header = header
        mk.ns = "micro_waypoints"
        mk.id = 0
        mk.type = Marker.SPHERE_LIST
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0

        s = float(self.get_parameter("marker_scale").value)
        mk.scale.x = s
        mk.scale.y = s
        mk.scale.z = s

        mk.color.a = 1.0
        mk.color.r = 0.1
        mk.color.g = 0.6
        mk.color.b = 1.0

        mk.points = []
        for i in range(len(Pm)):
            pt = Point()
            pt.x = float(Pm[i, 0])
            pt.y = float(Pm[i, 1])
            pt.z = 0.0
            mk.points.append(pt)

        self.pub_marker.publish(mk)

        self.get_logger().info(
            f"Resampled path: in={len(msg.poses)} filtered={len(P)} out={len(Pm)} "
            f"(spacing={spacing:.2f} m, frame={frame_id})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathToMicroWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
