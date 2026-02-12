#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class TorqueSupervisor(Node):
    """
    Model-based cmd_vel supervisor that estimates left/right torque and prevents torque saturation.

    Inputs:
      - cmd_vel_in (Twist): desired v, w (e.g., Frenet MPC output)
      - odom (Odometry): measured v, r from odom.twist.twist

    Outputs:
      - cmd_vel_out (Twist): supervised v, w
      - torque_est (Float32MultiArray): [tau_L, tau_R, tau_sum, tau_diff, a_v_des, a_r_des, scale]
    """

    def __init__(self):
        super().__init__("torque_supervisor")

        # --- Topics ---
        self.declare_parameter("cmdvel_in_topic", "/cmd_vel_frenet")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmdvel_out_topic", "/cmd_vel")
        self.declare_parameter("torque_est_topic", "/torque_est")

        # --- Timing ---
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("Ts", 0.05)  # if <=0, computed from rate_hz

        # --- Identified model params (normalized form) ---
        # dot(v) = b_v*tau_sum - f_c*tanh(v/v_eps) - a_v*v
        # dot(r) = b_r*tau_diff - a_r*r
        self.declare_parameter("a_v", 0.0)       # [1/s]
        self.declare_parameter("b_v", 1.0)       # [ (m/s^2) / torque_unit ]
        self.declare_parameter("f_c", 0.0)       # [m/s^2]
        self.declare_parameter("v_eps", 0.05)    # [m/s] tanh smoothing

        self.declare_parameter("a_r", 0.0)       # [1/s]
        self.declare_parameter("b_r", 1.0)       # [ (rad/s^2) / torque_unit ]

        # --- Torque limits ---
        self.declare_parameter("tau_side_max", 999.0)   # |tau_L|,|tau_R| <=
        self.declare_parameter("tau_sum_max", 999.0)    # |tau_sum| <=
        self.declare_parameter("tau_diff_max", 999.0)   # |tau_diff| <=

        # --- Optional command limits ---
        self.declare_parameter("v_max", 0.30)
        self.declare_parameter("w_max", 1.50)

        # --- State ---
        self.last_cmd_in: Optional[Twist] = None
        self.have_odom = False
        self.v_meas = 0.0
        self.r_meas = 0.0

        # --- ROS I/O ---
        self.sub_cmd = self.create_subscription(
            Twist, str(self.get_parameter("cmdvel_in_topic").value), self.cb_cmd, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.cb_odom, 20
        )

        self.pub_cmd = self.create_publisher(
            Twist, str(self.get_parameter("cmdvel_out_topic").value), 10
        )
        self.pub_tau = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("torque_est_topic").value), 10
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.step)

        self.get_logger().info("TorqueSupervisor started (odom-only).")

    def cb_cmd(self, msg: Twist):
        self.last_cmd_in = msg

    def cb_odom(self, msg: Odometry):
        self.v_meas = float(msg.twist.twist.linear.x)
        self.r_meas = float(msg.twist.twist.angular.z)
        self.have_odom = True

    def estimate_torques(self, v_des: float, w_des: float, Ts: float):
        a_v = float(self.get_parameter("a_v").value)
        b_v = float(self.get_parameter("b_v").value)
        f_c = float(self.get_parameter("f_c").value)
        v_eps = max(1e-6, float(self.get_parameter("v_eps").value))

        a_r = float(self.get_parameter("a_r").value)
        b_r = float(self.get_parameter("b_r").value)

        v = self.v_meas
        r = self.r_meas

        a_v_des = (v_des - v) / max(Ts, 1e-3)
        a_r_des = (w_des - r) / max(Ts, 1e-3)

        coul = math.tanh(v / v_eps)

        if abs(b_v) < 1e-9:
            tau_sum = 0.0
        else:
            tau_sum = (a_v_des + f_c * coul + a_v * v) / b_v

        if abs(b_r) < 1e-9:
            tau_diff = 0.0
        else:
            tau_diff = (a_r_des + a_r * r) / b_r

        tau_R = 0.5 * (tau_sum + tau_diff)
        tau_L = 0.5 * (tau_sum - tau_diff)

        return tau_L, tau_R, tau_sum, tau_diff, a_v_des, a_r_des

    def compute_scale(self, tau_L: float, tau_R: float, tau_sum: float, tau_diff: float) -> float:
        tau_side_max = float(self.get_parameter("tau_side_max").value)
        tau_sum_max = float(self.get_parameter("tau_sum_max").value)
        tau_diff_max = float(self.get_parameter("tau_diff_max").value)

        scales = []

        if tau_side_max > 0.0:
            sL = tau_side_max / max(abs(tau_L), 1e-12)
            sR = tau_side_max / max(abs(tau_R), 1e-12)
            scales += [sL, sR]

        if tau_sum_max > 0.0:
            scales.append(tau_sum_max / max(abs(tau_sum), 1e-12))
        if tau_diff_max > 0.0:
            scales.append(tau_diff_max / max(abs(tau_diff), 1e-12))

        if not scales:
            return 1.0

        s = min(1.0, min(scales))
        return float(max(0.0, s))

    def step(self):
        if self.last_cmd_in is None or not self.have_odom:
            return

        rate_hz = float(self.get_parameter("rate_hz").value)
        Ts = float(self.get_parameter("Ts").value)
        if Ts <= 0.0:
            Ts = 1.0 / max(rate_hz, 1e-3)

        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)

        v_des = clip(float(self.last_cmd_in.linear.x), -v_max, v_max)
        w_des = clip(float(self.last_cmd_in.angular.z), -w_max, w_max)

        tau_L, tau_R, tau_sum, tau_diff, _, _ = self.estimate_torques(v_des, w_des, Ts)
        s = self.compute_scale(tau_L, tau_R, tau_sum, tau_diff)

        v_out = v_des * s
        w_out = w_des * s

        tau_L2, tau_R2, tau_sum2, tau_diff2, a_v2, a_r2 = self.estimate_torques(v_out, w_out, Ts)

        out = Twist()
        out.linear.x = float(v_out)
        out.angular.z = float(w_out)
        self.pub_cmd.publish(out)

        m = Float32MultiArray()
        m.data = [
            float(tau_L2),
            float(tau_R2),
            float(tau_sum2),
            float(tau_diff2),
            float(a_v2),
            float(a_r2),
            float(s),
        ]
        self.pub_tau.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
