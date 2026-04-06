#!/usr/bin/env python3

from __future__ import annotations

import math
import random
from typing import Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # ROS quaternion -> yaw (Z axis), assuming standard ENU frames.
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _rotate_body_to_world(dx_body: float, dy_body: float, yaw: float) -> Tuple[float, float]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    # [dx_world; dy_world] = R(yaw) * [dx_body; dy_body]
    return dx_body * cy - dy_body * sy, dx_body * sy + dy_body * cy


class OdomNoisyNode(Node):
    """
    Subscribe to a raw Odometry topic, inject slip-like drift + Gaussian noise,
    and publish to a new topic while keeping the original header stamp.

    The injected error is a combination of:
    - White Gaussian noise on pose (x,y,yaw) and twist (vx,vy,wz)
    - A bounded random-walk drift (x,y,yaw) updated at `noise_rate_hz`
      to mimic wheel slip / cumulative odom bias.
    """

    def __init__(self) -> None:
        super().__init__('odom_noisy')

        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom_noisy')
        self.declare_parameter('output_frame', 'odom_noisy')
        self.declare_parameter('output_child_frame', 'base_footprint_noisy')
        self.declare_parameter('noise_rate_hz', 30.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('min_abs_yaw_rate', 0.09)

        # White noise (1-sigma)
        self.declare_parameter('pos_xy_std', 0.01)          # meters
        self.declare_parameter('yaw_std', 0.005)            # rad
        self.declare_parameter('lin_vel_xy_std', 0.01)      # m/s
        self.declare_parameter('yaw_rate_std', 0.01)        # rad/s

        # Random-walk drift (per sqrt(second)) and bounds
        self.declare_parameter('drift_pos_rw', 0.02)        # m / sqrt(s)
        self.declare_parameter('drift_yaw_rw', 0.01)        # rad / sqrt(s)
        self.declare_parameter('drift_pos_max', 0.8)        # m
        self.declare_parameter('drift_yaw_max', 0.6)        # rad

        # Simple low-frequency slip "bursts"
        self.declare_parameter('slip_burst_prob', 0.02)     # probability per update
        self.declare_parameter('slip_burst_xy', 0.08)       # meters added to drift
        self.declare_parameter('slip_burst_yaw', 0.03)      # rad added to drift

        seed = int(self.get_parameter('seed').get_parameter_value().integer_value)
        if seed != 0:
            random.seed(seed)

        self._drift_x = 0.0
        self._drift_y = 0.0
        self._drift_yaw = 0.0

        # QoS notes:
        # - Many simulators publish /odom as BEST_EFFORT; subscribing with BEST_EFFORT is safe.
        # - RViz Odometry display subscribes with RELIABLE by default, so /odom_noisy must be RELIABLE.
        sub_qos = QoSProfile(depth=50)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = DurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=50)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.VOLATILE

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(Odometry, out_topic, pub_qos)
        self._sub = self.create_subscription(Odometry, in_topic, self._on_odom, sub_qos)

        rate = float(self.get_parameter('noise_rate_hz').get_parameter_value().double_value)
        if rate <= 0.0:
            rate = 30.0
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._on_timer)

        self._last_msg: Odometry | None = None

        self.get_logger().info(
            f'Odom noisy: {in_topic} -> {out_topic}, noise_rate_hz={rate:.1f}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_msg = msg

    def _on_timer(self) -> None:
        if self._last_msg is None:
            return
        msg = self._last_msg
        q = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        wz = float(msg.twist.twist.angular.z)
        self._update_drift(yaw=yaw, wz=wz)
        self._pub.publish(self._make_noisy(msg))

    def _update_drift(self, *, yaw: float, wz: float) -> None:
        dt = self._dt

        min_abs_wz = abs(float(self.get_parameter('min_abs_yaw_rate').get_parameter_value().double_value))
        if abs(wz) < min_abs_wz:
            return

        drift_pos_rw = float(self.get_parameter('drift_pos_rw').get_parameter_value().double_value)
        drift_yaw_rw = float(self.get_parameter('drift_yaw_rw').get_parameter_value().double_value)
        drift_pos_max = abs(float(self.get_parameter('drift_pos_max').get_parameter_value().double_value))
        drift_yaw_max = abs(float(self.get_parameter('drift_yaw_max').get_parameter_value().double_value))

        # Random walk: increment ~ N(0, sigma*sqrt(dt))
        sigma_pos = drift_pos_rw * math.sqrt(max(dt, 1e-6))
        sigma_yaw = drift_yaw_rw * math.sqrt(max(dt, 1e-6))
        dx_body = random.gauss(0.0, sigma_pos)
        dy_body = random.gauss(0.0, sigma_pos)
        dx_world, dy_world = _rotate_body_to_world(dx_body, dy_body, yaw)
        self._drift_x += dx_world
        self._drift_y += dy_world
        self._drift_yaw = _wrap_pi(self._drift_yaw + random.gauss(0.0, sigma_yaw))

        # Optional slip bursts
        p = float(self.get_parameter('slip_burst_prob').get_parameter_value().double_value)
        if p > 0.0 and random.random() < p:
            burst_xy = float(self.get_parameter('slip_burst_xy').get_parameter_value().double_value)
            burst_yaw = float(self.get_parameter('slip_burst_yaw').get_parameter_value().double_value)
            # Slip burst in body lateral direction (Y) with sign from turn rate.
            s = 1.0 if wz >= 0.0 else -1.0
            dxw, dyw = _rotate_body_to_world(0.0, s * burst_xy, yaw)
            self._drift_x += dxw
            self._drift_y += dyw
            self._drift_yaw = _wrap_pi(self._drift_yaw + random.gauss(0.0, abs(burst_yaw)))

        # Clamp drift
        if drift_pos_max > 0.0:
            self._drift_x = max(-drift_pos_max, min(drift_pos_max, self._drift_x))
            self._drift_y = max(-drift_pos_max, min(drift_pos_max, self._drift_y))
        if drift_yaw_max > 0.0:
            self._drift_yaw = max(-drift_yaw_max, min(drift_yaw_max, self._drift_yaw))

    def _make_noisy(self, msg: Odometry) -> Odometry:
        out = Odometry()
        out.header = msg.header  # keep stamp exactly the same
        out.header.frame_id = self.get_parameter('output_frame').get_parameter_value().string_value
        out.child_frame_id = self.get_parameter('output_child_frame').get_parameter_value().string_value

        out.pose = msg.pose
        out.twist = msg.twist

        # White noise parameters
        pos_xy_std = float(self.get_parameter('pos_xy_std').get_parameter_value().double_value)
        yaw_std = float(self.get_parameter('yaw_std').get_parameter_value().double_value)
        lin_vel_xy_std = float(self.get_parameter('lin_vel_xy_std').get_parameter_value().double_value)
        yaw_rate_std = float(self.get_parameter('yaw_rate_std').get_parameter_value().double_value)

        # Pose noise + drift in the odom frame
        out.pose.pose.position.x = float(msg.pose.pose.position.x + self._drift_x + random.gauss(0.0, pos_xy_std))
        out.pose.pose.position.y = float(msg.pose.pose.position.y + self._drift_y + random.gauss(0.0, pos_xy_std))
        out.pose.pose.position.z = float(msg.pose.pose.position.z)

        q = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        yaw_noisy = _wrap_pi(yaw + self._drift_yaw + random.gauss(0.0, yaw_std))
        qx, qy, qz, qw = _yaw_to_quat(yaw_noisy)
        out.pose.pose.orientation.x = float(qx)
        out.pose.pose.orientation.y = float(qy)
        out.pose.pose.orientation.z = float(qz)
        out.pose.pose.orientation.w = float(qw)

        # Twist noise (leave z and x/y angular as-is for TB3 typical planar odom)
        out.twist.twist.linear.x = float(msg.twist.twist.linear.x + random.gauss(0.0, lin_vel_xy_std))
        out.twist.twist.linear.y = float(msg.twist.twist.linear.y + random.gauss(0.0, lin_vel_xy_std))
        out.twist.twist.linear.z = float(msg.twist.twist.linear.z)
        out.twist.twist.angular.x = float(msg.twist.twist.angular.x)
        out.twist.twist.angular.y = float(msg.twist.twist.angular.y)
        out.twist.twist.angular.z = float(msg.twist.twist.angular.z + random.gauss(0.0, yaw_rate_std))

        # Covariances: keep original (or zero). Users can override downstream if needed.
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNoisyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

