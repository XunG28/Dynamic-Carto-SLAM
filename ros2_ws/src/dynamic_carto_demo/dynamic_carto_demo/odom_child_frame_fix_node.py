#!/usr/bin/env python3
"""Republish Odometry with standard frame ids for nodes that expect TB3 TF (odom -> base_footprint)."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OdomChildFrameFixNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_child_frame_fix')

        self.declare_parameter('input_topic', '/odom_noisy')
        self.declare_parameter('output_topic', '/odom_noisy_carto')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        sub_qos = QoSProfile(depth=50)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = DurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=50)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.VOLATILE

        self._pub = self.create_publisher(Odometry, out_topic, pub_qos)
        self.create_subscription(Odometry, in_topic, self._cb, sub_qos)
        self.get_logger().info(f'{in_topic} -> {out_topic} (frame={self._frame_id}, child={self._child_frame_id})')

    def _cb(self, msg: Odometry) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        out.child_frame_id = self._child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomChildFrameFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # Launch may already have shut down the global context; avoid RCLError on double shutdown.
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
