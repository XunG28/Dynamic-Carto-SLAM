#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomToTfNode(Node):
    """Broadcast TF from an Odometry topic: frame_id -> child_frame_id."""

    def __init__(self) -> None:
        super().__init__('odom_to_tf')

        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('publish_topic', '/tf')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        # Subscribe BEST_EFFORT (bag replay / simulators often use it); TF publish RELIABLE.
        sub_qos = QoSProfile(depth=200)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = DurabilityPolicy.VOLATILE

        self._tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, in_topic, self._cb, sub_qos)

        self.get_logger().info(f'OdomToTf: {in_topic} -> TF ({self.get_parameter("publish_topic").value})')

    def _cb(self, msg: Odometry) -> None:
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = float(msg.pose.pose.position.x)
        tf.transform.translation.y = float(msg.pose.pose.position.y)
        tf.transform.translation.z = float(msg.pose.pose.position.z)
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToTfNode()
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

