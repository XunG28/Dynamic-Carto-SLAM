# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import os

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GazeboGroundTruthNode(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_ground_truth_publisher')

        default_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
        self.declare_parameter('model_name', default_model)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('output_frame', 'world')
        self.declare_parameter('child_frame', 'base_footprint')

        self._model_name = self.get_parameter('model_name').get_parameter_value().string_value
        rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self._frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self._child = self.get_parameter('child_frame').get_parameter_value().string_value

        self._latest_pose = None
        self._latest_twist = None
        self._have_warned_missing = False

        self.create_subscription(ModelStates, '/gazebo/model_states', self._on_states, 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/eval/ground_truth/pose', 10)
        self._pub_odom = self.create_publisher(Odometry, '/eval/ground_truth/odom', 10)

        period = 1.0 / rate if rate > 0.0 else 0.05
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Ground truth: model="{self._model_name}", frame="{self._frame}", child="{self._child}"'
        )

    def _on_states(self, msg: ModelStates) -> None:
        try:
            i = msg.name.index(self._model_name)
        except ValueError:
            if not self._have_warned_missing:
                self.get_logger().warn(
                    f'Model "{self._model_name}" not in /gazebo/model_states yet '
                    f'(names sample: {list(msg.name)[:5]}...)'
                )
                self._have_warned_missing = True
            return
        self._have_warned_missing = False
        self._latest_pose = msg.pose[i]
        self._latest_twist = msg.twist[i]

    def _publish(self) -> None:
        if self._latest_pose is None:
            return
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self._frame
        pose_msg.pose = self._latest_pose
        self._pub_pose.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._frame
        odom_msg.child_frame_id = self._child
        odom_msg.pose.pose = self._latest_pose
        if self._latest_twist is not None:
            odom_msg.twist.twist = self._latest_twist
        self._pub_odom.publish(odom_msg)


def main() -> None:
    rclpy.init()
    node = GazeboGroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

