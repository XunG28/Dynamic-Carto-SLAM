#!/usr/bin/env python3
"""LaserScan -> PointCloud2 for nodes (e.g. KISS-ICP) that only subscribe to PointCloud2."""

from __future__ import annotations

import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from laser_geometry.laser_geometry import LaserProjection
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class LaserscanToPointcloud2Node(Node):
    def __init__(self) -> None:
        super().__init__('laserscan_to_pointcloud2')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/points')

        self._proj = LaserProjection()
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 50)

        self.get_logger().info(f'{scan_topic} -> {cloud_topic} (PointCloud2)')

    @staticmethod
    def _sanitize_ranges(msg: LaserScan) -> LaserScan:
        """Replace NaN/inf/out-of-range with invalid sentinel so laser_geometry does not NaN-multiply."""
        out = copy.deepcopy(msg)
        rm = float(out.range_max)
        r0 = float(out.range_min)
        fixed = []
        for r in out.ranges:
            if math.isfinite(r) and r0 <= r <= rm:
                fixed.append(r)
            else:
                fixed.append(rm + 1.0)
        out.ranges = fixed
        return out

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            clean = self._sanitize_ranges(msg)
            cloud = self._proj.projectLaser(clean)
        except Exception as e:
            self.get_logger().warning(f'projectLaser failed: {e}')
            return
        self._pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaserscanToPointcloud2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
