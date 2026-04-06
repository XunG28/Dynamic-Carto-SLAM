#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


@dataclass(frozen=True)
class _T:
    # Rigid transform: translation + unit quaternion (x,y,z,w)
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _q_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _q_mul(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_norm(q: Tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    return math.sqrt(x * x + y * y + z * z + w * w)


def _q_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    n = _q_norm(q)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = q
    return (x / n, y / n, z / n, w / n)


def _rotate_vec(q: Tuple[float, float, float, float], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    # v' = q * (v,0) * q_conj
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    qc = _q_conj(q)
    r = _q_mul(_q_mul(q, qv), qc)
    return (r[0], r[1], r[2])


def _inv(t: _T) -> _T:
    q = _q_normalize((t.qx, t.qy, t.qz, t.qw))
    qc = _q_conj(q)
    rx, ry, rz = _rotate_vec(qc, (-t.x, -t.y, -t.z))
    return _T(rx, ry, rz, qc[0], qc[1], qc[2], qc[3])


def _mul(a: _T, b: _T) -> _T:
    qa = _q_normalize((a.qx, a.qy, a.qz, a.qw))
    qb = _q_normalize((b.qx, b.qy, b.qz, b.qw))
    bxr, byr, bzr = _rotate_vec(qa, (b.x, b.y, b.z))
    q = _q_mul(qa, qb)
    q = _q_normalize(q)
    return _T(
        a.x + bxr,
        a.y + byr,
        a.z + bzr,
        q[0],
        q[1],
        q[2],
        q[3],
    )


def _t_from_odom(msg: Odometry) -> _T:
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    return _T(float(p.x), float(p.y), float(p.z), float(o.x), float(o.y), float(o.z), float(o.w))


class WorldOdomBridgeNode(Node):
    """
    Compute and broadcast:
      world -> odom
      world -> odom_noisy

    using ground-truth base pose in world (from /eval/ground_truth/odom)
    and odom-estimated base pose in odom frames (from /odom and /odom_noisy).

    Mathematics:
      T_world_odom = T_world_base_gt * inv(T_odom_base_est)
    """

    def __init__(self) -> None:
        super().__init__('world_odom_bridge')

        # mode:
        # - dynamic: update world->odom every time using latest (gt, est). Good for continuous frame alignment.
        # - static_init: compute world->odom once when both (gt, est) are first available, then keep fixed.
        self.declare_parameter('mode', 'dynamic')

        self.declare_parameter('gt_odom_topic', '/eval/ground_truth/odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_noisy_topic', '/odom_noisy')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_noisy_frame', 'odom_noisy')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('publish_odom_noisy_tf', True)

        self._tf = TransformBroadcaster(self)

        self._latest_gt: Optional[Odometry] = None
        self._latest_odom: Optional[Odometry] = None
        self._latest_noisy: Optional[Odometry] = None
        self._static_world_odom: Optional[_T] = None
        self._static_world_odom_noisy: Optional[_T] = None

        self.create_subscription(Odometry, self._p('gt_odom_topic'), self._on_gt, 50)
        self.create_subscription(Odometry, self._p('odom_topic'), self._on_odom, 200)
        self.create_subscription(Odometry, self._p('odom_noisy_topic'), self._on_noisy, 200)

        self.get_logger().info(
            f'WorldOdomBridge(mode={self._mode()}): gt={self._p("gt_odom_topic")}, '
            f'odom={self._p("odom_topic")}, noisy={self._p("odom_noisy_topic")}'
        )

    def _p(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _mode(self) -> str:
        m = self.get_parameter('mode').get_parameter_value().string_value.strip().lower()
        return m if m else 'dynamic'

    def _on_gt(self, msg: Odometry) -> None:
        self._latest_gt = msg
        self._try_publish_from_gt()

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg
        self._try_publish_from_odom()

    def _on_noisy(self, msg: Odometry) -> None:
        self._latest_noisy = msg
        self._try_publish_from_noisy()

    def _try_publish_from_odom(self) -> None:
        if self._latest_gt is None or self._latest_odom is None:
            return
        if not self.get_parameter('publish_odom_tf').get_parameter_value().bool_value:
            return
        child = self.get_parameter('odom_frame').get_parameter_value().string_value
        stamp = self._latest_odom.header.stamp
        if self._mode() == 'static_init':
            if self._static_world_odom is None:
                self._static_world_odom = self._compute_world_to_frame(gt=self._latest_gt, est=self._latest_odom)
            self._send_tf(t=self._static_world_odom, child_frame=child, stamp=stamp)
            return
        self._publish_world_to_frame(gt=self._latest_gt, est=self._latest_odom, child_frame=child, stamp=stamp)

    def _try_publish_from_noisy(self) -> None:
        if self._latest_gt is None or self._latest_noisy is None:
            return
        if not self.get_parameter('publish_odom_noisy_tf').get_parameter_value().bool_value:
            return
        child = self.get_parameter('odom_noisy_frame').get_parameter_value().string_value
        stamp = self._latest_noisy.header.stamp
        if self._mode() == 'static_init':
            if self._static_world_odom_noisy is None:
                self._static_world_odom_noisy = self._compute_world_to_frame(gt=self._latest_gt, est=self._latest_noisy)
            self._send_tf(t=self._static_world_odom_noisy, child_frame=child, stamp=stamp)
            return
        self._publish_world_to_frame(gt=self._latest_gt, est=self._latest_noisy, child_frame=child, stamp=stamp)

    def _try_publish_from_gt(self) -> None:
        # When GT updates, republish both using the most recent estimates.
        self._try_publish_from_odom()
        self._try_publish_from_noisy()

    def _compute_world_to_frame(self, *, gt: Odometry, est: Odometry) -> _T:
        t_world_base = _t_from_odom(gt)
        t_est_base = _t_from_odom(est)
        return _mul(t_world_base, _inv(t_est_base))

    def _send_tf(self, *, t: _T, child_frame: str, stamp) -> None:
        world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = world_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = float(t.x)
        tf.transform.translation.y = float(t.y)
        tf.transform.translation.z = float(t.z)
        tf.transform.rotation.x = float(t.qx)
        tf.transform.rotation.y = float(t.qy)
        tf.transform.rotation.z = float(t.qz)
        tf.transform.rotation.w = float(t.qw)
        self._tf.sendTransform(tf)

    def _publish_world_to_frame(self, *, gt: Odometry, est: Odometry, child_frame: str, stamp) -> None:
        self._send_tf(
            t=self._compute_world_to_frame(gt=gt, est=est),
            child_frame=child_frame,
            stamp=stamp,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldOdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

