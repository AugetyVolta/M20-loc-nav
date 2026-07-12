#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from traj_utils.msg import Bspline
import tf2_ros


Point3 = Tuple[float, float, float]


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def de_boor(order: int, knots: List[float], points: List[Point3], t: float) -> Point3:
    degree = max(1, int(order))
    n = len(points) - 1
    if n < 0:
        return (0.0, 0.0, 0.0)

    low = knots[degree] if len(knots) > degree else 0.0
    high_idx = n + 1
    high = knots[high_idx] if len(knots) > high_idx else low
    t = min(max(float(t), low), high)

    k = degree
    upper = min(high_idx, len(knots) - 1)
    for i in range(degree, upper):
        if knots[i] <= t < knots[i + 1]:
            k = i
            break
    if abs(t - high) < 1e-9:
        k = min(n, high_idx - 1)

    d = []
    for j in range(degree + 1):
        idx = min(max(k - degree + j, 0), n)
        d.append([points[idx][0], points[idx][1], points[idx][2]])

    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            left_idx = k - degree + j
            right_idx = k + 1 + j - r
            denom = knots[right_idx] - knots[left_idx] if right_idx < len(knots) else 0.0
            alpha = 0.0 if abs(denom) < 1e-12 else (t - knots[left_idx]) / denom
            d[j][0] = (1.0 - alpha) * d[j - 1][0] + alpha * d[j][0]
            d[j][1] = (1.0 - alpha) * d[j - 1][1] + alpha * d[j][1]
            d[j][2] = (1.0 - alpha) * d[j - 1][2] + alpha * d[j][2]

    return (d[degree][0], d[degree][1], d[degree][2])


def apply_tf(point: Point3, tf_msg: TransformStamped) -> Point3:
    tx = tf_msg.transform.translation.x
    ty = tf_msg.transform.translation.y
    tz = tf_msg.transform.translation.z
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    # Quaternion rotation expanded for one point.
    px, py, pz = point
    tx2 = 2.0 * (y * pz - z * py)
    ty2 = 2.0 * (z * px - x * pz)
    tz2 = 2.0 * (x * py - y * px)
    rx = px + w * tx2 + (y * tz2 - z * ty2)
    ry = py + w * ty2 + (z * tx2 - x * tz2)
    rz = pz + w * tz2 + (x * ty2 - y * tx2)
    return (rx + tx, ry + ty, rz + tz)


class EgoBsplineToPath(Node):
    def __init__(self):
        super().__init__('ego_bspline_to_path')

        self.declare_parameter('bspline_topic', '/drone_0_planning/bspline')
        self.declare_parameter('output_path_topic', '/ego_local_path')
        self.declare_parameter('output_frame', 'odom_body')
        self.declare_parameter('source_frame', 'odom_body')
        self.declare_parameter('sample_dt', 0.10)
        self.declare_parameter('max_horizon_sec', 3.0)
        self.declare_parameter('min_path_points', 4)
        self.declare_parameter('path_timeout', 1.0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('tf_timeout', 0.03)

        self.bspline_topic = str(self.get_parameter('bspline_topic').value)
        self.output_path_topic = str(self.get_parameter('output_path_topic').value)
        self.output_frame = str(self.get_parameter('output_frame').value).lstrip('/')
        self.source_frame = str(self.get_parameter('source_frame').value).lstrip('/')
        self.sample_dt = float(self.get_parameter('sample_dt').value)
        self.max_horizon_sec = float(self.get_parameter('max_horizon_sec').value)
        self.min_path_points = int(self.get_parameter('min_path_points').value)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, self.output_path_topic, 10)
        self.bspline_sub = self.create_subscription(Bspline, self.bspline_topic, self._on_bspline, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, publish_rate), self._on_timer)

        self.latest_msg: Optional[Bspline] = None
        self.latest_receive_time = None
        self.get_logger().info(
            f'ego_bspline_to_path: {self.bspline_topic} -> {self.output_path_topic}, '
            f'{self.source_frame}->{self.output_frame}'
        )

    def _on_bspline(self, msg: Bspline):
        self.latest_msg = msg
        self.latest_receive_time = self.get_clock().now()

    def _lookup_tf(self):
        if self.source_frame == self.output_frame:
            return None
        try:
            return self.tf_buffer.lookup_transform(
                self.output_frame,
                self.source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(
                f'TF lookup failed: {self.output_frame} <- {self.source_frame}: {exc}'
            )
            return False

    def _publish_empty(self):
        msg = Path()
        msg.header.frame_id = self.output_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)

    def _on_timer(self):
        if self.latest_msg is None or self.latest_receive_time is None:
            return
        age = (self.get_clock().now() - self.latest_receive_time).nanoseconds * 1e-9
        if age > self.path_timeout:
            self._publish_empty()
            return

        msg = self.latest_msg
        points = [(p.x, p.y, p.z) for p in msg.pos_pts]
        if len(points) < max(2, msg.order + 1) or len(msg.knots) < 2:
            self._publish_empty()
            return

        degree = max(1, int(msg.order))
        t0 = msg.knots[degree]
        t1 = msg.knots[len(points)]
        if not math.isfinite(t0) or not math.isfinite(t1) or t1 <= t0:
            self._publish_empty()
            return
        t1 = min(t1, t0 + max(0.1, self.max_horizon_sec))

        tf_msg = self._lookup_tf()
        if tf_msg is False:
            return

        samples = []
        t = t0
        dt = max(0.02, self.sample_dt)
        while t <= t1 + 1e-9:
            pt = de_boor(degree, list(msg.knots), points, t)
            if tf_msg is not None:
                pt = apply_tf(pt, tf_msg)
            samples.append(pt)
            t += dt

        if len(samples) < self.min_path_points:
            self._publish_empty()
            return

        path = Path()
        path.header.frame_id = self.output_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for idx, pt in enumerate(samples):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = float(pt[2])
            if idx + 1 < len(samples):
                nxt = samples[idx + 1]
            else:
                nxt = samples[idx]
            yaw = math.atan2(nxt[1] - pt[1], nxt[0] - pt[0])
            qx, qy, qz, qw = yaw_to_quat(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = EgoBsplineToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
