#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class PctPathAdapter(Node):
    """Relay an external PCT 3D path into the existing ROS2 global_path topic."""

    def __init__(self) -> None:
        super().__init__("pct_path_adapter")

        self.declare_parameter("pct_path_topic", "/pct_path")
        self.declare_parameter("global_path_topic", "global_path")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("force_output_frame", False)
        self.declare_parameter("restamp", True)
        self.declare_parameter("min_path_points", 2)
        self.declare_parameter("republish_hz", 2.0)
        self.declare_parameter("status_topic", "pct_path_adapter/status")

        self.pct_path_topic = str(self.get_parameter("pct_path_topic").value)
        self.global_path_topic = str(self.get_parameter("global_path_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.force_output_frame = bool(self.get_parameter("force_output_frame").value)
        self.restamp = bool(self.get_parameter("restamp").value)
        self.min_path_points = int(self.get_parameter("min_path_points").value)
        republish_hz = float(self.get_parameter("republish_hz").value)
        status_topic = str(self.get_parameter("status_topic").value)

        qos_path = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(Path, self.pct_path_topic, self._on_path, qos_path)
        self.pub = self.create_publisher(Path, self.global_path_topic, qos_path)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.latest_path: Optional[Path] = None
        self.latest_status = "waiting"
        if republish_hz > 0.0:
            self.timer = self.create_timer(1.0 / republish_hz, self._on_republish)
        else:
            self.timer = None

        self.get_logger().info(
            "PCT path adapter ready: "
            f"{self.pct_path_topic} -> {self.global_path_topic}, "
            f"output_frame={self.output_frame}, force_output_frame={self.force_output_frame}"
        )

    @staticmethod
    def _finite_pose(ps: PoseStamped) -> bool:
        p = ps.pose.position
        q = ps.pose.orientation
        return all(
            math.isfinite(v)
            for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        )

    def _publish_status(self, text: str) -> None:
        self.latest_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _empty_path(self, frame_id: str) -> Path:
        out = Path()
        out.header.frame_id = frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        return out

    def _adapt_path(self, msg: Path) -> Path:
        input_frame = msg.header.frame_id or self.output_frame or "map"
        output_frame = self.output_frame if self.force_output_frame else input_frame
        if not output_frame:
            output_frame = "map"

        out = Path()
        out.header.frame_id = output_frame
        out.header.stamp = self.get_clock().now().to_msg() if self.restamp else msg.header.stamp

        for pose_in in msg.poses:
            if not self._finite_pose(pose_in):
                continue
            pose_out = PoseStamped()
            pose_out.header = out.header
            pose_out.pose = pose_in.pose
            out.poses.append(pose_out)

        return out

    def _on_path(self, msg: Path) -> None:
        path = self._adapt_path(msg)
        if len(path.poses) < self.min_path_points:
            frame_id = path.header.frame_id or self.output_frame or "map"
            self.latest_path = self._empty_path(frame_id)
            self.pub.publish(self.latest_path)
            self._publish_status(
                f"rejected pct path: {len(path.poses)} valid points < {self.min_path_points}"
            )
            return

        self.latest_path = path
        self.pub.publish(path)
        self._publish_status(
            f"published pct path: {len(path.poses)} points, frame={path.header.frame_id}"
        )

    def _on_republish(self) -> None:
        if self.latest_path is None:
            self._publish_status(self.latest_status)
            return
        if self.restamp:
            self.latest_path.header.stamp = self.get_clock().now().to_msg()
            for pose in self.latest_path.poses:
                pose.header = self.latest_path.header
        self.pub.publish(self.latest_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PctPathAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
