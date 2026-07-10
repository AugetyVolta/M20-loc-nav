#!/usr/bin/env python3
"""Capture the latest ROS Path to JSON or replay it continuously."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser("capture", help="Save the latest received Path")
    capture.add_argument("--topic", default="/path_1")
    capture.add_argument("--output", required=True)
    capture.add_argument(
        "--min-distance",
        type=float,
        default=0.05,
        help="Minimum 3D distance in metres between saved poses",
    )

    publish = subparsers.add_parser("publish", help="Publish a saved Path repeatedly")
    publish.add_argument("--input", required=True)
    publish.add_argument("--topic", default="/pct_path")
    publish.add_argument("--rate", type=float, default=2.0)
    publish.add_argument(
        "--frame-id",
        default="",
        help="Override the frame stored in the JSON file",
    )
    return parser


def _pose_record(pose: PoseStamped) -> dict[str, float]:
    position = pose.pose.position
    orientation = pose.pose.orientation
    return {
        "x": float(position.x),
        "y": float(position.y),
        "z": float(position.z),
        "qx": float(orientation.x),
        "qy": float(orientation.y),
        "qz": float(orientation.z),
        "qw": float(orientation.w),
    }


def _downsample(poses: list[PoseStamped], min_distance: float) -> list[dict[str, float]]:
    if not poses:
        return []

    threshold = max(0.0, float(min_distance))
    selected = [_pose_record(poses[0])]
    last = poses[0].pose.position

    for pose in poses[1:-1]:
        current = pose.pose.position
        distance = math.sqrt(
            (current.x - last.x) ** 2
            + (current.y - last.y) ** 2
            + (current.z - last.z) ** 2
        )
        if distance >= threshold:
            selected.append(_pose_record(pose))
            last = current

    if len(poses) > 1:
        final_record = _pose_record(poses[-1])
        if final_record != selected[-1]:
            selected.append(final_record)
    return selected


class PathCapture(Node):
    def __init__(self, topic: str, output: Path, min_distance: float) -> None:
        super().__init__("path_snapshot_capture")
        self.output = output
        self.min_distance = min_distance
        self.latest: PathMsg | None = None
        self.create_subscription(PathMsg, topic, self._on_path, 10)
        self.get_logger().info(
            f"Capturing latest path from {topic}; press Ctrl+C after playback finishes"
        )

    def _on_path(self, msg: PathMsg) -> None:
        self.latest = msg

    def save(self) -> None:
        if self.latest is None or not self.latest.poses:
            raise RuntimeError("No non-empty Path message was received")

        records = _downsample(self.latest.poses, self.min_distance)
        payload = {
            "frame_id": self.latest.header.frame_id,
            "source_pose_count": len(self.latest.poses),
            "pose_count": len(records),
            "poses": records,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.get_logger().info(
            f"Saved {len(records)} poses from {len(self.latest.poses)} source poses "
            f"to {self.output}"
        )


class PathPublisher(Node):
    def __init__(self, input_path: Path, topic: str, rate: float, frame_id: str) -> None:
        super().__init__("path_snapshot_publisher")
        data: dict[str, Any] = json.loads(input_path.read_text(encoding="utf-8"))
        self.frame_id = frame_id or str(data.get("frame_id") or "camera_init")
        self.records = list(data.get("poses") or [])
        if not self.records:
            raise RuntimeError(f"No poses found in {input_path}")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PathMsg, topic, qos)
        self.timer = self.create_timer(1.0 / max(0.1, rate), self._publish)
        self.get_logger().info(
            f"Publishing {len(self.records)} poses from {input_path} "
            f"to {topic} in frame {self.frame_id}"
        )

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = PathMsg()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp

        for record in self.records:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(record["x"])
            pose.pose.position.y = float(record["y"])
            pose.pose.position.z = float(record["z"])
            pose.pose.orientation.x = float(record["qx"])
            pose.pose.orientation.y = float(record["qy"])
            pose.pose.orientation.z = float(record["qz"])
            pose.pose.orientation.w = float(record["qw"])
            msg.poses.append(pose)

        self.publisher.publish(msg)


def main() -> int:
    args = _parser().parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node: Node | None = None
    exit_code = 0

    try:
        if args.command == "capture":
            capture = PathCapture(
                topic=args.topic,
                output=Path(args.output).expanduser().resolve(),
                min_distance=args.min_distance,
            )
            node = capture
            try:
                rclpy.spin(capture)
            except KeyboardInterrupt:
                pass
            capture.save()
        else:
            node = PathPublisher(
                input_path=Path(args.input).expanduser().resolve(),
                topic=args.topic,
                rate=args.rate,
                frame_id=args.frame_id,
            )
            rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, OSError, ValueError, KeyError) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print(f"path_snapshot_tool: {exc}", file=sys.stderr)
            exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
