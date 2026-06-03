#!/usr/bin/env python3

import math
import os
import sys
import traceback
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf_transformations import quaternion_matrix

import tf2_ros


Point3 = Tuple[float, float, float]


def _default_planner_root() -> str:
    env_root = os.environ.get("PCT_PLANNER_ROOT")
    if env_root:
        return env_root

    workspace_root = Path(__file__).resolve().parents[3]
    candidate = workspace_root / "src" / "global_path_planning"
    return str(candidate)


def _is_finite_point(point: Point3) -> bool:
    return all(math.isfinite(v) for v in point)


def _normalize_frame(frame_id: str) -> str:
    return (frame_id or "").lstrip("/")


class PctGlobalPlannerRos2(Node):
    """ROS2 wrapper for the vendored PCT tomogram planner."""

    def __init__(self) -> None:
        super().__init__("pct_global_planner")

        self.declare_parameter("planner_root", _default_planner_root())
        self.declare_parameter("tomogram_file", "output")
        self.declare_parameter("tomogram_dir", "/rsc/tomogram/")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("goal_topic", "/goal_3d")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("path_topic", "global_path")
        self.declare_parameter("pct_path_topic", "/pct_path")
        self.declare_parameter("status_topic", "pct_global_planner/status")
        self.declare_parameter("start_source", "tf")
        self.declare_parameter("start_z_offset", 0.0)
        self.declare_parameter("goal_z_offset", 0.0)
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("replan_on_start_update", False)

        self.planner_root = Path(str(self.get_parameter("planner_root").value)).expanduser()
        self.tomogram_file = str(self.get_parameter("tomogram_file").value)
        self.tomogram_dir = str(self.get_parameter("tomogram_dir").value)
        self.global_frame = _normalize_frame(str(self.get_parameter("global_frame").value)) or "map"
        self.base_frame = _normalize_frame(str(self.get_parameter("base_frame").value)) or "base_link"
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.pct_path_topic = str(self.get_parameter("pct_path_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.start_source = str(self.get_parameter("start_source").value).strip().lower()
        self.start_z_offset = float(self.get_parameter("start_z_offset").value)
        self.goal_z_offset = float(self.get_parameter("goal_z_offset").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.replan_on_start_update = bool(self.get_parameter("replan_on_start_update").value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(NavPath, self.path_topic, 1)
        self.pct_path_pub = (
            self.create_publisher(NavPath, self.pct_path_topic, 1)
            if self.pct_path_topic
            else None
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self._on_initialpose,
            10,
        )
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)

        self.manual_start: Optional[Point3] = None
        self.latest_goal: Optional[Point3] = None
        self.planner = self._load_planner()

        self.get_logger().info(
            "PCT global planner ready: "
            f"tomogram={self.tomogram_file}, start_source={self.start_source}, "
            f"path_topic={self.path_topic}, global_frame={self.global_frame}"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _load_planner(self):
        scripts_dir = self.planner_root / "planner" / "scripts"
        planner_dir = self.planner_root / "planner"
        lib_dir = self.planner_root / "planner" / "lib"
        tomogram_path = self.planner_root / self.tomogram_dir.lstrip("/") / f"{self.tomogram_file}.pickle"

        missing = []
        for path in (scripts_dir, planner_dir, lib_dir):
            if not path.exists():
                missing.append(str(path))
        if not tomogram_path.exists():
            missing.append(str(tomogram_path))
        if missing:
            detail = "\n  ".join(missing)
            raise FileNotFoundError(f"PCT planner input is missing:\n  {detail}")

        for path in (scripts_dir, planner_dir, lib_dir):
            path_str = str(path)
            if path_str not in sys.path:
                sys.path.insert(0, path_str)

        try:
            from config import Config  # type: ignore
            from planner_wrapper import TomogramPlanner  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                "Failed to import vendored PCT planner. Build it first with:\n"
                "  cd src/global_path_planning/planner\n"
                "  ./build_thirdparty.sh\n"
                "  ./build.sh\n"
                "and make sure the launch LD_LIBRARY_PATH includes the PCT planner lib paths."
            ) from exc

        cfg = Config()
        cfg.wrapper.tomo_dir = self.tomogram_dir
        planner = TomogramPlanner(cfg)
        planner.loadTomogram(self.tomogram_file)
        return planner

    def _lookup_tf(self, target: str, source: str):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(f"TF lookup failed: {target} <- {source}: {exc}")
            return None

    @staticmethod
    def _transform_point(point: Point3, tf_msg) -> Point3:
        trans = np.array(
            [
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z,
            ],
            dtype=np.float64,
        )
        q = tf_msg.transform.rotation
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float64)
        out = rot @ np.asarray(point, dtype=np.float64) + trans
        return (float(out[0]), float(out[1]), float(out[2]))

    def _pose_to_global_point(self, msg: PoseStamped, z_offset: float = 0.0) -> Optional[Point3]:
        src_frame = _normalize_frame(msg.header.frame_id) or self.global_frame
        point = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z + z_offset),
        )
        if not _is_finite_point(point):
            return None
        if src_frame == self.global_frame:
            return point
        tf_msg = self._lookup_tf(self.global_frame, src_frame)
        if tf_msg is None:
            return None
        return self._transform_point(point, tf_msg)

    def _current_start_from_tf(self) -> Optional[Point3]:
        tf_msg = self._lookup_tf(self.global_frame, self.base_frame)
        if tf_msg is None:
            return None
        point = (
            float(tf_msg.transform.translation.x),
            float(tf_msg.transform.translation.y),
            float(tf_msg.transform.translation.z + self.start_z_offset),
        )
        return point if _is_finite_point(point) else None

    def _select_start(self) -> Optional[Point3]:
        if self.start_source == "initialpose":
            return self.manual_start
        if self.start_source == "tf":
            return self._current_start_from_tf()
        if self.start_source == "initialpose_fallback":
            return self.manual_start or self._current_start_from_tf()
        if self.start_source == "tf_fallback":
            return self._current_start_from_tf() or self.manual_start
        self.get_logger().warn(
            f"Unknown start_source='{self.start_source}', expected tf/initialpose/tf_fallback/initialpose_fallback"
        )
        return self._current_start_from_tf() or self.manual_start

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        start = self._pose_to_global_point(pose, self.start_z_offset)
        if start is None:
            self._publish_status("initialpose rejected: transform or finite check failed")
            return
        self.manual_start = start
        self.get_logger().info(f"PCT start updated: [{start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}]")
        if self.replan_on_start_update and self.latest_goal is not None:
            self._try_plan()

    def _on_goal(self, msg: PoseStamped) -> None:
        goal = self._pose_to_global_point(msg, self.goal_z_offset)
        if goal is None:
            self._publish_empty_path()
            self._publish_status("goal rejected: transform or finite check failed")
            return
        self.latest_goal = goal
        self.get_logger().info(f"PCT goal updated: [{goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}]")
        self._try_plan()

    def _try_plan(self) -> None:
        start = self._select_start()
        goal = self.latest_goal
        if start is None or goal is None:
            self._publish_empty_path()
            self._publish_status("waiting for valid 3D start and goal")
            return

        try:
            traj_3d = self.planner.plan(
                np.asarray(start, dtype=np.float32),
                np.asarray(goal, dtype=np.float32),
            )
        except Exception:
            self._publish_empty_path()
            self.get_logger().error("PCT planning exception:\n" + traceback.format_exc())
            self._publish_status("planning exception")
            return

        if traj_3d is None or len(traj_3d) == 0:
            self._publish_empty_path()
            self._publish_status("planning failed: no path")
            return

        path_msg = self._traj_to_path(np.asarray(traj_3d, dtype=np.float64))
        self.path_pub.publish(path_msg)
        if self.pct_path_pub is not None:
            self.pct_path_pub.publish(path_msg)
        self._publish_status(f"published PCT path: {len(path_msg.poses)} points")

    def _publish_empty_path(self) -> None:
        msg = NavPath()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)
        if self.pct_path_pub is not None:
            self.pct_path_pub.publish(msg)

    def _traj_to_path(self, traj: np.ndarray) -> NavPath:
        msg = NavPath()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        headings = np.zeros((traj.shape[0],), dtype=np.float64)
        if traj.shape[0] >= 2:
            diffs = traj[1:, :2] - traj[:-1, :2]
            seg_yaws = np.arctan2(diffs[:, 1], diffs[:, 0])
            headings[:-1] = seg_yaws
            headings[-1] = seg_yaws[-1]

        for i in range(traj.shape[0]):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(traj[i, 0])
            ps.pose.position.y = float(traj[i, 1])
            ps.pose.position.z = float(traj[i, 2])
            half = float(headings[i]) * 0.5
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            msg.poses.append(ps)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PctGlobalPlannerRos2()
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
