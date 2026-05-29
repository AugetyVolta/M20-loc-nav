#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Publish Nav2 global paths for a sequence of map-frame goals.

This node keeps the original contract of publishing nav_msgs/Path on
`global_path`, but it can now execute multiple terminal goals in order:

  1. Ask Nav2 planner_server /compute_path_to_pose for the current goal.
  2. Publish the returned path to `global_path`.
  3. Watch robot pose from TF.
  4. Once the robot is within `goal_tolerance`, advance to the next goal.

It does not send velocity commands by itself. Your RL/PRIEST local planner
continues to consume `global_path`, generate `local_path`, and the adapter
continues to command the robot.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0  # radians


class GlobalPathSequencePublisher(Node):
    def __init__(self):
        super().__init__("global_path_sequence_publisher")

        # Main behavior.
        self.declare_parameter("goals", "")
        self.declare_parameter("goals_xy", [14.9616, -39.4128, -27.6913, -43.1581, -29.8596, -2.77471, 18.6057, 4.53992])
        self.declare_parameter("goal_yaw_unit", "deg")  # deg / rad, only for `goals` third column
        self.declare_parameter("goal_tolerance", 0.6)
        self.declare_parameter("replan_period", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("auto_advance", True)
        self.declare_parameter("start_index", 0)

        # Frames/topics.
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("path_topic", "global_path")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("waypoints_topic", "waypoints")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("tf_timeout", 0.2)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.replan_period = max(0.1, float(self.get_parameter("replan_period").value))
        self.loop = bool(self.get_parameter("loop").value)
        self.auto_advance = bool(self.get_parameter("auto_advance").value)

        self.waypoints = self._load_waypoints()
        self.current_index = int(self.get_parameter("start_index").value)
        if self.waypoints:
            self.current_index = int(np.clip(self.current_index, 0, len(self.waypoints) - 1))
        self.sequence_done = False
        self.request_in_flight = False
        self._last_path: Optional[Path] = None

        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.path_pub = self.create_publisher(Path, self.path_topic, 20)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.waypoints_pub = self.create_publisher(PoseArray, self.waypoints_topic, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.replan_period, self._on_timer)
        self.marker_timer = self.create_timer(1.0, self._publish_waypoints)

        self.get_logger().warn(
            "[global_path_sequence] ready\n"
            f"  goals={[(round(w.x, 3), round(w.y, 3), round(w.yaw, 3)) for w in self.waypoints]}\n"
            f"  path_topic={self.path_topic}, goal_topic={self.goal_topic}, waypoints_topic={self.waypoints_topic}\n"
            f"  frame={self.global_frame}, robot_frame={self.robot_frame}\n"
            f"  tolerance={self.goal_tolerance:.2f} m, replan_period={self.replan_period:.2f} s, "
            f"loop={self.loop}, auto_advance={self.auto_advance}"
        )

    def _load_waypoints(self) -> List[Waypoint]:
        goals_text = str(self.get_parameter("goals").value or "").strip()
        yaw_unit = str(self.get_parameter("goal_yaw_unit").value or "deg").lower()
        yaw_scale = math.pi / 180.0 if yaw_unit.startswith("deg") else 1.0

        if goals_text:
            return self._parse_goals_string(goals_text, yaw_scale)

        flat = list(self.get_parameter("goals_xy").value)
        if len(flat) % 2 != 0:
            raise ValueError("goals_xy must be a flat [x1, y1, x2, y2, ...] list")
        if len(flat) == 0:
            return []
        return [
            Waypoint(float(flat[i]), float(flat[i + 1]), 0.0)
            for i in range(0, len(flat), 2)
        ]

    @staticmethod
    def _parse_goals_string(value: str, yaw_scale: float) -> List[Waypoint]:
        """
        Parse examples:
          "1.0,2.0; 3.0,4.0; 5.0,6.0"
          "1.0,2.0,90; 3.0,4.0,180"

        Third column is yaw, default unit configured by `goal_yaw_unit`.
        """
        waypoints: List[Waypoint] = []
        for chunk in re.split(r"[;\n]+", value):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [p for p in re.split(r"[,\s]+", chunk) if p]
            if len(parts) not in (2, 3):
                raise ValueError(f"Invalid goal '{chunk}', expected x,y or x,y,yaw")
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) * yaw_scale if len(parts) == 3 else 0.0
            waypoints.append(Waypoint(x, y, yaw))
        return waypoints

    def _on_timer(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints configured.")
            return
        if self.sequence_done:
            return

        if self.auto_advance:
            self._advance_if_reached()
            if self.sequence_done:
                return

        if self.request_in_flight:
            return
        if not self.compute_path_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("ComputePathToPose server not available yet.")
            return

        self._request_plan_to_current_goal()

    def _advance_if_reached(self):
        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            return

        goal = self.waypoints[self.current_index]
        dist = math.hypot(robot_xy[0] - goal.x, robot_xy[1] - goal.y)
        if dist > self.goal_tolerance:
            return

        self.get_logger().warn(
            f"Reached waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"({goal.x:.3f}, {goal.y:.3f}), dist={dist:.3f} m"
        )

        if self.current_index + 1 < len(self.waypoints):
            self.current_index += 1
            next_goal = self.waypoints[self.current_index]
            self.get_logger().warn(
                f"Advance to waypoint {self.current_index + 1}/{len(self.waypoints)}: "
                f"({next_goal.x:.3f}, {next_goal.y:.3f})"
            )
            self._last_path = None
            return

        if self.loop:
            self.current_index = 0
            self._last_path = None
            self.get_logger().warn("Waypoint sequence completed; looping to waypoint 1.")
        else:
            self.sequence_done = True
            self.get_logger().warn("Waypoint sequence completed.")

    def _request_plan_to_current_goal(self):
        goal = self.waypoints[self.current_index]
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._to_pose_stamped(goal)
        goal_msg.use_start = False
        if self.planner_id:
            goal_msg.planner_id = self.planner_id

        self.request_in_flight = True
        self._publish_current_goal()
        self.get_logger().info(
            f"Planning to waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"({goal.x:.3f}, {goal.y:.3f})"
        )

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_in_flight = False
            self.get_logger().error(f"ComputePathToPose send failed: {exc}")
            return

        if not goal_handle.accepted:
            self.request_in_flight = False
            self.get_logger().error("ComputePathToPose goal rejected by server")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        self.request_in_flight = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"ComputePathToPose result failed: {exc}")
            return

        if result is None or result.path is None or len(result.path.poses) == 0:
            self.get_logger().error("No valid path returned")
            return

        path_msg = result.path
        path_msg.header.frame_id = self.global_frame
        self._last_path = path_msg
        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Published global path to waypoint {self.current_index + 1}/{len(self.waypoints)} "
            f"with {len(path_msg.poses)} poses"
        )

    def _lookup_robot_xy(self) -> Optional[np.ndarray]:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {self.global_frame} <- {self.robot_frame}: {exc}")
            return None

        return np.array(
            [trans.transform.translation.x, trans.transform.translation.y],
            dtype=np.float64,
        )

    def _to_pose_stamped(self, waypoint: Waypoint) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(waypoint.x)
        msg.pose.position.y = float(waypoint.y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        msg.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return msg

    def _publish_current_goal(self):
        if not self.waypoints or self.sequence_done:
            return
        self.goal_pub.publish(self._to_pose_stamped(self.waypoints[self.current_index]))

    def _publish_waypoints(self):
        if not self.waypoints:
            return
        arr = PoseArray()
        arr.header.frame_id = self.global_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            ps = self._to_pose_stamped(wp)
            arr.poses.append(ps.pose)
        self.waypoints_pub.publish(arr)
        self._publish_current_goal()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathSequencePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

