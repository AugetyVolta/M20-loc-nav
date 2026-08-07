#!/usr/bin/env python3

"""Manage an ordered queue of 3D goals for the PCT planner."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseArray, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, String
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0
    marker_id: int = -1


class SimTimeSafeInteractiveMarkerServer(InteractiveMarkerServer):
    """Accept the first RViz feedback while simulated time is still zero."""

    def processFeedback(self, feedback):
        if self.node.get_clock().now().nanoseconds == 0:
            with self.mutex:
                marker_context = self.marker_contexts.get(feedback.marker_name)
                if marker_context is not None:
                    marker_context.last_client_id = feedback.client_id
        super().processFeedback(feedback)


class GlobalPathSequencePublisher(Node):
    def __init__(self):
        super().__init__("global_path_sequence_publisher")

        self.declare_parameter("use_static_goals", False)
        self.declare_parameter("goals_xyz", [])
        self.declare_parameter("goal_tolerance", 1.0)
        self.declare_parameter("replan_period", 0.2)
        self.declare_parameter("loop", False)
        self.declare_parameter("auto_advance", True)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("auto_start_on_click", True)
        self.declare_parameter("min_clicked_spacing", 0.25)

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("robot_ground_offset", 0.45)
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_cancel_topic", "/pct/cancel_goal")
        self.declare_parameter("waypoints_topic", "/waypoints")
        self.declare_parameter("waypoints_pose_topic", "/waypoints_pose_array")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("delete_clicked_point_topic", "/waypoint_sequence/delete_nearest")
        self.declare_parameter("replace_clicked_point_topic", "/waypoint_sequence/replace_nearest")
        self.declare_parameter("status_topic", "/waypoint_sequence/status")
        self.declare_parameter("clear_topic", "/waypoint_sequence/clear")
        self.declare_parameter("undo_topic", "/waypoint_sequence/undo")
        self.declare_parameter("pause_topic", "/waypoint_sequence/pause")
        self.declare_parameter("resume_topic", "/waypoint_sequence/resume")
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("edit_radius", 1.0)

        self.declare_parameter("marker_point_diameter", 0.4)
        self.declare_parameter("marker_label_height", 0.35)
        self.declare_parameter("marker_z_offset", 0.2)
        self.declare_parameter("enable_interactive_markers", True)
        self.declare_parameter("interactive_marker_namespace", "waypoint_editor")

        self.global_frame = str(self.get_parameter("global_frame").value).lstrip("/")
        self.robot_frame = str(self.get_parameter("robot_frame").value).lstrip("/")
        self.robot_ground_offset = max(
            0.0, float(self.get_parameter("robot_ground_offset").value)
        )
        self.goal_tolerance = max(0.05, float(self.get_parameter("goal_tolerance").value))
        self.replan_period = max(0.2, float(self.get_parameter("replan_period").value))
        self.loop = bool(self.get_parameter("loop").value)
        self.auto_advance = bool(self.get_parameter("auto_advance").value)
        self.auto_start_on_click = bool(self.get_parameter("auto_start_on_click").value)
        self.min_clicked_spacing = max(
            0.0, float(self.get_parameter("min_clicked_spacing").value)
        )
        self.tf_timeout = max(0.01, float(self.get_parameter("tf_timeout").value))
        self.edit_radius = max(0.05, float(self.get_parameter("edit_radius").value))

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.goal_cancel_topic = str(self.get_parameter("goal_cancel_topic").value)
        self.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.waypoints_pose_topic = str(self.get_parameter("waypoints_pose_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.delete_clicked_point_topic = str(
            self.get_parameter("delete_clicked_point_topic").value
        )
        self.replace_clicked_point_topic = str(
            self.get_parameter("replace_clicked_point_topic").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.clear_topic = str(self.get_parameter("clear_topic").value)
        self.undo_topic = str(self.get_parameter("undo_topic").value)
        self.pause_topic = str(self.get_parameter("pause_topic").value)
        self.resume_topic = str(self.get_parameter("resume_topic").value)

        self.marker_point_diameter = max(
            0.05, float(self.get_parameter("marker_point_diameter").value)
        )
        self.marker_label_height = max(
            0.1, float(self.get_parameter("marker_label_height").value)
        )
        self.marker_z_offset = float(self.get_parameter("marker_z_offset").value)
        self.enable_interactive_markers = bool(
            self.get_parameter("enable_interactive_markers").value
        )
        self.interactive_marker_namespace = str(
            self.get_parameter("interactive_marker_namespace").value
        )

        self._next_marker_id = 0
        self.waypoints = self._load_static_waypoints()
        start_index = int(self.get_parameter("start_index").value)
        self.current_index = min(max(0, start_index), max(0, len(self.waypoints) - 1))
        self.sequence_done = False
        self.paused = False
        self._interactive_markers_dirty = True
        self._interactive_resync_remaining = 0
        self._interactive_drag_active = False
        self._interactive_drag_origin: Optional[tuple[int, Waypoint]] = None
        self._pending_menu_action: Optional[tuple[str, int]] = None
        self._menu_action_timer = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ui_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.goal_cancel_pub = self.create_publisher(Empty, self.goal_cancel_topic, 10)
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, self.waypoints_topic, 10
        )
        self.waypoints_pose_pub = self.create_publisher(
            PoseArray, self.waypoints_pose_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(
            PointStamped, self.clicked_point_topic, self._on_clicked_point, 10
        )
        self.create_subscription(
            PointStamped,
            self.delete_clicked_point_topic,
            self._on_delete_clicked_point,
            10,
        )
        self.create_subscription(
            PointStamped,
            self.replace_clicked_point_topic,
            self._on_replace_clicked_point,
            10,
        )
        self.create_subscription(Empty, self.clear_topic, self._on_clear, 10)
        self.create_subscription(Empty, self.undo_topic, self._on_undo, 10)
        self.create_subscription(Empty, self.pause_topic, self._on_pause, 10)
        self.create_subscription(Empty, self.resume_topic, self._on_resume, 10)

        self.marker_server = None
        self.menu_handler = None
        self.delete_menu_entry = None
        self.clear_menu_entry = None
        if self.enable_interactive_markers:
            self.marker_server = SimTimeSafeInteractiveMarkerServer(
                self, self.interactive_marker_namespace
            )
            self.menu_handler = MenuHandler()
            self.delete_menu_entry = self.menu_handler.insert(
                "Delete this waypoint", callback=self._on_interactive_menu
            )
            self.clear_menu_entry = self.menu_handler.insert(
                "Clear all waypoints", callback=self._on_interactive_menu
            )

        self.goal_timer = self.create_timer(self.replan_period, self._on_timer)
        self.marker_timer = self.create_timer(
            1.0,
            self._on_marker_timer,
            clock=self.ui_clock,
        )
        self.status_timer = self.create_timer(
            1.0,
            self._publish_status,
            clock=self.ui_clock,
        )

        self._publish_visualization()
        self.get_logger().info(
            "3D PCT waypoint queue ready: "
            f"click={self.clicked_point_topic}, goal={self.goal_topic}, "
            f"frame={self.global_frame}, robot={self.robot_frame}, "
            f"tolerance={self.goal_tolerance:.2f}m"
        )

    def _load_static_waypoints(self) -> List[Waypoint]:
        if not bool(self.get_parameter("use_static_goals").value):
            return []
        values = list(self.get_parameter("goals_xyz").value)
        if len(values) % 3 != 0:
            raise ValueError("goals_xyz must be [x1, y1, z1, x2, y2, z2, ...]")
        return [
            self._new_waypoint(
                float(values[i]), float(values[i + 1]), float(values[i + 2])
            )
            for i in range(0, len(values), 3)
        ]

    def _new_waypoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        waypoint = Waypoint(x, y, z, yaw, self._next_marker_id)
        self._next_marker_id += 1
        return waypoint

    def _on_timer(self):
        if not self.waypoints or self.paused or self.sequence_done:
            return
        if self.auto_advance and self._advance_if_reached():
            return
        self._publish_current_goal()

    def _on_marker_timer(self):
        if self._interactive_resync_remaining > 0 and not self._interactive_drag_active:
            self._interactive_resync_remaining -= 1
            self._interactive_markers_dirty = True
        self._publish_visualization()

    def _mark_interactive_markers_dirty(self, *, resync: bool = True):
        self._interactive_markers_dirty = True
        if resync:
            self._interactive_resync_remaining = 3

    def _advance_if_reached(self) -> bool:
        distance = self._current_goal_distance()
        if distance is None or distance > self.goal_tolerance:
            return False

        reached = self.waypoints[self.current_index]
        self.get_logger().info(
            f"Reached 3D waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"({reached.x:.2f}, {reached.y:.2f}, {reached.z:.2f}), "
            f"distance={distance:.2f}m"
        )

        if self.current_index + 1 < len(self.waypoints):
            self.current_index += 1
            self._mark_interactive_markers_dirty()
            self._publish_current_goal()
            self._publish_visualization()
            self._publish_status(
                f"reached waypoint {self.current_index}/{len(self.waypoints)}; "
                "stopped old path and requested the next plan"
            )
            return True

        if self.loop:
            self.current_index = 0
            self._mark_interactive_markers_dirty()
            self._publish_current_goal()
            self._publish_visualization()
            self._publish_status("loop restarted; stopped old path and requested waypoint 1")
            return True

        self._cancel_goal()
        self.sequence_done = True
        self._mark_interactive_markers_dirty()
        self._publish_visualization()
        self._publish_status("sequence completed; paths cleared and navigation stopped")
        return True

    def _on_clicked_point(self, msg: PointStamped):
        waypoint = self._point_to_waypoint(msg)
        if waypoint is None:
            return
        if (
            self.waypoints
            and self._distance_between(waypoint, self.waypoints[-1])
            < self.min_clicked_spacing
        ):
            return

        was_done = self.sequence_done
        self.waypoints.append(waypoint)
        if len(self.waypoints) == 1:
            self.current_index = 0
        elif was_done:
            self.current_index = len(self.waypoints) - 1
        self.sequence_done = False
        if self.auto_start_on_click:
            self.paused = False
        self._mark_interactive_markers_dirty()
        self._publish_current_goal()
        self._publish_visualization()
        self._publish_status(
            f"added waypoint {len(self.waypoints)}: "
            f"({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})"
        )

    def _on_delete_clicked_point(self, msg: PointStamped):
        waypoint = self._point_to_waypoint(msg)
        if waypoint is None:
            return
        nearest = self._nearest_waypoint(waypoint)
        if nearest is None:
            return
        index, distance = nearest
        if distance > self.edit_radius:
            self._publish_status(
                f"delete ignored: nearest waypoint is {distance:.2f}m away"
            )
            return
        self._delete_waypoint(index, f"deleted waypoint {index + 1}")

    def _delete_waypoint(self, index: int, detail: str):
        if index < 0 or index >= len(self.waypoints):
            return
        removed = self.waypoints.pop(index)
        self._retire_interactive_marker(removed)
        if not self.waypoints:
            self.current_index = 0
            self.sequence_done = True
            self._cancel_goal()
        else:
            if index < self.current_index:
                self.current_index -= 1
            elif index == self.current_index:
                self.current_index = min(self.current_index, len(self.waypoints) - 1)
            self.sequence_done = False
            self._publish_current_goal()
        self._mark_interactive_markers_dirty()
        self._publish_visualization()
        self._publish_status(
            f"{detail}: ({removed.x:.2f}, {removed.y:.2f}, {removed.z:.2f})"
        )

    def _on_replace_clicked_point(self, msg: PointStamped):
        replacement = self._point_to_waypoint(msg)
        if replacement is None:
            return
        nearest = self._nearest_waypoint(replacement)
        if nearest is None:
            return
        index, distance = nearest
        if distance > self.edit_radius:
            self._publish_status(
                f"replace ignored: nearest waypoint is {distance:.2f}m away"
            )
            return
        old = self.waypoints[index]
        self.waypoints[index] = Waypoint(
            replacement.x,
            replacement.y,
            replacement.z,
            old.yaw,
            old.marker_id,
        )
        self.sequence_done = False
        self._mark_interactive_markers_dirty()
        if index == self.current_index:
            self._publish_current_goal()
        self._publish_visualization()
        self._publish_status(
            f"replaced waypoint {index + 1}: "
            f"({old.x:.2f}, {old.y:.2f}, {old.z:.2f}) -> "
            f"({replacement.x:.2f}, {replacement.y:.2f}, {replacement.z:.2f})"
        )

    def _on_clear(self, _msg: Empty):
        for waypoint in self.waypoints:
            self._retire_interactive_marker(waypoint)
        self.waypoints.clear()
        self.current_index = 0
        self.sequence_done = True
        self.paused = False
        self._mark_interactive_markers_dirty()
        self._cancel_goal()
        self._publish_visualization()
        self._publish_status("cleared all waypoints")

    def _on_undo(self, _msg: Empty):
        if not self.waypoints:
            return
        removed_index = len(self.waypoints) - 1
        removed = self.waypoints.pop()
        self._retire_interactive_marker(removed)
        if not self.waypoints:
            self.current_index = 0
            self.sequence_done = True
            self._cancel_goal()
        else:
            self.current_index = min(self.current_index, len(self.waypoints) - 1)
            self.sequence_done = False
            self._publish_current_goal()
        self._mark_interactive_markers_dirty()
        self._publish_visualization()
        self._publish_status(
            f"undid waypoint {removed_index + 1}: "
            f"({removed.x:.2f}, {removed.y:.2f}, {removed.z:.2f})"
        )

    def _on_pause(self, _msg: Empty):
        if not self.waypoints:
            return
        self.paused = True
        self._cancel_goal()
        self._publish_status("paused and cancelled active PCT path")

    def _on_resume(self, _msg: Empty):
        if not self.waypoints:
            return
        self.paused = False
        self.sequence_done = False
        self._publish_current_goal()
        self._publish_status("resumed")

    def _publish_current_goal(self):
        if not self.waypoints or self.paused or self.sequence_done:
            return
        self.goal_pub.publish(self._to_pose_stamped(self.waypoints[self.current_index]))

    def _cancel_goal(self):
        self.goal_cancel_pub.publish(Empty())

    def _point_to_waypoint(self, msg: PointStamped) -> Optional[Waypoint]:
        source_frame = (msg.header.frame_id or self.global_frame).lstrip("/")
        point = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if source_frame and source_frame != self.global_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Cannot transform waypoint {source_frame}->{self.global_frame}: {exc}"
                )
                return None
            point = self._apply_transform(point, transform)
        return self._new_waypoint(point[0], point[1], point[2])

    def _lookup_robot_ground_position(self) -> Optional[tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(
                f"Waiting for TF {self.global_frame}->{self.robot_frame}: {exc}"
            )
            return None
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            float(translation.z) - self.robot_ground_offset,
        )

    def _current_goal_distance(self) -> Optional[float]:
        if not self.waypoints or self.sequence_done:
            return None
        robot = self._lookup_robot_ground_position()
        if robot is None:
            return None
        goal = self.waypoints[self.current_index]
        return math.sqrt(
            (robot[0] - goal.x) ** 2
            + (robot[1] - goal.y) ** 2
            + (robot[2] - goal.z) ** 2
        )

    def _nearest_waypoint(self, waypoint: Waypoint) -> Optional[tuple[int, float]]:
        if not self.waypoints:
            self._publish_status("edit ignored: no waypoints")
            return None
        distances = [self._distance_between(waypoint, item) for item in self.waypoints]
        index = min(range(len(distances)), key=distances.__getitem__)
        return index, distances[index]

    @staticmethod
    def _distance_between(first: Waypoint, second: Waypoint) -> float:
        return math.sqrt(
            (first.x - second.x) ** 2
            + (first.y - second.y) ** 2
            + (first.z - second.z) ** 2
        )

    @staticmethod
    def _apply_transform(point, transform_msg):
        transform = transform_msg.transform
        q = transform.rotation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1.0e-12:
            x = y = z = 0.0
            w = 1.0
        else:
            x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm

        px, py, pz = point
        rx = (
            (1.0 - 2.0 * (y * y + z * z)) * px
            + 2.0 * (x * y - z * w) * py
            + 2.0 * (x * z + y * w) * pz
        )
        ry = (
            2.0 * (x * y + z * w) * px
            + (1.0 - 2.0 * (x * x + z * z)) * py
            + 2.0 * (y * z - x * w) * pz
        )
        rz = (
            2.0 * (x * z - y * w) * px
            + 2.0 * (y * z + x * w) * py
            + (1.0 - 2.0 * (x * x + y * y)) * pz
        )
        return (
            rx + transform.translation.x,
            ry + transform.translation.y,
            rz + transform.translation.z,
        )

    def _to_pose_stamped(self, waypoint: Waypoint) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = waypoint.x
        msg.pose.position.y = waypoint.y
        msg.pose.position.z = waypoint.z
        msg.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        msg.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return msg

    def _publish_visualization(self, *, sync_interactive: bool = True):
        header_stamp = self.get_clock().now().to_msg()
        pose_array = PoseArray()
        pose_array.header.frame_id = self.global_frame
        pose_array.header.stamp = header_stamp
        for waypoint in self.waypoints:
            pose_array.poses.append(self._to_pose_stamped(waypoint).pose)
        self.waypoints_pose_pub.publish(pose_array)
        self.waypoint_marker_pub.publish(self._build_markers(pose_array.header))
        if sync_interactive:
            self._sync_interactive_markers(pose_array.header)

    def _build_markers(self, header) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        # The target sphere belongs exclusively to the interactive marker
        # server. Normal markers only provide waypoint numbers.
        for index, waypoint in enumerate(self.waypoints):
            label = Marker()
            label.header = header
            label.ns = "pct_waypoint_labels"
            label.id = index + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=waypoint.x,
                y=waypoint.y,
                z=waypoint.z
                + self.marker_z_offset
                + self.marker_point_diameter,
            )
            label.pose.orientation.w = 1.0
            label.scale.z = self.marker_label_height
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = self._interactive_label(index)
            result.markers.append(label)

        return result

    def _set_waypoint_color(self, marker: Marker, index: int):
        if self.sequence_done or index < self.current_index:
            marker.color.r = 0.2
            marker.color.g = 0.85
        elif index == self.current_index:
            # Match the original PCT end_pos marker exactly.
            marker.color.g = 0.25
            marker.color.b = 1.0
        else:
            marker.color.g = 0.55
            marker.color.b = 1.0

    def _sync_interactive_markers(self, header):
        if (
            self.marker_server is None
            or not self._interactive_markers_dirty
            or self._interactive_drag_active
        ):
            return
        if self.waypoints:
            for index, waypoint in enumerate(self.waypoints):
                marker = self._make_interactive_marker(index, waypoint, header)
                self.marker_server.insert(
                    marker, feedback_callback=self._on_interactive_feedback
                )
                self.menu_handler.apply(self.marker_server, marker.name)
        self.marker_server.applyChanges()
        self._interactive_markers_dirty = False

    @staticmethod
    def _interactive_marker_name(waypoint: Waypoint) -> str:
        return f"waypoint_{waypoint.marker_id}"

    def _retire_interactive_marker(self, waypoint: Waypoint):
        if self.marker_server is None:
            return
        name = self._interactive_marker_name(waypoint)
        marker = self.marker_server.get(name)
        if marker is None:
            return

        hidden_pose = marker.pose
        hidden_pose.position.z = -1000.0
        self.marker_server.setPose(name, hidden_pose)
        self.marker_server.setCallback(name, None)
        self.marker_server.setCallback(
            name,
            None,
            InteractiveMarkerFeedback.MENU_SELECT,
        )

    def _make_interactive_marker(self, index: int, waypoint: Waypoint, header):
        interactive = InteractiveMarker()
        interactive.header.frame_id = header.frame_id
        interactive.name = self._interactive_marker_name(waypoint)
        interactive.description = ""
        interactive.pose.position = Point(
            x=waypoint.x,
            y=waypoint.y,
            z=waypoint.z + self.marker_z_offset,
        )
        interactive.pose.orientation.w = 1.0
        interactive.scale = 1.0

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = self.marker_point_diameter
        sphere.scale.y = self.marker_point_diameter
        sphere.scale.z = self.marker_point_diameter
        sphere.color.a = 0.95
        self._set_waypoint_color(sphere, index)

        move_xy = InteractiveMarkerControl()
        move_xy.name = "move_xy"
        move_xy.description = "Drag in map XY plane; right-click for waypoint menu"
        move_xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_xy.orientation_mode = InteractiveMarkerControl.FIXED
        move_xy.always_visible = True
        self._set_orientation(move_xy, (1.0, 0.0, -1.0, 0.0))
        move_xy.markers.append(sphere)
        interactive.controls.append(move_xy)

        axes = (
            ("move_x", (1.0, 1.0, 0.0, 0.0)),
            ("move_y", (1.0, 0.0, 0.0, 1.0)),
            ("move_z", (1.0, 0.0, 1.0, 0.0)),
        )
        for name, orientation in axes:
            control = InteractiveMarkerControl()
            control.name = name
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation_mode = InteractiveMarkerControl.FIXED
            self._set_orientation(control, orientation)
            interactive.controls.append(control)

        return interactive

    @staticmethod
    def _set_orientation(control, values):
        w, x, y, z = values
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        control.orientation.w = w / norm
        control.orientation.x = x / norm
        control.orientation.y = y / norm
        control.orientation.z = z / norm

    def _on_interactive_feedback(self, feedback):
        index = self._interactive_waypoint_index(feedback.marker_name)
        if index is None or index >= len(self.waypoints):
            return
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self._on_interactive_menu(feedback)
            return
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self._interactive_drag_active = True
            waypoint = self.waypoints[index]
            self._interactive_drag_origin = (waypoint.marker_id, waypoint)
            return
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return

        old = self.waypoints[index]
        if (
            feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE
            and self._interactive_drag_origin is None
        ):
            self._interactive_drag_origin = (old.marker_id, old)
        updated = Waypoint(
            float(feedback.pose.position.x),
            float(feedback.pose.position.y),
            float(feedback.pose.position.z) - self.marker_z_offset,
            old.yaw,
            old.marker_id,
        )
        self.waypoints[index] = updated
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._interactive_drag_active = True
            self._publish_visualization(sync_interactive=False)
            return
        self._interactive_drag_active = False
        drag_origin = self._interactive_drag_origin
        self._interactive_drag_origin = None
        previous = old
        if drag_origin is not None and drag_origin[0] == old.marker_id:
            previous = drag_origin[1]
        moved_distance = math.sqrt(
            (updated.x - previous.x) ** 2
            + (updated.y - previous.y) ** 2
            + (updated.z - previous.z) ** 2
        )
        if moved_distance <= 1.0e-4:
            return
        self.sequence_done = False
        self._mark_interactive_markers_dirty()
        if index == self.current_index:
            self._publish_current_goal()
        self._publish_visualization()
        self._publish_status(
            f"dragged waypoint {index + 1}: "
            f"({previous.x:.2f}, {previous.y:.2f}, {previous.z:.2f}) -> "
            f"({updated.x:.2f}, {updated.y:.2f}, {updated.z:.2f})"
        )

    def _on_interactive_menu(self, feedback):
        index = self._interactive_waypoint_index(feedback.marker_name)
        if index is None:
            return
        if feedback.menu_entry_id == self.delete_menu_entry:
            self._defer_menu_action("delete", index)
        elif feedback.menu_entry_id == self.clear_menu_entry:
            self._defer_menu_action("clear", index)

    def _defer_menu_action(self, action: str, index: int):
        self._pending_menu_action = (action, index)
        if self._menu_action_timer is None:
            self._menu_action_timer = self.create_timer(
                0.2,
                self._execute_pending_menu_action,
                clock=self.ui_clock,
            )

    def _execute_pending_menu_action(self):
        timer = self._menu_action_timer
        self._menu_action_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

        pending = self._pending_menu_action
        self._pending_menu_action = None
        if pending is None:
            return

        action, index = pending
        if action == "delete":
            self._delete_waypoint(index, f"deleted waypoint {index + 1} from RViz")
        elif action == "clear":
            self._on_clear(Empty())

    def _interactive_waypoint_index(self, name: str) -> Optional[int]:
        prefix = "waypoint_"
        if not name.startswith(prefix):
            return None
        try:
            marker_id = int(name[len(prefix):])
        except ValueError:
            return None
        for index, waypoint in enumerate(self.waypoints):
            if waypoint.marker_id == marker_id:
                return index
        return None

    def _interactive_label(self, index: int) -> str:
        return str(index + 1)

    def _format_status(self) -> str:
        if not self.waypoints:
            return "empty: use RViz Publish Point to add 3D PCT waypoints"
        if self.sequence_done:
            return f"done: {len(self.waypoints)} waypoint(s)"
        state = "paused" if self.paused else "active"
        distance = self._current_goal_distance()
        distance_text = "unknown" if distance is None else f"{distance:.2f}m"
        return (
            f"{state}: waypoint={self.current_index + 1}/{len(self.waypoints)}, "
            f"3d_distance={distance_text}, tolerance={self.goal_tolerance:.2f}m"
        )

    def _publish_status(self, detail: Optional[str] = None):
        msg = String()
        msg.data = self._format_status()
        if detail:
            msg.data += f"; {detail}"
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathSequencePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
