#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ROS2 port of Pure Pursuit sub-goal publisher
#
# Original authorship notes retained in spirit:
#  revision history: xzt
#   20210604 (TE): first version (ROS1)
#
# This ROS2 node publishes a sub-goal point using the pure pursuit algorithm.
# Topics:
#   Subscribes:  path (nav_msgs/Path)
#   Publishes:   subgoal (geometry_msgs/PoseStamped), final_goal (geometry_msgs/PoseStamped)
#
# Frames:
#   world_frame (default: "map")
#   robot_frame (default: "base_link")
#
# Parameters (ROS2):
#   lookahead   [double, default 1.0]  Lookahead distance (m)
#   rate        [double, default 20.0] Control loop rate (Hz)
#   goal_margin [double, default 0.9]  Distance threshold to final goal (m)  # kept for parity
#   wheel_base  [double, default 0.23] Robot wheel base (m)                  # kept for parity
#   wheel_radius[double, default 0.025]Wheel radius (m)                      # kept for parity
#   v_max       [double, default 0.5]  Max linear velocity (m/s)             # kept for parity
#   w_max       [double, default 5.0]  Max angular velocity (rad/s)          # kept for parity
#   world_frame [string, default "map"]
#   robot_frame [string, default "base_link"]
#
# Notes:
# - Uses tf2_ros Buffer/TransformListener to query robot pose.
# - Applies TRANSIENT_LOCAL QoS on the Path subscription so late-joiners can
#   receive the last path (ROS1 latched-topic behavior).
# - Adds multiple numerical/logic guards to avoid runtime errors.

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener

# tf transformations (Euler/Quaternion ops)
# Package name is 'tf_transformations' in ROS 2 (python lib).
from tf_transformations import (
    euler_from_quaternion,
    quaternion_multiply,
    quaternion_inverse,
)


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # ---------------- Parameters ----------------
        self.declare_parameter('lookahead', 1.8)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('goal_margin', 0.45)

        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.5)

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        self.lookahead = float(self.get_parameter('lookahead').value)
        self.rate = float(self.get_parameter('rate').value)
        self.goal_margin = float(self.get_parameter('goal_margin').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        # ---------------- TF2 ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Data & Lock ----------------
        self.path = None
        self.lock = threading.Lock()
        self.timer = None
        self._waiting_for_path_logged = False

        # ---------------- QoS ----------------
        # Path is often published once and should be latched-like for late subscribers.
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---------------- Sub/Pub ----------------
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, path_qos)
        self.cnn_goal_pub = self.create_publisher(PoseStamped, 'subgoal', pub_qos)
        self.final_goal_pub = self.create_publisher(PoseStamped, 'final_goal', pub_qos)

        self.get_logger().info('PurePursuitNode initialized (ROS2).')

    # --------------- Path Callback ---------------
    def path_callback(self, msg: Path):
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg

        if self.path is None or len(self.path.poses) < 2:
            if not self._waiting_for_path_logged:
                self.get_logger().info('PurePursuit: Path cleared, waiting for a new plan')
                self._waiting_for_path_logged = True
        else:
            self._waiting_for_path_logged = False

        # Start timer upon first path reception
        if self.timer is None:
            self.start()

    # --------------- Timer ---------------
    def start(self):
        # run controller at specified rate
        period = 1.0 / max(self.rate, 1e-3)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'PurePursuit control loop started at {self.rate:.1f} Hz')

    # --------------- TF Pose ---------------
    def get_current_pose(self):
        try:
            t: Time = Time()  # latest available
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, t)
        except Exception as ex:
            self.get_logger().warn(f'Could not get robot pose: {ex}')
            return np.array([np.nan, np.nan]), np.nan

        x = np.array([trans.transform.translation.x, trans.transform.translation.y], dtype=float)
        q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        (_, _, theta) = euler_from_quaternion(q)
        self.get_logger().debug(f'x = {x[0]:.3f}, y = {x[1]:.3f}, theta = {theta:.3f}')
        return x, theta

    # --------------- Geometry Helpers ---------------
    def find_closest_point(self, x: np.ndarray, seg: int = -1):
        """
        Find the closest point on the path to x.

        Returns:
            pt_min: np.array([x, y])
            dist_min: float
            seg_min: int
        """
        pt_min = np.array([np.nan, np.nan], dtype=float)
        dist_min = np.inf
        seg_min = -1

        if self.path is None or len(self.path.poses) < 2:
            return pt_min, dist_min, seg_min

        if seg == -1:
            # search entire path
            for i in range(len(self.path.poses) - 1):
                pt, dist, s = self.find_closest_point(x, i)
                if dist < dist_min:
                    pt_min, dist_min, seg_min = pt, dist, s
        else:
            # single segment
            p_start = np.array([
                self.path.poses[seg].pose.position.x,
                self.path.poses[seg].pose.position.y,
            ], dtype=float)
            p_end = np.array([
                self.path.poses[seg + 1].pose.position.x,
                self.path.poses[seg + 1].pose.position.y,
            ], dtype=float)

            v = p_end - p_start
            length_seg = np.linalg.norm(v)
            if length_seg < 1e-9:
                # degenerate segment; choose start point distance
                pt_min = p_start
                dist_min = np.linalg.norm(pt_min - x)
                seg_min = seg
                return pt_min, dist_min, seg_min

            v = v / length_seg
            dist_projected = np.dot(x - p_start, v)

            if dist_projected < 0.0:
                pt_min = p_start
            elif dist_projected > length_seg:
                pt_min = p_end
            else:
                pt_min = p_start + dist_projected * v

            dist_min = np.linalg.norm(pt_min - x)
            seg_min = seg

        return pt_min, dist_min, seg_min

    def find_goal(self, x: np.ndarray, pt: np.ndarray, dist: float, seg: int):
        """
        Determine the goal point along the path.
        Returns:
            goal: np.array([x, y])
            end_goal_pos: [x, y]
            end_goal_rot: [x, y, z, w]
        """
        goal = None

        if self.path is None or len(self.path.poses) < 2:
            return None, None, None

        # default: end pose info (used for final goal frame transform/orientation)
        end_pose = self.path.poses[-1].pose
        end_goal_pos = [end_pose.position.x, end_pose.position.y]
        end_goal_rot = [
            end_pose.orientation.x,
            end_pose.orientation.y,
            end_pose.orientation.z,
            end_pose.orientation.w,
        ]

        if dist > self.lookahead:
            # far from path: drive toward closest point
            goal = pt
        else:
            seg_max = len(self.path.poses) - 2

            # end of current segment
            p_end = np.array([
                self.path.poses[seg + 1].pose.position.x,
                self.path.poses[seg + 1].pose.position.y,
            ], dtype=float)
            dist_end = np.linalg.norm(x - p_end)

            # advance until leaving the lookahead circle or reaching last segment
            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = np.array([
                    self.path.poses[seg + 1].pose.position.x,
                    self.path.poses[seg + 1].pose.position.y,
                ], dtype=float)
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                # searched whole path: goal is the path end
                pt2 = np.array([
                    self.path.poses[seg_max + 1].pose.position.x,
                    self.path.poses[seg_max + 1].pose.position.y,
                ], dtype=float)
                goal = pt2
            else:
                # find intersection with the lookahead circle on this segment
                pt2, _, seg2 = self.find_closest_point(x, seg)

                p_start = np.array([
                    self.path.poses[seg2].pose.position.x,
                    self.path.poses[seg2].pose.position.y,
                ], dtype=float)
                p_end = np.array([
                    self.path.poses[seg2 + 1].pose.position.x,
                    self.path.poses[seg2 + 1].pose.position.y,
                ], dtype=float)

                v = p_end - p_start
                length_seg = np.linalg.norm(v)
                if length_seg < 1e-9:
                    goal = pt2
                else:
                    v = v / length_seg
                    dist_proj_x = np.dot(x - pt2, v)
                    dist_proj_y = np.linalg.norm(np.cross(x - pt2, v))
                    under_radical = self.lookahead ** 2 - dist_proj_y ** 2
                    if under_radical < 0.0:
                        # numerical guard
                        under_radical = 0.0
                    goal = pt2 + (np.sqrt(under_radical) + dist_proj_x) * v

        return goal, end_goal_pos, end_goal_rot

    # --------------- Main Control Loop ---------------
    def timer_callback(self):
        with self.lock:
            # get current pose
            x, theta = self.get_current_pose()
            if np.isnan(x[0]):
                return

            # closest point
            pt, dist, seg = self.find_closest_point(x)
            if np.isnan(pt).any() or seg < 0:
                return

            # goal
            goal, end_goal_pos, end_goal_rot = self.find_goal(x, pt, dist, seg)
            if goal is None or end_goal_pos is None:
                return

        # ---- Transform goal to robot(local) coordinates ----
        # Homogeneous transform (map -> robot)
        c, s = np.cos(theta), np.sin(theta)
        map_T_robot = np.array([[c, -s, x[0]],
                                [s,  c, x[1]],
                                [0., 0., 1. ]], dtype=float)
        inv_map_T_robot = np.linalg.inv(map_T_robot)

        goal_h = np.array([[goal[0]], [goal[1]], [1.0]], dtype=float)
        goal_local = inv_map_T_robot @ goal_h   # 3x1
        goal_local = goal_local[0:2, :].flatten()  # (2,)

        # ---- Final goal (relative to robot) ----
        end_goal_h = np.array([[end_goal_pos[0]], [end_goal_pos[1]], [1.0]], dtype=float)
        relative_goal = (inv_map_T_robot @ end_goal_h).flatten()

        # ---- Relative orientation to final goal ----
        # orientation_to_target = end_goal_rot * inverse(current_rot)
        try:
            # get current orientation again to avoid sharing state; cheap op
            t = Time()
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, t)
            cur_q = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ]
            orientation_to_target = quaternion_multiply(end_goal_rot, quaternion_inverse(cur_q))
            yaw = euler_from_quaternion(orientation_to_target)[2]
            _ = yaw  # kept for possible downstream usage
        except Exception:
            pass

        # ---- Publish subgoal ----
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.robot_frame

        cnn_goal = PoseStamped()
        cnn_goal.header = hdr
        cnn_goal.pose.position.x = float(goal_local[0])
        cnn_goal.pose.position.y = float(goal_local[1])

        if not np.isnan(cnn_goal.pose.position.x) and not np.isnan(cnn_goal.pose.position.y):
            self.cnn_goal_pub.publish(cnn_goal)
            self.get_logger().info(f'Subgoal (local): {goal_local[0]:.3f}, {goal_local[1]:.3f}')

        # ---- Publish final goal (relative position only) ----
        final_goal = PoseStamped()
        final_goal.header = hdr
        final_goal.pose.position.x = float(relative_goal[0])
        final_goal.pose.position.y = float(relative_goal[1])

        if not np.isnan(final_goal.pose.position.x) and not np.isnan(final_goal.pose.position.y):
            self.final_goal_pub.publish(final_goal)
            self.get_logger().info(f'Final goal (local): {relative_goal[0]:.3f}, {relative_goal[1]:.3f}')


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
