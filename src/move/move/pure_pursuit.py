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
    quaternion_matrix,
    quaternion_multiply,
    quaternion_inverse,
)


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # ---------------- Parameters ----------------
        self.declare_parameter('lookahead', 1.8)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('goal_margin', 0.45)

        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.5)

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('use_3d_path_distance', False)

        self.lookahead = float(self.get_parameter('lookahead').value)
        self.rate = float(self.get_parameter('rate').value)
        self.goal_margin = float(self.get_parameter('goal_margin').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.use_3d_path_distance = bool(self.get_parameter('use_3d_path_distance').value)

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
        volatile_path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        latched_path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
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
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, volatile_path_qos)
        self.path_sub_latched = self.create_subscription(
            Path, 'plan', self.path_callback, latched_path_qos
        )
        self.cnn_goal_pub = self.create_publisher(PoseStamped, 'subgoal', pub_qos)
        self.final_goal_pub = self.create_publisher(PoseStamped, 'final_goal', pub_qos)

        self.get_logger().debug('PurePursuitNode initialized (ROS2).')

    # --------------- Path Callback ---------------
    def path_callback(self, msg: Path):
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg

        if self.path is None or len(self.path.poses) < 2:
            if not self._waiting_for_path_logged:
                self.get_logger().debug('PurePursuit: Path cleared, waiting for a new plan')
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
        self.get_logger().debug(f'PurePursuit control loop started at {self.rate:.1f} Hz')

    # --------------- TF Pose ---------------
    def get_current_pose(self):
        try:
            t: Time = Time()  # latest available
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, t)
        except Exception as ex:
            self.get_logger().debug(f'Could not get robot pose: {ex}')
            dim = 3 if self.use_3d_path_distance else 2
            return np.full(dim, np.nan, dtype=float), np.nan, None

        if self.use_3d_path_distance:
            x = np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ],
                dtype=float,
            )
        else:
            x = np.array([trans.transform.translation.x, trans.transform.translation.y], dtype=float)
        q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        (_, _, theta) = euler_from_quaternion(q)
        self.get_logger().debug(f'x = {x[0]:.3f}, y = {x[1]:.3f}, theta = {theta:.3f}')
        return x, theta, trans

    def path_point(self, idx: int) -> np.ndarray:
        pos = self.path.poses[idx].pose.position
        if self.use_3d_path_distance:
            return np.array([pos.x, pos.y, pos.z], dtype=float)
        return np.array([pos.x, pos.y], dtype=float)

    # --------------- Geometry Helpers ---------------
    def find_closest_point(self, x: np.ndarray, seg: int = -1):
        """
        Find the closest point on the path to x.

        Returns:
            pt_min: np.array([x, y])
            dist_min: float
            seg_min: int
        """
        dim = 3 if self.use_3d_path_distance else 2
        pt_min = np.full(dim, np.nan, dtype=float)
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
            p_start = self.path_point(seg)
            p_end = self.path_point(seg + 1)

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
        if self.use_3d_path_distance:
            end_goal_pos = [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        else:
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
            p_end = self.path_point(seg + 1)
            dist_end = np.linalg.norm(x - p_end)

            # advance until leaving the lookahead circle or reaching last segment
            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = self.path_point(seg + 1)
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                # searched whole path: goal is the path end
                pt2 = self.path_point(seg_max + 1)
                goal = pt2
            else:
                # find intersection with the lookahead circle on this segment
                pt2, _, seg2 = self.find_closest_point(x, seg)

                p_start = self.path_point(seg2)
                p_end = self.path_point(seg2 + 1)

                v = p_end - p_start
                length_seg = np.linalg.norm(v)
                if length_seg < 1e-9:
                    goal = pt2
                else:
                    v = v / length_seg
                    dist_proj_x = np.dot(x - pt2, v)
                    dist_proj_y2 = np.linalg.norm(x - pt2) ** 2 - dist_proj_x ** 2
                    dist_proj_y2 = max(0.0, float(dist_proj_y2))
                    under_radical = self.lookahead ** 2 - dist_proj_y2
                    if under_radical < 0.0:
                        # numerical guard
                        under_radical = 0.0
                    goal = pt2 + (np.sqrt(under_radical) + dist_proj_x) * v

        return goal, end_goal_pos, end_goal_rot

    def world_point_to_robot(self, point, tf_world_robot, theta):
        if self.use_3d_path_distance:
            p = np.asarray(point, dtype=float)
            if p.shape[0] == 2:
                p = np.array([p[0], p[1], 0.0], dtype=float)
            t = np.array(
                [
                    tf_world_robot.transform.translation.x,
                    tf_world_robot.transform.translation.y,
                    tf_world_robot.transform.translation.z,
                ],
                dtype=float,
            )
            q = tf_world_robot.transform.rotation
            rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            return rot.T @ (p - t)

        c, s = np.cos(theta), np.sin(theta)
        rel = np.array(
            [
                point[0] - tf_world_robot.transform.translation.x,
                point[1] - tf_world_robot.transform.translation.y,
            ],
            dtype=float,
        )
        return np.array([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]], dtype=float)

    # --------------- Main Control Loop ---------------
    def timer_callback(self):
        with self.lock:
            # get current pose
            x, theta, tf_world_robot = self.get_current_pose()
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
        goal_local = self.world_point_to_robot(goal, tf_world_robot, theta)
        relative_goal = self.world_point_to_robot(end_goal_pos, tf_world_robot, theta)

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
        if self.use_3d_path_distance and len(goal_local) > 2:
            cnn_goal.pose.position.z = float(goal_local[2])

        if not np.isnan(cnn_goal.pose.position.x) and not np.isnan(cnn_goal.pose.position.y):
            self.cnn_goal_pub.publish(cnn_goal)
            self.get_logger().debug(f'Subgoal (local): {goal_local[0]:.3f}, {goal_local[1]:.3f}')

        # ---- Publish final goal (relative position only) ----
        final_goal = PoseStamped()
        final_goal.header = hdr
        final_goal.pose.position.x = float(relative_goal[0])
        final_goal.pose.position.y = float(relative_goal[1])
        if self.use_3d_path_distance and len(relative_goal) > 2:
            final_goal.pose.position.z = float(relative_goal[2])

        if not np.isnan(final_goal.pose.position.x) and not np.isnan(final_goal.pose.position.y):
            self.final_goal_pub.publish(final_goal)
            self.get_logger().debug(f'Final goal (local): {relative_goal[0]:.3f}, {relative_goal[1]:.3f}')


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
