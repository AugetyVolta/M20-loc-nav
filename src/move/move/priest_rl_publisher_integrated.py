#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Integrated a.py + pure_pursuit.py local planner.

This entrypoint keeps a.py as the PRIEST/RL local path implementation and
embeds the pure_pursuit.py subgoal publisher in the same ROS node.

Topics kept compatible with pure_pursuit.py:
  Subscribes: plan
  Publishes: subgoal, final_goal

With this file, pure_pursuit.py does not need to be launched separately.
"""

import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from std_msgs.msg import Header
from tf_transformations import (
    euler_from_quaternion,
    quaternion_inverse,
    quaternion_multiply,
)

try:
    from move.priest_rl_publisher_nav_cmd import RLLocalPlannerNodeROS2
except ImportError:
    from priest_rl_publisher_nav_cmd import RLLocalPlannerNodeROS2


class IntegratedALocalPlannerNode(RLLocalPlannerNodeROS2):
    def __init__(self):
        super().__init__()

        # Keep these compatible with pure_pursuit.py.
        self.declare_parameter("integrated_pure_pursuit", True)
        self.declare_parameter("lookahead", 1.8)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("goal_margin", 0.45)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        self.integrated_pure_pursuit = bool(
            self.get_parameter("integrated_pure_pursuit").get_parameter_value().bool_value
        )
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.rate = float(self.get_parameter("rate").value)
        self.goal_margin = float(self.get_parameter("goal_margin").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)

        self.path: Optional[Path] = None
        self.lock = threading.Lock()
        self.pp_timer = None
        self.final_goal_position = None

        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.path_sub = None
        self.cnn_goal_pub = None
        self.final_goal_pub = None
        if self.integrated_pure_pursuit:
            self.path_sub = self.create_subscription(Path, "plan", self.path_callback, path_qos)
            self.cnn_goal_pub = self.create_publisher(PoseStamped, "subgoal", pub_qos)
            self.final_goal_pub = self.create_publisher(PoseStamped, "final_goal", pub_qos)

        self.get_logger().warn(
            "[a_integrated] a.py + pure_pursuit.py are running in one node.\n"
            f"  enabled={self.integrated_pure_pursuit}\n"
            "  subscribes=plan, publishes=subgoal/final_goal\n"
            f"  lookahead={self.lookahead}, rate={self.rate}, goal_margin={self.goal_margin}\n"
            f"  world_frame={self.world_frame}, robot_frame={self.robot_frame}"
        )

    # --------------- Pure pursuit path callback ---------------
    def path_callback(self, msg: Path):
        self.get_logger().debug("PurePursuit: Got path")
        with self.lock:
            self.path = msg

        if self.pp_timer is None:
            self.start_pure_pursuit()

    def start_pure_pursuit(self):
        period = 1.0 / max(self.rate, 1e-3)
        self.pp_timer = self.create_timer(period, self.pure_pursuit_timer_callback)
        self.get_logger().info(f"PurePursuit control loop started at {self.rate:.1f} Hz")

    # --------------- TF Pose, same behavior as pure_pursuit.py ---------------
    def get_current_pose(self):
        try:
            t: Time = Time()
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, t)
        except Exception as ex:
            self.get_logger().warn(f"Could not get robot pose: {ex}")
            return np.array([np.nan, np.nan]), np.nan

        x = np.array([trans.transform.translation.x, trans.transform.translation.y], dtype=float)
        q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        (_, _, theta) = euler_from_quaternion(q)
        self.get_logger().debug(f"x = {x[0]:.3f}, y = {x[1]:.3f}, theta = {theta:.3f}")
        return x, theta

    # --------------- Geometry Helpers, same behavior as pure_pursuit.py ---------------
    def find_closest_point(self, x: np.ndarray, seg: int = -1):
        pt_min = np.array([np.nan, np.nan], dtype=float)
        dist_min = np.inf
        seg_min = -1

        if self.path is None or len(self.path.poses) < 2:
            self.get_logger().warn("Pure Pursuit: No valid path received yet")
            return pt_min, dist_min, seg_min

        if seg == -1:
            for i in range(len(self.path.poses) - 1):
                pt, dist, s = self.find_closest_point(x, i)
                if dist < dist_min:
                    pt_min, dist_min, seg_min = pt, dist, s
        else:
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
        goal = None

        if self.path is None or len(self.path.poses) < 2:
            self.get_logger().warn("Pure Pursuit: No valid path to find goal")
            return None, None, None

        end_pose = self.path.poses[-1].pose
        end_goal_pos = [end_pose.position.x, end_pose.position.y]
        end_goal_rot = [
            end_pose.orientation.x,
            end_pose.orientation.y,
            end_pose.orientation.z,
            end_pose.orientation.w,
        ]

        if dist > self.lookahead:
            goal = pt
        else:
            seg_max = len(self.path.poses) - 2

            p_end = np.array([
                self.path.poses[seg + 1].pose.position.x,
                self.path.poses[seg + 1].pose.position.y,
            ], dtype=float)
            dist_end = np.linalg.norm(x - p_end)

            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = np.array([
                    self.path.poses[seg + 1].pose.position.x,
                    self.path.poses[seg + 1].pose.position.y,
                ], dtype=float)
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                pt2 = np.array([
                    self.path.poses[seg_max + 1].pose.position.x,
                    self.path.poses[seg_max + 1].pose.position.y,
                ], dtype=float)
                goal = pt2
            else:
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
                        under_radical = 0.0
                    goal = pt2 + (np.sqrt(under_radical) + dist_proj_x) * v

        return goal, end_goal_pos, end_goal_rot

    # --------------- Main pure pursuit loop, same behavior as pure_pursuit.py ---------------
    def pure_pursuit_timer_callback(self):
        with self.lock:
            x, theta = self.get_current_pose()
            if np.isnan(x[0]):
                return

            pt, dist, seg = self.find_closest_point(x)
            if np.isnan(pt).any() or seg < 0:
                return

            goal, end_goal_pos, end_goal_rot = self.find_goal(x, pt, dist, seg)
            if goal is None or end_goal_pos is None:
                return

        c, s = np.cos(theta), np.sin(theta)
        map_T_robot = np.array(
            [
                [c, -s, x[0]],
                [s, c, x[1]],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        inv_map_T_robot = np.linalg.inv(map_T_robot)

        goal_h = np.array([[goal[0]], [goal[1]], [1.0]], dtype=float)
        goal_local = inv_map_T_robot @ goal_h
        goal_local = goal_local[0:2, :].flatten()

        end_goal_h = np.array([[end_goal_pos[0]], [end_goal_pos[1]], [1.0]], dtype=float)
        relative_goal = (inv_map_T_robot @ end_goal_h).flatten()

        try:
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
            _ = yaw
        except Exception:
            pass

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.robot_frame

        cnn_goal = PoseStamped()
        cnn_goal.header = hdr
        cnn_goal.pose.position.x = float(goal_local[0])
        cnn_goal.pose.position.y = float(goal_local[1])

        if not np.isnan(cnn_goal.pose.position.x) and not np.isnan(cnn_goal.pose.position.y):
            self.subgoal_position = (cnn_goal.pose.position.x, cnn_goal.pose.position.y)
            if self.cnn_goal_pub is not None:
                self.cnn_goal_pub.publish(cnn_goal)
            self.get_logger().info(f"Subgoal (local): {goal_local[0]:.3f}, {goal_local[1]:.3f}")

        final_goal = PoseStamped()
        final_goal.header = hdr
        final_goal.pose.position.x = float(relative_goal[0])
        final_goal.pose.position.y = float(relative_goal[1])

        if not np.isnan(final_goal.pose.position.x) and not np.isnan(final_goal.pose.position.y):
            self.final_goal_position = (final_goal.pose.position.x, final_goal.pose.position.y)
            if self.final_goal_pub is not None:
                self.final_goal_pub.publish(final_goal)
            self.get_logger().info(
                f"Final goal (local): {relative_goal[0]:.3f}, {relative_goal[1]:.3f}"
            )


def main():
    rclpy.init()
    node = IntegratedALocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
