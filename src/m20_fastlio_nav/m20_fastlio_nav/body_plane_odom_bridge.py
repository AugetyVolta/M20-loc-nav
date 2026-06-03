import time
from typing import Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from m20_fastlio_nav.fastlio_odom_bridge import (
    Quaternion,
    Vector3,
    is_finite_vector,
    is_valid_quat,
    normalize_quat,
    quat_conjugate,
    quat_multiply,
    rotate_vector,
    shortest_angle_delta,
    vector_param,
    yaw_from_quat,
)


class BodyPlaneOdomBridge(Node):
    """Publish a body-parallel odometry frame for 3D local navigation."""

    def __init__(self) -> None:
        super().__init__("body_plane_odom_bridge")

        self.declare_parameter("source_odom_topic", "/Odometry_loc")
        self.declare_parameter("output_odom_topic", "/odom_body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("body_odom_frame", "odom_body")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("reset_on_wall_time_gap", True)
        self.declare_parameter("bag_switch_wall_gap_sec", 1.5)
        self.declare_parameter(
            "base_to_body_translation",
            [0.32713234, 0.01413551, 0.31238696],
        )
        self.declare_parameter(
            "base_to_body_quaternion",
            [-0.00394028, 0.24367785, 0.00970223, 0.96979969],
        )

        source_topic = str(self.get_parameter("source_odom_topic").value)
        output_topic = str(self.get_parameter("output_odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.body_odom_frame = str(self.get_parameter("body_odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.reset_on_wall_time_gap = bool(self.get_parameter("reset_on_wall_time_gap").value)
        self.bag_switch_wall_gap_sec = float(self.get_parameter("bag_switch_wall_gap_sec").value)

        self.t_base_body = vector_param(
            self.get_parameter("base_to_body_translation").value,
            3,
            "base_to_body_translation",
        )
        q_base_body = vector_param(
            self.get_parameter("base_to_body_quaternion").value,
            4,
            "base_to_body_quaternion",
        )
        self.q_base_body = normalize_quat(q_base_body)  # type: ignore[arg-type]
        self.q_body_base = quat_conjugate(self.q_base_body)
        self.t_body_base = rotate_vector(
            self.q_body_base,
            (-self.t_base_body[0], -self.t_base_body[1], -self.t_base_body[2]),
        )

        self.odom_pub = self.create_publisher(Odometry, output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, source_topic, self.odom_callback, 50)
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, 10)

        self.prev_stamp = None
        self.prev_pos_odom_base: Vector3 | None = None
        self.prev_yaw = None
        self.last_odom_stamp_ns = None
        self.last_clock_stamp_ns = None
        self.last_odom_wall_time = None
        self.last_clock_wall_time = None
        self.invalid_odom_count = 0
        self.invalid_tf_count = 0

        self.get_logger().info(
            f"publishing body-plane odom {output_topic}: "
            f"{self.odom_frame}->{self.body_odom_frame}, odom child={self.base_frame}"
        )

    @staticmethod
    def stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def reset_temporal_state(self, reason: str) -> None:
        self.prev_stamp = None
        self.prev_pos_odom_base = None
        self.prev_yaw = None
        self.get_logger().warn(f"Detected time discontinuity ({reason}); reset body-plane odom state")

    def wall_gap_detected(self, attr_name: str, reason: str) -> bool:
        now = time.monotonic()
        last = getattr(self, attr_name)
        setattr(self, attr_name, now)
        if (
            self.reset_on_wall_time_gap
            and last is not None
            and now - last > self.bag_switch_wall_gap_sec
        ):
            self.reset_temporal_state(f"{reason}; wall gap {now - last:.2f}s")
            self.last_odom_stamp_ns = None
            return True
        return False

    def clock_callback(self, msg: Clock) -> None:
        self.wall_gap_detected("last_clock_wall_time", "/clock stream gap")
        clock_ns = self.stamp_to_ns(msg.clock)
        if self.last_clock_stamp_ns is not None and clock_ns + 100_000_000 < self.last_clock_stamp_ns:
            self.reset_temporal_state("/clock moved backwards")
            self.last_odom_stamp_ns = None
        self.last_clock_stamp_ns = clock_ns

    def odom_callback(self, msg: Odometry) -> None:
        if self.wall_gap_detected("last_odom_wall_time", "/Odometry_loc stream gap"):
            return

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamp_ns = self.stamp_to_ns(stamp)
        if self.last_odom_stamp_ns is not None and stamp_ns + 100_000_000 < self.last_odom_stamp_ns:
            self.reset_temporal_state("/Odometry_loc moved backwards")
        self.last_odom_stamp_ns = stamp_ns

        result = self.base_pose_from_source(msg)
        if result is None:
            return
        p_odom_base, q_odom_base = result

        p_body_odom_base = rotate_vector(
            quat_conjugate(q_odom_base),
            p_odom_base,
        )
        yaw = yaw_from_quat(q_odom_base)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.body_odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = p_body_odom_base[0]
        out.pose.pose.position.y = p_body_odom_base[1]
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance = msg.pose.covariance

        if self.prev_stamp is not None and self.prev_pos_odom_base is not None and self.prev_yaw is not None:
            dt = (stamp_ns - self.prev_stamp) * 1.0e-9
            if 0.001 <= dt <= 0.5:
                delta_odom = (
                    p_odom_base[0] - self.prev_pos_odom_base[0],
                    p_odom_base[1] - self.prev_pos_odom_base[1],
                    p_odom_base[2] - self.prev_pos_odom_base[2],
                )
                delta_base = rotate_vector(quat_conjugate(q_odom_base), delta_odom)
                out.twist.twist.linear.x = delta_base[0] / dt
                out.twist.twist.linear.y = delta_base[1] / dt
                out.twist.twist.linear.z = 0.0
                out.twist.twist.angular.z = shortest_angle_delta(yaw, self.prev_yaw) / dt

        out.twist.covariance[0] = 0.05
        out.twist.covariance[7] = 0.05
        out.twist.covariance[35] = 0.05

        self.prev_stamp = stamp_ns
        self.prev_pos_odom_base = p_odom_base
        self.prev_yaw = yaw

        self.odom_pub.publish(out)
        if self.publish_tf:
            self.publish_body_odom_tf(stamp, q_odom_base)

    def base_pose_from_source(self, msg: Odometry) -> Tuple[Vector3, Quaternion] | None:
        src_pose = msg.pose.pose
        p_odom_body = (
            float(src_pose.position.x),
            float(src_pose.position.y),
            float(src_pose.position.z),
        )
        q_odom_body_raw = (
            float(src_pose.orientation.x),
            float(src_pose.orientation.y),
            float(src_pose.orientation.z),
            float(src_pose.orientation.w),
        )
        if not is_finite_vector(p_odom_body) or not is_valid_quat(q_odom_body_raw):
            self.invalid_odom_count += 1
            if self.invalid_odom_count == 1 or self.invalid_odom_count % 50 == 0:
                self.get_logger().warn(
                    "Ignoring invalid /Odometry_loc pose; not publishing /odom_body "
                    f"(count={self.invalid_odom_count})"
                )
            return None
        self.invalid_odom_count = 0

        q_odom_body = normalize_quat(q_odom_body_raw)
        rotated_body_base = rotate_vector(q_odom_body, self.t_body_base)
        p_odom_base = (
            p_odom_body[0] + rotated_body_base[0],
            p_odom_body[1] + rotated_body_base[1],
            p_odom_body[2] + rotated_body_base[2],
        )
        q_odom_base = quat_multiply(q_odom_body, self.q_body_base)
        return p_odom_base, q_odom_base

    def publish_body_odom_tf(self, stamp, q_odom_base: Quaternion) -> None:
        if not is_valid_quat(q_odom_base):
            self.invalid_tf_count += 1
            if self.invalid_tf_count == 1 or self.invalid_tf_count % 50 == 0:
                self.get_logger().warn(
                    f"Not publishing invalid TF {self.odom_frame}->{self.body_odom_frame} "
                    f"(count={self.invalid_tf_count})"
                )
            return
        self.invalid_tf_count = 0

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.body_odom_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q_odom_base[0]
        tf_msg.transform.rotation.y = q_odom_base[1]
        tf_msg.transform.rotation.z = q_odom_base[2]
        tf_msg.transform.rotation.w = q_odom_base[3]
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BodyPlaneOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
