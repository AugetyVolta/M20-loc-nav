import math
import time
from typing import Iterable, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w


def normalize_quat(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    if not all(math.isfinite(v) for v in q):
        return (0.0, 0.0, 0.0, 1.0)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def is_finite_vector(v: Vector3) -> bool:
    return all(math.isfinite(value) for value in v)


def is_valid_quat(q: Quaternion) -> bool:
    if not all(math.isfinite(v) for v in q):
        return False
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return math.isfinite(norm) and norm >= 1.0e-9


def quat_conjugate(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quat(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    qx, qy, qz, qw = normalize_quat(q)
    vx, vy, vz = v

    # q * v * q^-1, expanded to avoid allocations.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quat_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quat(q: Quaternion) -> float:
    x, y, z, w = normalize_quat(q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(now: float, prev: float) -> float:
    return math.atan2(math.sin(now - prev), math.cos(now - prev))


def compose_transform(
    p_parent_child1: Vector3,
    q_parent_child1: Quaternion,
    p_child1_child2: Vector3,
    q_child1_child2: Quaternion,
) -> Tuple[Vector3, Quaternion]:
    rotated = rotate_vector(q_parent_child1, p_child1_child2)
    return (
        (
            p_parent_child1[0] + rotated[0],
            p_parent_child1[1] + rotated[1],
            p_parent_child1[2] + rotated[2],
        ),
        quat_multiply(q_parent_child1, q_child1_child2),
    )


def invert_transform(position: Vector3, orientation: Quaternion) -> Tuple[Vector3, Quaternion]:
    q_inv = quat_conjugate(orientation)
    p_inv = rotate_vector(q_inv, (-position[0], -position[1], -position[2]))
    return p_inv, q_inv


def planarize_pose(position: Vector3, orientation: Quaternion) -> Tuple[Vector3, Quaternion]:
    yaw = yaw_from_quat(orientation)
    return (position[0], position[1], 0.0), quat_from_yaw(yaw)


def vector_param(value: Iterable[float], expected_len: int, name: str) -> Tuple[float, ...]:
    data = tuple(float(v) for v in value)
    if len(data) != expected_len:
        raise ValueError(f"{name} must have {expected_len} values, got {len(data)}")
    return data


class FastLioOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_odom_bridge")

        self.declare_parameter("source_odom_topic", "/Odometry_loc")
        self.declare_parameter("output_odom_topic", "/odom_body")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("nav_odom_frame", "odom_body")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("nav_base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_nav_base_tf", False)
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

        source_topic = self.get_parameter("source_odom_topic").value
        output_topic = self.get_parameter("output_odom_topic").value
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.nav_odom_frame = str(self.get_parameter("nav_odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.nav_base_frame = str(self.get_parameter("nav_base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_nav_base_tf = bool(self.get_parameter("publish_nav_base_tf").value)
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
        neg_t_base_body = (
            -self.t_base_body[0],
            -self.t_base_body[1],
            -self.t_base_body[2],
        )
        self.t_body_base = rotate_vector(self.q_body_base, neg_t_base_body)

        self.odom_pub = self.create_publisher(Odometry, output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.prev_stamp = None
        self.prev_pos = None
        self.prev_yaw = None
        self.body_odom_anchor = None
        self.last_odom_stamp_ns = None
        self.last_clock_stamp_ns = None
        self.last_odom_wall_time = None
        self.last_clock_wall_time = None
        self.invalid_odom_count = 0
        self.invalid_tf_count = 0

        self.sub = self.create_subscription(Odometry, source_topic, self.odom_callback, 50)
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.get_logger().info(
            f"bridging {source_topic} to {output_topic} as "
            f"{self.map_frame}->{self.nav_odom_frame} body-plane and "
            f"{self.odom_frame}->{self.base_frame} (3D), "
            f"odom child={self.nav_base_frame}, publish_nav_base_tf={self.publish_nav_base_tf}"
        )

    @staticmethod
    def stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def reset_tf_buffer(self) -> None:
        self.tf_buffer.clear()

    def reset_temporal_state(self, reason: str) -> None:
        self.prev_stamp = None
        self.prev_pos = None
        self.prev_yaw = None
        self.body_odom_anchor = None
        self.reset_tf_buffer()
        self.get_logger().warn(f"Detected time discontinuity ({reason}); reset bridge TF buffer/state")

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

    def lookup_map_to_odom(self) -> Tuple[Vector3, Quaternion, object] | None:
        try:
            transform_map_odom = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                Time(),
            )
        except Exception:
            return None

        position = (
            float(transform_map_odom.transform.translation.x),
            float(transform_map_odom.transform.translation.y),
            float(transform_map_odom.transform.translation.z),
        )
        orientation_raw = (
            float(transform_map_odom.transform.rotation.x),
            float(transform_map_odom.transform.rotation.y),
            float(transform_map_odom.transform.rotation.z),
            float(transform_map_odom.transform.rotation.w),
        )
        if not is_finite_vector(position) or not is_valid_quat(orientation_raw):
            self.get_logger().warn("Ignoring invalid map->odom transform from TF buffer")
            return None
        orientation = normalize_quat(orientation_raw)
        return position, orientation, transform_map_odom.header.stamp

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
                    "Ignoring invalid /Odometry_loc pose; not publishing body-plane odom or TF "
                    f"(count={self.invalid_odom_count})"
                )
            return
        self.invalid_odom_count = 0
        q_odom_body = normalize_quat(q_odom_body_raw)

        rotated_body_base = rotate_vector(q_odom_body, self.t_body_base)
        p_odom_base = (
            p_odom_body[0] + rotated_body_base[0],
            p_odom_body[1] + rotated_body_base[1],
            p_odom_body[2] + rotated_body_base[2],
        )
        q_odom_base = quat_multiply(q_odom_body, self.q_body_base)

        if self.body_odom_anchor is None:
            self.body_odom_anchor = p_odom_base
            self.get_logger().info(
                "body-plane odom anchor set at "
                f"x={p_odom_base[0]:.3f}, y={p_odom_base[1]:.3f}, z={p_odom_base[2]:.3f}"
            )

        p_odom_nav = (
            p_odom_base[0] - self.body_odom_anchor[0],
            p_odom_base[1] - self.body_odom_anchor[1],
            0.0,
        )
        q_odom_nav = quat_from_yaw(yaw_from_quat(q_odom_base))
        nav_odom_tf = None

        map_to_odom = self.lookup_map_to_odom()
        if map_to_odom is not None:
            p_map_odom, q_map_odom, _tf_stamp = map_to_odom
            p_map_base, q_map_base = compose_transform(
                p_map_odom,
                q_map_odom,
                p_odom_base,
                q_odom_base,
            )

            # Body-plane Nav2 frame:
            # - /odom_body keeps a small anchored planar pose for DWB progress and velocity math.
            # - map -> odom_body is chosen so TF lookup odom_body -> base_link lands on
            #   the real 3D base_link pose, including stair height and body plane.
            q_map_odom_nav = quat_multiply(q_map_base, quat_conjugate(q_odom_nav))
            rotated_nav_base = rotate_vector(q_map_odom_nav, p_odom_nav)
            p_map_odom_nav = (
                p_map_base[0] - rotated_nav_base[0],
                p_map_base[1] - rotated_nav_base[1],
                p_map_base[2] - rotated_nav_base[2],
            )
            nav_odom_tf = (stamp, p_map_odom_nav, q_map_odom_nav)

        nav_yaw = yaw_from_quat(q_odom_base)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.nav_odom_frame
        out.child_frame_id = self.nav_base_frame
        out.pose.pose.position.x = p_odom_nav[0]
        out.pose.pose.position.y = p_odom_nav[1]
        out.pose.pose.position.z = p_odom_nav[2]
        out.pose.pose.orientation.x = q_odom_nav[0]
        out.pose.pose.orientation.y = q_odom_nav[1]
        out.pose.pose.orientation.z = q_odom_nav[2]
        out.pose.pose.orientation.w = q_odom_nav[3]
        out.pose.covariance = msg.pose.covariance

        if self.prev_stamp is not None and self.prev_pos is not None and self.prev_yaw is not None:
            dt = (stamp_ns - self.prev_stamp) * 1.0e-9
            if 0.001 <= dt <= 0.5:
                delta_odom = (
                    p_odom_base[0] - self.prev_pos[0],
                    p_odom_base[1] - self.prev_pos[1],
                    p_odom_base[2] - self.prev_pos[2],
                )
                delta_base = rotate_vector(quat_conjugate(q_odom_base), delta_odom)
                out.twist.twist.linear.x = delta_base[0] / dt
                out.twist.twist.linear.y = delta_base[1] / dt
                out.twist.twist.linear.z = 0.0
                out.twist.twist.angular.z = shortest_angle_delta(nav_yaw, self.prev_yaw) / dt

        out.twist.covariance[0] = 0.05
        out.twist.covariance[7] = 0.05
        out.twist.covariance[35] = 0.05

        self.prev_stamp = stamp_ns
        self.prev_pos = p_odom_base
        self.prev_yaw = nav_yaw

        self.odom_pub.publish(out)
        if self.publish_tf:
            if nav_odom_tf is not None:
                tf_stamp, p_map_odom_nav, q_map_odom_nav = nav_odom_tf
                self.publish_transform(
                    tf_stamp,
                    p_map_odom_nav,
                    q_map_odom_nav,
                    self.nav_odom_frame,
                    self.map_frame,
                )
            if self.publish_nav_base_tf:
                self.publish_transform(
                    stamp,
                    p_odom_nav,
                    q_odom_nav,
                    self.nav_base_frame,
                    self.nav_odom_frame,
                )
            self.publish_transform(
                stamp,
                p_odom_base,
                q_odom_base,
                self.base_frame,
                self.odom_frame,
            )

    def publish_transform(
        self,
        stamp,
        position: Vector3,
        orientation: Quaternion,
        child_frame_id: str,
        parent_frame_id: str = "",
    ) -> None:
        if not is_finite_vector(position) or not is_valid_quat(orientation):
            self.invalid_tf_count += 1
            if self.invalid_tf_count == 1 or self.invalid_tf_count % 50 == 0:
                self.get_logger().warn(
                    f"Not publishing invalid TF {parent_frame_id or self.odom_frame}->{child_frame_id} "
                    f"(count={self.invalid_tf_count})"
                )
            return
        self.invalid_tf_count = 0
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent_frame_id or self.odom_frame
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = position[0]
        tf_msg.transform.translation.y = position[1]
        tf_msg.transform.translation.z = position[2]
        tf_msg.transform.rotation.x = orientation[0]
        tf_msg.transform.rotation.y = orientation[1]
        tf_msg.transform.rotation.z = orientation[2]
        tf_msg.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
