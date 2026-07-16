import math
import struct
from typing import Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def vector_param(value: Iterable[float], expected_len: int, name: str) -> Tuple[float, ...]:
    data = tuple(float(v) for v in value)
    if len(data) != expected_len:
        raise ValueError(f"{name} must have {expected_len} values, got {len(data)}")
    return data


def normalize_quat(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    qx, qy, qz, qw = normalize_quat(q)
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def transform_point(tf_msg: TransformStamped, point: Vector3) -> Vector3:
    q = tf_msg.transform.rotation
    rotated = rotate_vector((q.x, q.y, q.z, q.w), point)
    return (
        rotated[0] + tf_msg.transform.translation.x,
        rotated[1] + tf_msg.transform.translation.y,
        rotated[2] + tf_msg.transform.translation.z,
    )


class EgoOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("ego_odom_bridge")
        self.declare_parameter("source_odom_topic", "/odom_body")
        self.declare_parameter("output_odom_topic", "/ego_odom")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout", 0.03)
        self.declare_parameter("publish_rate", 30.0)

        self.source_odom_topic = str(self.get_parameter("source_odom_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value).lstrip("/")
        self.base_frame = str(self.get_parameter("base_frame").value).lstrip("/")
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.latest_odom: Optional[Odometry] = None
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.sub = self.create_subscription(Odometry, self.source_odom_topic, self._on_odom, odom_qos)
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self._on_timer)
        self.get_logger().info(
            f"EGO odom bridge: {self.source_odom_topic} + TF "
            f"{self.output_frame}<-{self.base_frame} -> {self.output_odom_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self._publish_from_tf(msg)

    def _on_timer(self) -> None:
        self._publish_from_tf(self.latest_odom)

    def _publish_from_tf(self, msg: Optional[Odometry]) -> None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(f"TF lookup failed: {self.output_frame}<-{self.base_frame}: {exc}")
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp if msg is not None else tf_msg.header.stamp
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = tf_msg.transform.translation.x
        out.pose.pose.position.y = tf_msg.transform.translation.y
        out.pose.pose.position.z = tf_msg.transform.translation.z
        out.pose.pose.orientation = tf_msg.transform.rotation
        if msg is not None:
            out.pose.covariance = msg.pose.covariance

            q = tf_msg.transform.rotation
            linear_world = rotate_vector(
                (q.x, q.y, q.z, q.w),
                (
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ),
            )
            out.twist.twist.linear.x = linear_world[0]
            out.twist.twist.linear.y = linear_world[1]
            out.twist.twist.linear.z = linear_world[2]
            out.twist.twist.angular = msg.twist.twist.angular
            out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)


class EgoCloudFrameBridge(Node):
    def __init__(self) -> None:
        super().__init__("ego_cloud_frame_bridge")
        self.declare_parameter("input_cloud_topic", "/cloud_registered_body_1")
        self.declare_parameter("output_cloud_topic", "/ego_cloud")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("input_frame_override", "body")
        self.declare_parameter("max_points", 25000)
        self.declare_parameter("self_filter_radius", 0.45)
        self.declare_parameter("self_filter_z_min", -0.70)
        self.declare_parameter("self_filter_z_max", 0.90)
        self.declare_parameter("tf_timeout", 0.03)
        self.declare_parameter(
            "base_to_body_translation",
            [0.32713234, 0.01413551, 0.31238696],
        )
        self.declare_parameter(
            "base_to_body_quaternion",
            [-0.00394028, 0.24367785, 0.00970223, 0.96979969],
        )

        self.input_cloud_topic = str(self.get_parameter("input_cloud_topic").value)
        self.output_cloud_topic = str(self.get_parameter("output_cloud_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value).lstrip("/")
        self.base_frame = str(self.get_parameter("base_frame").value).lstrip("/")
        self.body_frame = str(self.get_parameter("body_frame").value).lstrip("/")
        self.input_frame_override = str(self.get_parameter("input_frame_override").value).lstrip("/")
        self.max_points = int(self.get_parameter("max_points").value)
        self.self_filter_radius = float(self.get_parameter("self_filter_radius").value)
        self.self_filter_z_min = float(self.get_parameter("self_filter_z_min").value)
        self.self_filter_z_max = float(self.get_parameter("self_filter_z_max").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.t_base_body = vector_param(
            self.get_parameter("base_to_body_translation").value,
            3,
            "base_to_body_translation",
        )
        self.q_base_body = normalize_quat(
            vector_param(
                self.get_parameter("base_to_body_quaternion").value,
                4,
                "base_to_body_quaternion",
            )
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.pub = self.create_publisher(PointCloud2, self.output_cloud_topic, 2)
        self.sub = self.create_subscription(PointCloud2, self.input_cloud_topic, self._on_cloud, qos)
        self.get_logger().info(
            f"EGO cloud bridge: {self.input_cloud_topic} -> {self.output_cloud_topic} "
            f"({self.output_frame}), self_filter_radius={self.self_filter_radius:.2f}m"
        )

    @staticmethod
    def _field_offsets(msg: PointCloud2):
        offsets = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z") and field.datatype == PointField.FLOAT32:
                offsets[field.name] = field.offset
        return offsets if {"x", "y", "z"} <= set(offsets) else None

    def _body_to_base(self, point: Vector3) -> Vector3:
        rotated = rotate_vector(self.q_base_body, point)
        return (
            rotated[0] + self.t_base_body[0],
            rotated[1] + self.t_base_body[1],
            rotated[2] + self.t_base_body[2],
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        offsets = self._field_offsets(msg)
        if offsets is None:
            self.get_logger().warn("Input cloud lacks float32 x/y/z fields", throttle_duration_sec=2.0)
            return

        try:
            tf_base_to_output = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(f"TF lookup failed: {self.output_frame}<-{self.base_frame}: {exc}")
            return

        input_frame = self.input_frame_override or msg.header.frame_id.lstrip("/")
        source_is_body = input_frame == self.body_frame
        source_is_base = input_frame == self.base_frame
        if not source_is_body and not source_is_base:
            self.get_logger().warn(
                f"Unsupported input frame {input_frame}; expected {self.body_frame} or {self.base_frame}",
                throttle_duration_sec=2.0,
            )
            return

        total = int(msg.width) * int(msg.height)
        if total <= 0:
            return
        stride = max(1, math.ceil(total / self.max_points)) if self.max_points > 0 else 1

        endian = ">" if msg.is_bigendian else "<"
        unpack = struct.Struct(endian + "f").unpack_from
        output = bytearray()
        for idx in range(0, total, stride):
            row = idx // msg.width
            col = idx % msg.width
            offset = row * msg.row_step + col * msg.point_step
            x = unpack(msg.data, offset + offsets["x"])[0]
            y = unpack(msg.data, offset + offsets["y"])[0]
            z = unpack(msg.data, offset + offsets["z"])[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            point = (x, y, z)
            if source_is_body:
                point = self._body_to_base(point)

            # Fast-LIO body-frame clouds can contain near-body returns from the robot
            # itself. If passed to EGO unchanged, the inflated occupancy map may mark
            # the current base position as occupied and prevent every replan.
            base_xy_dist = math.hypot(point[0], point[1])
            if (
                self.self_filter_radius > 0.0
                and base_xy_dist < self.self_filter_radius
                and self.self_filter_z_min <= point[2] <= self.self_filter_z_max
            ):
                continue

            point = transform_point(tf_base_to_output, point)
            output.extend(struct.pack("<fff", point[0], point[1], point[2]))

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.height = 1
        out.width = len(output) // 12
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = out.point_step * out.width
        out.is_dense = True
        out.data = bytes(output)
        self.pub.publish(out)


def ego_odom_bridge_main(args=None):
    rclpy.init(args=args)
    node = EgoOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def ego_cloud_frame_bridge_main(args=None):
    rclpy.init(args=args)
    node = EgoCloudFrameBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
