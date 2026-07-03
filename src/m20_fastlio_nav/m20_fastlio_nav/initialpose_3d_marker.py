#!/usr/bin/env python3

import math
import copy
import threading
import time
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class InitialPose3DMarker(Node):
    def __init__(self):
        super().__init__("initialpose_3d_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("marker_namespace", "initialpose_3d_marker")
        self.declare_parameter("marker_name", "initialpose_3d")
        self.declare_parameter("publish_topic", "/initialpose")
        self.declare_parameter("odom_topic", "/Odometry_loc")
        self.declare_parameter("marker_scale", 1.0)
        self.declare_parameter("sync_from_tf_on_start", False)
        self.declare_parameter("republish_until_first_odom", False)
        self.declare_parameter("republish_period_sec", 0.5)
        self.declare_parameter("republish_timeout_sec", 30.0)
        self.declare_parameter("publish_pose_update_before_odom", True)
        self.declare_parameter("pose_update_publish_period_sec", 0.2)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.marker_name = str(self.get_parameter("marker_name").value)
        self.publish_topic = str(self.get_parameter("publish_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.sync_from_tf_on_start = bool(self.get_parameter("sync_from_tf_on_start").value)
        self.republish_until_first_odom = bool(self.get_parameter("republish_until_first_odom").value)
        self.republish_period_sec = float(self.get_parameter("republish_period_sec").value)
        self.republish_timeout_sec = float(self.get_parameter("republish_timeout_sec").value)
        self.publish_pose_update_before_odom = bool(self.get_parameter("publish_pose_update_before_odom").value)
        self.pose_update_publish_period_sec = float(self.get_parameter("pose_update_publish_period_sec").value)
        self.manual_pose_set = False
        self.odom_seen = False
        self.last_pose_update_publish_wall = 0.0
        self.republish_stop_event = threading.Event()
        self.republish_thread = None

        namespace = str(self.get_parameter("marker_namespace").value)
        initialpose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, self.publish_topic, initialpose_qos)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.marker_server = InteractiveMarkerServer(self, namespace)
        self.tf_buffer = None
        self.tf_listener = None

        self.current_pose = self._pose_from_list(self.get_parameter("initial_pose").value)
        self._make_marker()
        self.marker_server.applyChanges()

        self.tf_sync_timer = None
        self.tf_sync_attempts = 0
        if self.sync_from_tf_on_start:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_sync_timer = self.create_timer(0.5, self._try_sync_from_tf)
        self.get_logger().info(
            f"3D initial pose marker ready: namespace=/{namespace}, publishes {self.publish_topic}. "
            f"Drag or rotate it in RViz and release the mouse to publish. "
            f"The last published pose is kept with transient-local QoS for pre-bag initialization."
        )

    def _pose_from_list(self, values: Iterable[float]) -> Pose:
        vals = list(values)
        if len(vals) != 7:
            self.get_logger().warn("initial_pose must be [x, y, z, qx, qy, qz, qw], use identity pose")
            vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        pose = Pose()
        pose.position.x = float(vals[0])
        pose.position.y = float(vals[1])
        pose.position.z = float(vals[2])
        pose.orientation.x = float(vals[3])
        pose.orientation.y = float(vals[4])
        pose.orientation.z = float(vals[5])
        pose.orientation.w = float(vals[6])
        self._normalize_pose_orientation(pose)
        return pose

    def _try_sync_from_tf(self):
        if not self.sync_from_tf_on_start or self.manual_pose_set:
            self.tf_sync_timer.cancel()
            return
        self.tf_sync_attempts += 1
        if self.tf_sync_attempts > 30:
            self.get_logger().warn(
                f"Could not initialize marker from TF {self.frame_id}->{self.base_frame}; keep parameter initial_pose"
            )
            self.tf_sync_timer.cancel()
            return
        try:
            if self.tf_buffer is None:
                return
            tf_msg = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, rclpy.time.Time())
        except TransformException:
            return

        pose = Pose()
        pose.position.x = tf_msg.transform.translation.x
        pose.position.y = tf_msg.transform.translation.y
        pose.position.z = tf_msg.transform.translation.z
        pose.orientation = tf_msg.transform.rotation
        self._normalize_pose_orientation(pose)
        self.current_pose = pose
        self.marker_server.setPose(self.marker_name, self.current_pose)
        self.marker_server.applyChanges()
        self.tf_sync_timer.cancel()
        self.get_logger().info(
            f"Initialized 3D initial pose marker from TF {self.frame_id}->{self.base_frame}: "
            f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
        )

    def _make_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = self.marker_name
        int_marker.description = "3D initial pose"
        int_marker.scale = self.marker_scale
        int_marker.pose = self.current_pose

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(self._make_pose_sphere())
        int_marker.controls.append(visual_control)

        axes = [
            ("move_x", InteractiveMarkerControl.MOVE_AXIS, (1.0, 1.0, 0.0, 0.0)),
            ("rotate_z", InteractiveMarkerControl.ROTATE_AXIS, (1.0, 0.0, 1.0, 0.0)),
            ("move_z", InteractiveMarkerControl.MOVE_AXIS, (1.0, 0.0, 1.0, 0.0)),
            ("move_y", InteractiveMarkerControl.MOVE_AXIS, (1.0, 0.0, 0.0, 1.0)),
        ]
        for control_name, mode, orientation in axes:
            control = InteractiveMarkerControl()
            control.name = control_name
            control.interaction_mode = mode
            control.orientation_mode = InteractiveMarkerControl.FIXED
            self._set_normalized_orientation(control, orientation)
            int_marker.controls.append(control)

        self.marker_server.insert(int_marker, feedback_callback=self._process_feedback)

    def _make_pose_sphere(self) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.05
        marker.color.a = 0.95
        return marker

    def _process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.marker_name != self.marker_name:
            return
        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_DOWN,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.BUTTON_CLICK,
        ):
            self._stop_startup_tf_sync()
        self.current_pose = feedback.pose
        self._normalize_pose_orientation(self.current_pose)
        if feedback.event_type in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            self.marker_server.setPose(self.marker_name, self.current_pose)
            self.marker_server.applyChanges()
        if (
            feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE
            and self.publish_pose_update_before_odom
            and not self.odom_seen
        ):
            now_wall = time.monotonic()
            period = max(0.05, self.pose_update_publish_period_sec)
            if now_wall - self.last_pose_update_publish_wall >= period:
                self.last_pose_update_publish_wall = now_wall
                self._publish_initialpose()
        if feedback.event_type in (InteractiveMarkerFeedback.MOUSE_UP, InteractiveMarkerFeedback.BUTTON_CLICK):
            self._publish_initialpose()

    def _stop_startup_tf_sync(self):
        if self.manual_pose_set:
            return
        self.manual_pose_set = True
        if self.tf_sync_timer is not None:
            self.tf_sync_timer.cancel()
        self.get_logger().info("Manual marker edit detected; startup TF sync disabled")

    def _publish_initialpose(self):
        msg = self._make_initialpose_msg()
        self.pub_initialpose.publish(msg)
        self._start_initialpose_republish(msg)
        self.get_logger().warn(
            f"Published {self.publish_topic}: x={msg.pose.pose.position.x:.3f}, "
            f"y={msg.pose.pose.position.y:.3f}, z={msg.pose.pose.position.z:.3f}, "
            f"q=({msg.pose.pose.orientation.x:.4f}, {msg.pose.pose.orientation.y:.4f}, "
            f"{msg.pose.pose.orientation.z:.4f}, {msg.pose.pose.orientation.w:.4f})"
        )

    def _make_initialpose_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose = self.current_pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[14] = 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        return msg

    def _odom_callback(self, _msg: Odometry):
        if self.odom_seen:
            return
        self.odom_seen = True
        self.republish_stop_event.set()
        self.get_logger().info(f"Received first {self.odom_topic}; stop initial pose republishing")

    def _start_initialpose_republish(self, msg: PoseWithCovarianceStamped):
        if not self.republish_until_first_odom or self.odom_seen:
            return
        self.republish_stop_event.set()
        if self.republish_thread is not None and self.republish_thread.is_alive():
            self.republish_thread.join(timeout=0.2)
        self.republish_stop_event = threading.Event()
        self.republish_thread = threading.Thread(
            target=self._republish_initialpose_until_odom,
            args=(copy.deepcopy(msg), self.republish_stop_event),
            daemon=True,
        )
        self.republish_thread.start()

    def _republish_initialpose_until_odom(self, msg: PoseWithCovarianceStamped, stop_event: threading.Event):
        deadline = time.monotonic() + max(0.0, self.republish_timeout_sec)
        period = max(0.05, self.republish_period_sec)
        while time.monotonic() < deadline and not self.odom_seen:
            if stop_event.wait(period):
                break
            repeat_msg = copy.deepcopy(msg)
            repeat_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_initialpose.publish(repeat_msg)
        if not self.odom_seen and not stop_event.is_set():
            self.get_logger().warn(
                f"Stopped republishing {self.publish_topic} after {self.republish_timeout_sec:.1f}s "
                f"without receiving {self.odom_topic}"
            )

    def destroy_node(self):
        self.republish_stop_event.set()
        if self.republish_thread is not None and self.republish_thread.is_alive():
            self.republish_thread.join(timeout=0.5)
        return super().destroy_node()

    @staticmethod
    def _normalize_pose_orientation(pose: Pose):
        q = pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1.0e-9 or not math.isfinite(norm):
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            q.w = 1.0
            return
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm

    @staticmethod
    def _set_normalized_orientation(control: InteractiveMarkerControl, values):
        w, x, y, z = values
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        control.orientation.w = w / norm
        control.orientation.x = x / norm
        control.orientation.y = y / norm
        control.orientation.z = z / norm


def main(args=None):
    rclpy.init(args=args)
    node = InitialPose3DMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
