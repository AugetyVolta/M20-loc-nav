import json
import math
import socket
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class StairGaitManager(Node):
    def __init__(self):
        super().__init__("stair_gait_manager")

        self.declare_parameter("enabled", True)
        self.declare_parameter("stair_state_topic", "/pct_stair_state")
        self.declare_parameter("odom_topic", "/odom_body")
        self.declare_parameter("udp_ip", "10.21.31.103")
        self.declare_parameter("udp_port", 30000)
        self.declare_parameter("udp_msg_id", 1)
        self.declare_parameter("require_stationary", True)
        self.declare_parameter("stationary_linear_threshold", 0.05)
        self.declare_parameter("stationary_angular_threshold", 0.10)
        self.declare_parameter("stationary_hold_time", 0.3)
        self.declare_parameter("switch_cooldown", 3.0)
        self.declare_parameter("pause_nav_cmd_enabled", True)
        self.declare_parameter("pause_nav_cmd_topic", "/stair_gait_pause_nav_cmd")
        self.declare_parameter("pause_before_switch_time", 0.6)
        self.declare_parameter("pause_after_switch_time", 1.0)
        self.declare_parameter("initial_gait_param", 4097)
        self.declare_parameter("flat_gait_param", 4097)
        self.declare_parameter("stair_up_gait_param", 4097)
        self.declare_parameter("stair_down_gait_param", 4099)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.udp_ip = str(self.get_parameter("udp_ip").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.udp_msg_id = int(self.get_parameter("udp_msg_id").value) & 0xFFFF
        self.require_stationary = bool(self.get_parameter("require_stationary").value)
        self.stationary_linear_threshold = max(
            0.0, float(self.get_parameter("stationary_linear_threshold").value)
        )
        self.stationary_angular_threshold = max(
            0.0, float(self.get_parameter("stationary_angular_threshold").value)
        )
        self.stationary_hold_time = max(0.0, float(self.get_parameter("stationary_hold_time").value))
        self.switch_cooldown = max(0.0, float(self.get_parameter("switch_cooldown").value))
        self.pause_nav_cmd_enabled = bool(self.get_parameter("pause_nav_cmd_enabled").value)
        self.pause_nav_cmd_topic = str(self.get_parameter("pause_nav_cmd_topic").value)
        self.pause_before_switch_time = max(0.0, float(self.get_parameter("pause_before_switch_time").value))
        self.pause_after_switch_time = max(0.0, float(self.get_parameter("pause_after_switch_time").value))
        self.initial_gait_param = int(self.get_parameter("initial_gait_param").value)
        self.flat_gait_param = int(self.get_parameter("flat_gait_param").value)
        self.stair_up_gait_param = int(self.get_parameter("stair_up_gait_param").value)
        self.stair_down_gait_param = int(self.get_parameter("stair_down_gait_param").value)

        self.current_state = "flat"
        self.pending_gait_param = None
        self.pending_state = None
        self.last_sent_gait_param = self.initial_gait_param if self.initial_gait_param > 0 else None
        self.last_send_wall_time = 0.0
        self.last_odom_wall_time = 0.0
        self.stationary_since = time.monotonic()
        self.is_stationary = False
        self.last_odom_pose = None
        self.last_odom_pose_wall_time = None
        self.last_motion_linear = float("inf")
        self.last_motion_angular = float("inf")
        self.pause_nav_cmd_active = False
        self.pause_started_wall_time = None
        self.pause_release_wall_time = None

        stair_state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pause_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stair_state_topic").value),
            self._on_stair_state,
            stair_state_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            20,
        )
        self.pause_nav_cmd_pub = self.create_publisher(Bool, self.pause_nav_cmd_topic, pause_qos)
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info(
            "Stair gait manager ready: "
            f"enabled={int(self.enabled)}, require_stationary={int(self.require_stationary)}, "
            f"flat={self.flat_gait_param}, up={self.stair_up_gait_param}, "
            f"down={self.stair_down_gait_param}, udp={self.udp_ip}:{self.udp_port}, "
            f"pause_nav_cmd={int(self.pause_nav_cmd_enabled)}"
        )

    def _on_stair_state(self, msg):
        state = str(msg.data).strip()
        if state not in ("flat", "stair_up", "stair_down"):
            self.get_logger().warn(f"Ignore unknown stair state: {state}")
            return
        if state == self.current_state:
            return
        self.current_state = state
        gait_param = self._gait_param_for_state(state)
        if gait_param <= 0 or gait_param == self.last_sent_gait_param:
            self.pending_gait_param = None
            self.pending_state = None
            self._schedule_pause_release(0.0)
            return
        self.pending_gait_param = gait_param
        self.pending_state = state
        self._set_pause_nav_cmd(True, "gait switch pending")
        self.get_logger().info(f"Queue gait switch: state={state}, GaitParam={gait_param}")

    def _on_odom(self, msg):
        twist = msg.twist.twist
        # Gait switching only requires the dog to stop translating and yawing
        # on the ground. Body-height and roll/pitch vibration must not keep the
        # stationary gate open indefinitely.
        twist_linear = math.hypot(twist.linear.x, twist.linear.y)
        twist_angular = abs(twist.angular.z)
        now = time.monotonic()
        self.last_odom_wall_time = now
        pose_linear, pose_angular = self._odom_pose_velocity(msg, now)
        linear = max(twist_linear, pose_linear)
        angular = max(twist_angular, pose_angular)
        self.last_motion_linear = linear
        self.last_motion_angular = angular
        stationary_now = (
            linear <= self.stationary_linear_threshold
            and angular <= self.stationary_angular_threshold
        )
        if stationary_now:
            if not self.is_stationary:
                self.stationary_since = now
            self.is_stationary = True
        else:
            self.is_stationary = False
            self.stationary_since = None

    def _timer_cb(self):
        now = time.monotonic()
        self._refresh_pause_nav_cmd(now)
        if not self.enabled or self.pending_gait_param is None:
            return
        if now - self.last_send_wall_time < self.switch_cooldown:
            return
        if not self._pause_before_switch_ready(now):
            return
        if self.require_stationary and not self._stationary_ready(now):
            self.get_logger().info(
                f"Waiting stationary before gait switch: state={self.pending_state}, "
                f"GaitParam={self.pending_gait_param}, "
                f"linear={self.last_motion_linear:.3f}m/s "
                f"(limit={self.stationary_linear_threshold:.3f}), "
                f"yaw_rate={self.last_motion_angular:.3f}rad/s "
                f"(limit={self.stationary_angular_threshold:.3f})",
                throttle_duration_sec=2.0,
            )
            return
        gait_param = self.pending_gait_param
        state = self.pending_state or self.current_state
        if self._send_gait_param_udp(gait_param):
            self.last_sent_gait_param = gait_param
            self.last_send_wall_time = now
            self.pending_gait_param = None
            self.pending_state = None
            self._schedule_pause_release(self.pause_after_switch_time)
            self.get_logger().info(f"Sent gait switch: state={state}, GaitParam={gait_param}")

    def _pause_before_switch_ready(self, now):
        if not self.pause_nav_cmd_enabled or self.pause_before_switch_time <= 0.0:
            return True
        if self.pause_started_wall_time is None:
            self._set_pause_nav_cmd(True, "gait switch pending")
            return False
        return now - self.pause_started_wall_time >= self.pause_before_switch_time

    def _stationary_ready(self, now):
        if self.last_odom_wall_time <= 0.0:
            return False
        if now - self.last_odom_wall_time > 1.0:
            return False
        if not self.is_stationary or self.stationary_since is None:
            return False
        return now - self.stationary_since >= self.stationary_hold_time

    def _refresh_pause_nav_cmd(self, now):
        if not self.pause_nav_cmd_enabled:
            return
        if self.pause_nav_cmd_active:
            self._publish_pause_nav_cmd(True)
        if (
            self.pause_nav_cmd_active
            and self.pending_gait_param is None
            and self.pause_release_wall_time is not None
            and now >= self.pause_release_wall_time
        ):
            self._set_pause_nav_cmd(False, "gait switch complete")

    def _set_pause_nav_cmd(self, active, reason):
        if not self.pause_nav_cmd_enabled:
            return
        now = time.monotonic()
        if active:
            if not self.pause_nav_cmd_active:
                self.get_logger().info(f"Pause NAV_CMD output: {reason}")
                self.pause_started_wall_time = now
            self.pause_nav_cmd_active = True
            self.pause_release_wall_time = None
            self._publish_pause_nav_cmd(True)
        else:
            if self.pause_nav_cmd_active:
                self.get_logger().info(f"Release NAV_CMD output pause: {reason}")
            self.pause_nav_cmd_active = False
            self.pause_started_wall_time = None
            self.pause_release_wall_time = None
            self._publish_pause_nav_cmd(False)

    def _schedule_pause_release(self, delay):
        if not self.pause_nav_cmd_enabled or not self.pause_nav_cmd_active:
            return
        self.pause_release_wall_time = time.monotonic() + max(0.0, float(delay))

    def _publish_pause_nav_cmd(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.pause_nav_cmd_pub.publish(msg)

    def _odom_pose_velocity(self, msg, now):
        pose = msg.pose.pose
        current = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            self._yaw_from_quaternion(pose.orientation),
        )
        if self.last_odom_pose is None or self.last_odom_pose_wall_time is None:
            self.last_odom_pose = current
            self.last_odom_pose_wall_time = now
            return 0.0, 0.0

        dt = now - self.last_odom_pose_wall_time
        previous = self.last_odom_pose
        self.last_odom_pose = current
        self.last_odom_pose_wall_time = now
        if dt <= 1.0e-3:
            return 0.0, 0.0

        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        dyaw = self._wrap_angle(current[3] - previous[3])
        linear = math.hypot(dx, dy) / dt
        angular = abs(dyaw) / dt
        return linear, angular

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _gait_param_for_state(self, state):
        if state == "stair_down":
            return self.stair_down_gait_param
        if state == "stair_up":
            return self.stair_up_gait_param
        return self.flat_gait_param

    def _send_gait_param_udp(self, gait_param):
        payload = {
            "PatrolDevice": {
                "Type": 2,
                "Command": 23,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                "Items": {
                    "GaitParam": int(gait_param),
                },
            }
        }
        asdu_data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        header = self._build_protocol_header(
            data_length=len(asdu_data),
            msg_id=self.udp_msg_id,
            asdu_format=0x01,
        )
        self.udp_msg_id = (self.udp_msg_id + 1) & 0xFFFF
        if self.udp_msg_id == 0:
            self.udp_msg_id = 1
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(header + asdu_data, (self.udp_ip, self.udp_port))
            return True
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send GaitParam={gait_param} to {self.udp_ip}:{self.udp_port}: {exc}"
            )
            return False

    @staticmethod
    def _build_protocol_header(data_length, msg_id=1, asdu_format=0x01):
        if not 0 <= int(data_length) <= 65535:
            raise ValueError("data_length must be in [0, 65535]")
        if not 0 <= int(msg_id) <= 65535:
            raise ValueError("msg_id must be in [0, 65535]")
        if int(asdu_format) not in (0x00, 0x01):
            raise ValueError("asdu_format must be XML(0x00) or JSON(0x01)")
        header = bytearray(16)
        header[0] = 0xEB
        header[1] = 0x91
        header[2] = 0xEB
        header[3] = 0x90
        header[4] = int(data_length) & 0xFF
        header[5] = (int(data_length) >> 8) & 0xFF
        header[6] = int(msg_id) & 0xFF
        header[7] = (int(msg_id) >> 8) & 0xFF
        header[8] = int(asdu_format)
        return header


def main(args=None):
    rclpy.init(args=args)
    node = StairGaitManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
