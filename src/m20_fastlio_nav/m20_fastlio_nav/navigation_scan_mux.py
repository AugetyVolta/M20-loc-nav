import time

import rclpy
from m20_navigation_msgs.msg import NavigationMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class NavigationScanMux(Node):
    def __init__(self):
        super().__init__("navigation_scan_mux")

        self.declare_parameter("raw_scan_topic", "/scan")
        self.declare_parameter("traversability_scan_topic", "/traversability_filtered_scan")
        self.declare_parameter("output_scan_topic", "/navigation_scan")
        self.declare_parameter("navigation_mode_topic", "/navigation_mode")
        self.declare_parameter("initial_scan_profile", "raw")
        self.declare_parameter("mode_timeout", 2.0)

        initial_profile = str(self.get_parameter("initial_scan_profile").value).strip().lower()
        self.scan_profile = (
            NavigationMode.SCAN_TRAVERSABILITY
            if initial_profile == "traversability"
            else NavigationMode.SCAN_RAW
        )
        self.mode_timeout = max(0.1, float(self.get_parameter("mode_timeout").value))
        self.last_mode_wall_time = None
        self.last_logged_profile = None

        mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            NavigationMode,
            str(self.get_parameter("navigation_mode_topic").value),
            self._on_mode,
            mode_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("raw_scan_topic").value),
            self._on_raw_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("traversability_scan_topic").value),
            self._on_traversability_scan,
            qos_profile_sensor_data,
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_scan_topic").value),
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._check_mode_timeout)

        self.get_logger().info(
            "Navigation scan mux ready: "
            f"raw={self.get_parameter('raw_scan_topic').value}, "
            f"traversability={self.get_parameter('traversability_scan_topic').value}, "
            f"output={self.get_parameter('output_scan_topic').value}"
        )

    def _on_mode(self, msg):
        self.last_mode_wall_time = time.monotonic()
        if msg.scan_profile not in (
            NavigationMode.SCAN_RAW,
            NavigationMode.SCAN_TRAVERSABILITY,
        ):
            self.get_logger().warn(f"Ignore unknown scan profile: {msg.scan_profile}")
            return
        self.scan_profile = msg.scan_profile
        self._log_profile()

    def _on_raw_scan(self, msg):
        if self.scan_profile == NavigationMode.SCAN_RAW:
            self.scan_pub.publish(msg)

    def _on_traversability_scan(self, msg):
        if self.scan_profile == NavigationMode.SCAN_TRAVERSABILITY:
            self.scan_pub.publish(msg)

    def _check_mode_timeout(self):
        if self.last_mode_wall_time is None:
            return
        age = time.monotonic() - self.last_mode_wall_time
        if age > self.mode_timeout:
            self.get_logger().warn(
                f"Navigation mode is stale ({age:.2f}s); holding scan profile",
                throttle_duration_sec=2.0,
            )

    def _log_profile(self):
        if self.scan_profile == self.last_logged_profile:
            return
        self.last_logged_profile = self.scan_profile
        profile = (
            "traversability"
            if self.scan_profile == NavigationMode.SCAN_TRAVERSABILITY
            else "raw"
        )
        self.get_logger().info(f"Navigation scan profile switched to {profile}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationScanMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
