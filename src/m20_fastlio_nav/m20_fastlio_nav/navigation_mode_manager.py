import math
import time

import rclpy
from m20_navigation_msgs.msg import NavigationMode, TerrainState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class NavigationModeManager(Node):
    def __init__(self):
        super().__init__("navigation_mode_manager")

        self.declare_parameter("terrain_state_topic", "/terrain/state")
        self.declare_parameter("navigation_mode_topic", "/navigation_mode")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("pct_dynamic_guard_distance", 5.0)
        self.declare_parameter("costmap_stair_pre_distance", 4.0)
        self.declare_parameter("scan_stair_pre_distance", 4.0)
        self.declare_parameter("heading_guard_pre_distance", 3.0)
        self.declare_parameter("gait_switch_pre_distance", 3.0)

        self.state_timeout = max(0.1, float(self.get_parameter("state_timeout").value))
        self.pct_dynamic_guard_distance = max(
            0.0,
            float(self.get_parameter("pct_dynamic_guard_distance").value),
        )
        self.costmap_stair_pre_distance = max(
            0.0,
            float(self.get_parameter("costmap_stair_pre_distance").value),
        )
        self.scan_stair_pre_distance = max(
            0.0,
            float(self.get_parameter("scan_stair_pre_distance").value),
        )
        self.heading_guard_pre_distance = max(
            0.0,
            float(self.get_parameter("heading_guard_pre_distance").value),
        )
        self.gait_switch_pre_distance = max(
            0.0,
            float(self.get_parameter("gait_switch_pre_distance").value),
        )

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.mode_pub = self.create_publisher(
            NavigationMode,
            str(self.get_parameter("navigation_mode_topic").value),
            latched_qos,
        )
        self.create_subscription(
            TerrainState,
            str(self.get_parameter("terrain_state_topic").value),
            self._on_terrain_state,
            latched_qos,
        )

        self.terrain_state = None
        self.terrain_state_wall_time = None
        self.last_mode = None
        self.last_mode_key = None

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            "Navigation mode manager ready: "
            f"dynamic_guard={self.pct_dynamic_guard_distance:.2f}m, "
            f"gait={self.gait_switch_pre_distance:.2f}m, "
            f"costmap={self.costmap_stair_pre_distance:.2f}m, "
            f"scan={self.scan_stair_pre_distance:.2f}m, "
            f"heading={self.heading_guard_pre_distance:.2f}m"
        )

    def _on_terrain_state(self, msg):
        self.terrain_state = msg
        self.terrain_state_wall_time = time.monotonic()

    def _timer_cb(self):
        now = time.monotonic()
        age = math.inf
        if self.terrain_state_wall_time is not None:
            age = now - self.terrain_state_wall_time

        if (
            self.terrain_state is not None
            and self.terrain_state.valid
            and age <= self.state_timeout
        ):
            mode = self._mode_from_terrain(self.terrain_state)
        elif self.last_mode is not None:
            mode = self._copy_mode(self.last_mode)
            mode.header.stamp = self.get_clock().now().to_msg()
            mode.terrain_valid = False
            mode.reason = f"holding_last_mode:terrain_age={age:.2f}"
        else:
            mode = self._safe_startup_mode(age)

        self.last_mode = mode
        self.mode_pub.publish(mode)
        self._log_transition(mode)

    def _mode_from_terrain(self, state):
        mode = NavigationMode()
        mode.header = state.header
        mode.current_terrain = state.current_mode
        mode.upcoming_terrain = state.upcoming_mode
        mode.distance_to_entry = state.distance_to_entry
        mode.terrain_valid = state.valid
        mode.route_id = state.route_id

        current_stair = self._is_stair(state.current_mode)
        upcoming_stair = self._is_stair(state.upcoming_mode)
        distance = float(state.distance_to_entry)

        dynamic_guard = current_stair or (
            upcoming_stair and distance <= self.pct_dynamic_guard_distance
        )
        costmap_stair = current_stair or (
            upcoming_stair and distance <= self.costmap_stair_pre_distance
        )
        scan_stair = current_stair or (
            upcoming_stair and distance <= self.scan_stair_pre_distance
        )
        heading_guard = current_stair or (
            upcoming_stair and distance <= self.heading_guard_pre_distance
        )
        gait_stair = current_stair or (
            upcoming_stair and distance <= self.gait_switch_pre_distance
        )

        mode.pct_dynamic_enabled = not dynamic_guard
        mode.costmap_profile = (
            NavigationMode.PROFILE_STAIR
            if costmap_stair
            else NavigationMode.PROFILE_FLAT
        )
        mode.scan_profile = (
            NavigationMode.SCAN_TRAVERSABILITY
            if scan_stair
            else NavigationMode.SCAN_RAW
        )
        mode.heading_guard_enabled = heading_guard
        if current_stair:
            mode.gait_terrain = state.current_mode
        elif gait_stair:
            mode.gait_terrain = state.upcoming_mode
        else:
            mode.gait_terrain = TerrainState.FLAT
        mode.reason = (
            f"terrain={self._mode_name(state.current_mode)},"
            f"upcoming={self._mode_name(state.upcoming_mode)},"
            f"entry={distance:.2f}"
        )
        return mode

    def _safe_startup_mode(self, age):
        mode = NavigationMode()
        mode.header.stamp = self.get_clock().now().to_msg()
        mode.current_terrain = NavigationMode.TERRAIN_UNKNOWN
        mode.upcoming_terrain = NavigationMode.TERRAIN_UNKNOWN
        mode.distance_to_entry = math.inf
        mode.terrain_valid = False
        mode.route_id = 0
        mode.pct_dynamic_enabled = True
        mode.costmap_profile = NavigationMode.PROFILE_FLAT
        mode.scan_profile = NavigationMode.SCAN_RAW
        mode.heading_guard_enabled = False
        mode.gait_terrain = NavigationMode.TERRAIN_FLAT
        mode.reason = f"startup_default:terrain_age={age:.2f}"
        return mode

    @staticmethod
    def _copy_mode(source):
        mode = NavigationMode()
        mode.header = source.header
        mode.current_terrain = source.current_terrain
        mode.upcoming_terrain = source.upcoming_terrain
        mode.distance_to_entry = source.distance_to_entry
        mode.terrain_valid = source.terrain_valid
        mode.route_id = source.route_id
        mode.pct_dynamic_enabled = source.pct_dynamic_enabled
        mode.costmap_profile = source.costmap_profile
        mode.scan_profile = source.scan_profile
        mode.heading_guard_enabled = source.heading_guard_enabled
        mode.gait_terrain = source.gait_terrain
        mode.reason = source.reason
        return mode

    def _log_transition(self, mode):
        key = (
            mode.current_terrain,
            mode.upcoming_terrain,
            mode.pct_dynamic_enabled,
            mode.costmap_profile,
            mode.scan_profile,
            mode.heading_guard_enabled,
            mode.gait_terrain,
            mode.terrain_valid,
        )
        if key == self.last_mode_key:
            return
        self.last_mode_key = key
        self.get_logger().info(
            "Navigation mode: "
            f"current={self._mode_name(mode.current_terrain)}, "
            f"upcoming={self._mode_name(mode.upcoming_terrain)}, "
            f"entry={mode.distance_to_entry:.2f}m, "
            f"pct_dynamic={int(mode.pct_dynamic_enabled)}, "
            f"costmap={mode.costmap_profile}, scan={mode.scan_profile}, "
            f"heading_guard={int(mode.heading_guard_enabled)}, "
            f"gait={self._mode_name(mode.gait_terrain)}, "
            f"valid={int(mode.terrain_valid)}"
        )

    @staticmethod
    def _is_stair(mode):
        return mode in (TerrainState.STAIR_UP, TerrainState.STAIR_DOWN)

    @staticmethod
    def _mode_name(mode):
        return {
            TerrainState.FLAT: "flat",
            TerrainState.STAIR_UP: "stair_up",
            TerrainState.STAIR_DOWN: "stair_down",
        }.get(mode, "unknown")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
