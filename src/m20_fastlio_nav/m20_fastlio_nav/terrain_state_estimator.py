import math
import numpy as np
import rclpy
import tf2_ros
from m20_navigation_msgs.msg import TerrainState
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


class TerrainStateEstimator(Node):
    def __init__(self):
        super().__init__("terrain_state_estimator")

        self.declare_parameter("reference_path_topic", "/pct/reference_path")
        self.declare_parameter("terrain_state_topic", "/terrain/state")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("min_path_length", 0.6)
        self.declare_parameter("slope_window_max", 1.5)
        self.declare_parameter("stair_up_slope", 0.14)
        self.declare_parameter("stair_up_dz", 0.28)
        self.declare_parameter("stair_down_slope", 0.18)
        self.declare_parameter("stair_down_dz", 0.35)
        self.declare_parameter("platform_merge_distance", 3.0)
        self.declare_parameter("current_entry_margin", 0.0)
        self.declare_parameter("current_exit_margin", 0.6)
        self.declare_parameter("projection_max_distance", 2.0)
        self.declare_parameter("projection_backtrack_points", 10)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.min_path_length = max(0.05, float(self.get_parameter("min_path_length").value))
        self.slope_window_max = max(
            self.min_path_length,
            float(self.get_parameter("slope_window_max").value),
        )
        self.stair_up_slope = max(0.0, float(self.get_parameter("stair_up_slope").value))
        self.stair_up_dz = max(0.0, float(self.get_parameter("stair_up_dz").value))
        self.stair_down_slope = max(0.0, float(self.get_parameter("stair_down_slope").value))
        self.stair_down_dz = max(0.0, float(self.get_parameter("stair_down_dz").value))
        self.platform_merge_distance = max(
            0.0,
            float(self.get_parameter("platform_merge_distance").value),
        )
        self.current_entry_margin = max(
            0.0,
            float(self.get_parameter("current_entry_margin").value),
        )
        self.current_exit_margin = max(
            0.0,
            float(self.get_parameter("current_exit_margin").value),
        )
        self.projection_max_distance = max(
            0.1,
            float(self.get_parameter("projection_max_distance").value),
        )
        self.projection_backtrack_points = max(
            0,
            int(self.get_parameter("projection_backtrack_points").value),
        )

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(
            TerrainState,
            str(self.get_parameter("terrain_state_topic").value),
            latched_qos,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("reference_path_topic").value),
            self._on_reference_path,
            latched_qos,
        )

        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path = None
        self.arc = None
        self.zones = []
        self.route_id = 0
        self.progress_index = 0

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info(
            "Terrain state estimator ready: "
            f"reference={self.get_parameter('reference_path_topic').value}, "
            f"min_span={self.min_path_length:.2f}m, "
            f"max_window={self.slope_window_max:.2f}m, "
            f"platform_merge={self.platform_merge_distance:.2f}m"
        )

    def _on_reference_path(self, msg):
        points = []
        for pose in msg.poses:
            position = pose.pose.position
            point = np.array([position.x, position.y, position.z], dtype=np.float64)
            if np.all(np.isfinite(point)):
                points.append(point)
        if len(points) < 2:
            self._clear_route("reference path has fewer than two finite poses")
            return

        path = np.asarray(points, dtype=np.float64)
        keep = np.ones((len(path),), dtype=bool)
        keep[1:] = np.linalg.norm(np.diff(path, axis=0), axis=1) > 1.0e-5
        path = path[keep]
        if len(path) < 2:
            self._clear_route("reference path collapses to one unique pose")
            return

        xy_step = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
        arc = np.zeros((len(path),), dtype=np.float64)
        arc[1:] = np.cumsum(xy_step)
        if arc[-1] < self.min_path_length:
            self._clear_route("reference path is too short")
            return

        self.path = path
        self.arc = arc
        self.zones = self._extract_stair_zones(path, arc)
        self.route_id = (self.route_id + 1) & 0xFFFFFFFF
        self.progress_index = 0
        zone_text = ", ".join(
            f"{self._mode_name(zone['mode'])}@{zone['start']:.1f}-{zone['end']:.1f}m"
            for zone in self.zones
        )
        self.get_logger().info(
            f"Terrain route {self.route_id}: {len(path)} poses, "
            f"length={arc[-1]:.2f}m, zones=[{zone_text}]"
        )

    def _clear_route(self, reason):
        self.path = None
        self.arc = None
        self.zones = []
        self.progress_index = 0
        self.route_id = (self.route_id + 1) & 0xFFFFFFFF
        self.get_logger().warn(f"Terrain route cleared: {reason}")

    def _extract_stair_zones(self, path, arc):
        segment_count = len(path) - 1
        up_score = np.zeros((segment_count,), dtype=np.float64)
        down_score = np.zeros((segment_count,), dtype=np.float64)

        for start in range(len(path) - 1):
            for end in range(start + 1, len(path)):
                span = float(arc[end] - arc[start])
                if span > self.slope_window_max:
                    break
                if span < self.min_path_length:
                    continue
                dz = float(path[end, 2] - path[start, 2])
                slope = dz / span
                if slope >= self.stair_up_slope and dz >= self.stair_up_dz:
                    score = (slope / max(self.stair_up_slope, 1.0e-6)) * (
                        dz / max(self.stair_up_dz, 1.0e-6)
                    )
                    up_score[start:end] = np.maximum(up_score[start:end], score)
                if slope <= -self.stair_down_slope and dz <= -self.stair_down_dz:
                    score = (-slope / max(self.stair_down_slope, 1.0e-6)) * (
                        -dz / max(self.stair_down_dz, 1.0e-6)
                    )
                    down_score[start:end] = np.maximum(down_score[start:end], score)

        labels = np.full((segment_count,), TerrainState.FLAT, dtype=np.uint8)
        labels[up_score > down_score] = TerrainState.STAIR_UP
        labels[down_score > up_score] = TerrainState.STAIR_DOWN

        zones = []
        index = 0
        while index < segment_count:
            mode = int(labels[index])
            if mode == TerrainState.FLAT:
                index += 1
                continue
            end = index + 1
            while end < segment_count and int(labels[end]) == mode:
                end += 1
            score_values = (
                up_score[index:end]
                if mode == TerrainState.STAIR_UP
                else down_score[index:end]
            )
            direction = 1.0 if mode == TerrainState.STAIR_UP else -1.0
            signed_height_steps = direction * np.diff(path[index:end + 1, 2])
            height_epsilon = 0.05 * (
                self.stair_up_dz
                if mode == TerrainState.STAIR_UP
                else self.stair_down_dz
            )
            height_changes = np.flatnonzero(signed_height_steps > max(0.01, height_epsilon))
            if height_changes.size > 0:
                zone_start_index = index + int(height_changes[0])
                zone_end_index = index + int(height_changes[-1]) + 1
                zones.append(
                    {
                        "mode": mode,
                        "start": float(arc[zone_start_index]),
                        "end": float(arc[zone_end_index]),
                        "confidence": float(min(1.0, np.max(score_values) / 2.0)),
                    }
                )
            index = end

        merged = []
        for zone in zones:
            if (
                merged
                and merged[-1]["mode"] == zone["mode"]
                and zone["start"] - merged[-1]["end"] <= self.platform_merge_distance
            ):
                merged[-1]["end"] = zone["end"]
                merged[-1]["confidence"] = max(
                    merged[-1]["confidence"],
                    zone["confidence"],
                )
            else:
                merged.append(dict(zone))
        return merged

    def _publish_state(self):
        msg = TerrainState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.current_mode = TerrainState.UNKNOWN
        msg.upcoming_mode = TerrainState.UNKNOWN
        msg.distance_to_entry = math.inf
        msg.distance_to_exit = math.inf
        msg.confidence = 0.0
        msg.valid = False
        msg.route_id = self.route_id

        if self.path is None or self.arc is None:
            msg.reason = "no_reference_path"
            self.state_pub.publish(msg)
            return

        robot = self._robot_position()
        if robot is None:
            msg.reason = "tf_unavailable"
            self.state_pub.publish(msg)
            return

        nearest, projection_distance = self._nearest_progress_index(robot)
        if projection_distance > self.projection_max_distance:
            msg.reason = f"route_projection_too_far:{projection_distance:.2f}"
            self.state_pub.publish(msg)
            return

        progress = float(self.arc[nearest])
        current_zone = None
        for zone in self.zones:
            if (
                progress >= zone["start"] - self.current_entry_margin
                and progress <= zone["end"] + self.current_exit_margin
            ):
                current_zone = zone
                break

        upcoming_zone = current_zone
        if upcoming_zone is None:
            for zone in self.zones:
                if zone["start"] > progress:
                    upcoming_zone = zone
                    break

        msg.current_mode = (
            TerrainState.FLAT if current_zone is None else current_zone["mode"]
        )
        msg.upcoming_mode = (
            TerrainState.FLAT if upcoming_zone is None else upcoming_zone["mode"]
        )
        if upcoming_zone is not None:
            msg.distance_to_entry = max(0.0, upcoming_zone["start"] - progress)
            msg.confidence = upcoming_zone["confidence"]
        else:
            msg.confidence = 1.0
        if current_zone is not None:
            msg.distance_to_exit = max(0.0, current_zone["end"] - progress)
        msg.valid = True
        msg.reason = f"route_progress:{progress:.2f}"
        self.state_pub.publish(msg)

    def _nearest_progress_index(self, robot):
        start = max(0, self.progress_index - self.projection_backtrack_points)
        candidates = self.path[start:]
        distances = np.linalg.norm(candidates - robot.reshape((1, 3)), axis=1)
        nearest = start + int(np.argmin(distances))
        self.progress_index = max(self.progress_index, nearest)
        projection_distance = float(
            np.linalg.norm(self.path[self.progress_index] - robot)
        )
        if projection_distance <= self.projection_max_distance:
            return self.progress_index, projection_distance

        # Normal progress remains monotonic. If that projection is no longer
        # credible, reacquire anywhere on the same static route so reversing or
        # replay resets do not leave terrain state permanently UNKNOWN.
        global_distances = np.linalg.norm(
            self.path - robot.reshape((1, 3)),
            axis=1,
        )
        global_nearest = int(np.argmin(global_distances))
        global_distance = float(global_distances[global_nearest])
        if global_distance <= self.projection_max_distance:
            self.progress_index = global_nearest
            return global_nearest, global_distance
        return self.progress_index, projection_distance

    def _robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.global_frame}->{self.robot_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        translation = transform.transform.translation
        return np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )

    @staticmethod
    def _mode_name(mode):
        return {
            TerrainState.FLAT: "flat",
            TerrainState.STAIR_UP: "stair_up",
            TerrainState.STAIR_DOWN: "stair_down",
        }.get(mode, "unknown")


def main(args=None):
    rclpy.init(args=args)
    node = TerrainStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
