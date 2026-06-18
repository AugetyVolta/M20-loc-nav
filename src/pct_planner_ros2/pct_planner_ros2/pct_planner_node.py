import importlib
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from .pct_paths import (
    configure_planner_imports,
    default_pct_root,
    missing_ld_library_dirs,
    tomogram_stem,
    vector3,
)


class PctPlannerNode(Node):
    def __init__(self):
        super().__init__("pct_planner_node")

        self.declare_parameter("pct_root", default_pct_root())
        self.declare_parameter("tomogram_file", "building2_9")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("path_topic", "/pct_path")
        self.declare_parameter("start_source", "fixed")
        self.declare_parameter("odom_topic", "/odom_body")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("start_point_topic", "/pct_start_point")
        self.declare_parameter("goal_point_topic", "/pct_goal_point")
        self.declare_parameter("initial_start", [-5.5, 6.0, 0.5])
        self.declare_parameter("initial_goal", [2.0, -3.0, 4.5])
        self.declare_parameter("use_interactive_markers", True)
        self.declare_parameter("marker_namespace", "basic_controls")
        self.declare_parameter("marker_z_offset", 0.5)
        self.declare_parameter("replan_interval", 0.5)
        self.declare_parameter("position_epsilon", 0.01)
        self.declare_parameter("auto_plan", True)
        self.declare_parameter("always_replan", False)
        self.declare_parameter("a_star_cost_threshold", 20.0)
        self.declare_parameter("safe_cost_margin", 15.0)
        self.declare_parameter("step_cost_weight", 1.0)
        self.declare_parameter("max_heading_rate", 10.0)
        self.declare_parameter("layer_match_height_tolerance", 1.0)
        self.declare_parameter("global_path_perception_enabled", False)
        self.declare_parameter("global_path_perception_scan_topic", "/scan")
        self.declare_parameter("global_path_perception_cloud_topic", "")
        self.declare_parameter("global_path_perception_min_range", 0.45)
        self.declare_parameter("global_path_perception_width", 4.0)
        self.declare_parameter("global_path_perception_height", 4.0)
        self.declare_parameter("global_path_perception_inflation_radius", 0.60)
        self.declare_parameter("global_path_perception_inscribed_radius", 0.35)
        self.declare_parameter("global_path_perception_cost", -1.0)
        self.declare_parameter("global_path_perception_cost_scaling_factor", 5.0)
        self.declare_parameter("global_path_perception_height_tolerance", 0.75)
        self.declare_parameter("global_path_perception_clear_robot_radius", -1.0)
        self.declare_parameter("global_path_perception_update_interval", 0.2)
        self.declare_parameter("global_path_perception_persistence", 1.0)
        self.declare_parameter("global_path_perception_min_changed_cells", 3)
        self.declare_parameter("global_path_perception_max_points", 720)
        self.declare_parameter("global_path_perception_mark_all_layers", False)

        self.pct_root = self.get_parameter("pct_root").value
        self.tomogram_file = tomogram_stem(self.get_parameter("tomogram_file").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.start_source = self.get_parameter("start_source").value
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.marker_z_offset = float(self.get_parameter("marker_z_offset").value)
        self.position_epsilon = float(self.get_parameter("position_epsilon").value)
        self.auto_plan = bool(self.get_parameter("auto_plan").value)
        self.always_replan = bool(self.get_parameter("always_replan").value)
        self.global_path_perception_enabled = bool(self.get_parameter("global_path_perception_enabled").value)
        self.global_path_perception_min_range = float(self.get_parameter("global_path_perception_min_range").value)
        self.global_path_perception_width = max(0.0, float(self.get_parameter("global_path_perception_width").value))
        self.global_path_perception_height = max(0.0, float(self.get_parameter("global_path_perception_height").value))
        self.global_path_perception_half_width = 0.5 * self.global_path_perception_width
        self.global_path_perception_half_height = 0.5 * self.global_path_perception_height
        self.global_path_perception_window_range = math.hypot(
            self.global_path_perception_half_width,
            self.global_path_perception_half_height,
        )
        self.global_path_perception_inflation_radius = float(
            self.get_parameter("global_path_perception_inflation_radius").value
        )
        self.global_path_perception_inscribed_radius = float(
            self.get_parameter("global_path_perception_inscribed_radius").value
        )
        self.global_path_perception_cost = float(self.get_parameter("global_path_perception_cost").value)
        if self.global_path_perception_cost <= 0.0:
            self.global_path_perception_cost = float(self.get_parameter("a_star_cost_threshold").value) + 5.0
        self.global_path_perception_cost_scaling_factor = float(
            self.get_parameter("global_path_perception_cost_scaling_factor").value
        )
        self.global_path_perception_height_tolerance = float(
            self.get_parameter("global_path_perception_height_tolerance").value
        )
        self.global_path_perception_clear_robot_radius = float(
            self.get_parameter("global_path_perception_clear_robot_radius").value
        )
        if self.global_path_perception_clear_robot_radius <= 0.0:
            self.global_path_perception_clear_robot_radius = self.global_path_perception_inscribed_radius + 0.03
        self.global_path_perception_update_interval = float(
            self.get_parameter("global_path_perception_update_interval").value
        )
        self.global_path_perception_persistence = float(
            self.get_parameter("global_path_perception_persistence").value
        )
        self.global_path_perception_min_changed_cells = int(
            self.get_parameter("global_path_perception_min_changed_cells").value
        )
        self.global_path_perception_max_points = int(self.get_parameter("global_path_perception_max_points").value)
        self.global_path_perception_mark_all_layers = bool(
            self.get_parameter("global_path_perception_mark_all_layers").value
        )

        self.start_pos = np.array(
            vector3(self.get_parameter("initial_start").value, [-5.5, 6.0, 0.5]),
            dtype=np.float32,
        )
        self.goal_pos = np.array(
            vector3(self.get_parameter("initial_goal").value, [2.0, -3.0, 4.5]),
            dtype=np.float32,
        )
        self.last_planned_start = None
        self.last_planned_goal = None
        self.last_planned_perception_seq = -1
        self.planning = False
        self.perception_layer_seq = 0
        self.last_perception_update_time = None
        self.last_perception_update_wall_time = 0.0

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, path_qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._load_pct_core()
        self.planner.loadTomogram(self.tomogram_file)
        self.get_logger().info(f"Loaded tomogram '{self.tomogram_file}' from {self.pct_root}")
        self._init_global_path_perception_inputs()

        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            self._on_goal_pose,
            10,
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("clicked_point_topic").value,
            self._on_clicked_point,
            10,
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("start_point_topic").value,
            self._on_start_point,
            10,
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("goal_point_topic").value,
            self._on_goal_point,
            10,
        )
        if self.start_source == "odom":
            self.create_subscription(
                Odometry,
                self.get_parameter("odom_topic").value,
                self._on_odom,
                20,
            )
            self.get_logger().info(f"Using odometry start source: {self.get_parameter('odom_topic').value}")
        elif self.start_source == "tf":
            self.get_logger().info(f"Using TF start source: {self.global_frame}->{self.robot_frame}")

        self.marker_server = None
        if bool(self.get_parameter("use_interactive_markers").value):
            self._init_interactive_markers()

        self.timer = self.create_timer(float(self.get_parameter("replan_interval").value), self._timer_cb)

    def _init_global_path_perception_inputs(self):
        if not self.global_path_perception_enabled:
            return

        scan_topic = str(self.get_parameter("global_path_perception_scan_topic").value)
        cloud_topic = str(self.get_parameter("global_path_perception_cloud_topic").value)
        self.perception_scan_sub = None
        self.perception_cloud_sub = None

        if scan_topic:
            self.perception_scan_sub = self.create_subscription(
                LaserScan,
                scan_topic,
                self._on_global_path_perception_scan,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"C++ PCT global path perception enabled from LaserScan '{scan_topic}' "
                f"window={self.global_path_perception_width:.2f}x{self.global_path_perception_height:.2f}m, "
                f"inflation={self.global_path_perception_inflation_radius:.2f}m, "
                f"scale={self.global_path_perception_cost_scaling_factor:.2f}, "
                f"cost={self.global_path_perception_cost:.1f}"
            )

        if cloud_topic:
            self.perception_cloud_sub = self.create_subscription(
                PointCloud2,
                cloud_topic,
                self._on_global_path_perception_cloud,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"C++ PCT global path perception also listening to PointCloud2 '{cloud_topic}'"
            )

        if self.perception_scan_sub is None and self.perception_cloud_sub is None:
            self.get_logger().warn("global_path_perception_enabled=true but no scan/cloud topic is configured")

    def _load_pct_core(self):
        missing = missing_ld_library_dirs(self.pct_root)
        if missing:
            self.get_logger().warn(
                "PCT library paths are not in LD_LIBRARY_PATH. Source scripts/pct_env.sh before running. "
                + "Missing: "
                + ", ".join(missing)
            )

        configure_planner_imports(self.pct_root)
        for module_name in (
            "config",
            "utils",
            "planner_wrapper",
            "lib.a_star",
            "lib.ele_planner",
            "lib.traj_opt",
            "a_star",
            "ele_planner",
            "traj_opt",
        ):
            sys.modules.pop(module_name, None)

        try:
            planner_config = importlib.import_module("config").Config
            tomogram_planner = importlib.import_module("planner_wrapper").TomogramPlanner
        except Exception as exc:
            raise RuntimeError(
                "Failed to import PCT planner modules. Check PCT_PLANNER_ROOT, PYTHONPATH, "
                "LD_LIBRARY_PATH, and that PCT_planner/planner/build.sh has completed."
            ) from exc

        cfg = planner_config()
        cfg.planner.a_star_cost_threshold = float(self.get_parameter("a_star_cost_threshold").value)
        cfg.planner.safe_cost_margin = float(self.get_parameter("safe_cost_margin").value)
        cfg.planner.step_cost_weight = float(self.get_parameter("step_cost_weight").value)
        cfg.planner.max_heading_rate = float(self.get_parameter("max_heading_rate").value)
        cfg.planner.layer_match_height_tolerance = float(
            self.get_parameter("layer_match_height_tolerance").value
        )
        self.planner = tomogram_planner(cfg)

    def _timer_cb(self):
        if not self.auto_plan or self.planning:
            return
        if not self._update_start_from_tf():
            return
        self._expire_global_path_perception_if_stale()
        if not self.always_replan and not self._position_changed():
            return
        self._plan_and_publish()

    def _update_start_from_tf(self):
        if self.start_source != "tf":
            return True
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
            return False
        translation = transform.transform.translation
        self.start_pos = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )
        return True

    def _position_changed(self):
        if self.last_planned_start is None or self.last_planned_goal is None:
            return True
        return (
            np.linalg.norm(self.start_pos - self.last_planned_start) >= self.position_epsilon
            or np.linalg.norm(self.goal_pos - self.last_planned_goal) >= self.position_epsilon
            or self.perception_layer_seq != self.last_planned_perception_seq
        )

    def _plan_and_publish(self):
        self.planning = True
        start_time = time.perf_counter()
        try:
            traj_3d = self.planner.plan(self.start_pos, self.goal_pos)
            if traj_3d is None:
                plan_info = getattr(self.planner, "last_plan_info", {})
                self.get_logger().warn(
                    f"No PCT path found: start={self.start_pos.tolist()}, goal={self.goal_pos.tolist()}, "
                    f"plan_info={plan_info}"
                )
                return
            self.path_pub.publish(self._traj_to_path(traj_3d))
            self.last_planned_start = self.start_pos.copy()
            self.last_planned_goal = self.goal_pos.copy()
            self.last_planned_perception_seq = self.perception_layer_seq
            dt_ms = (time.perf_counter() - start_time) * 1000.0
            self.get_logger().info(
                f"Published /pct_path with {len(traj_3d)} poses in {dt_ms:.1f} ms"
            )
        except Exception as exc:
            self.get_logger().error(f"PCT planning failed: {exc}")
        finally:
            self.planning = False

    def _traj_to_path(self, traj):
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = self.frame_id

        for waypoint in traj:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        self.start_pos = np.array([position.x, position.y, position.z], dtype=np.float32)

    def _on_global_path_perception_scan(self, msg):
        if not self._global_path_perception_update_due():
            return

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges >= max(float(msg.range_min), self.global_path_perception_min_range)
        range_max = self.global_path_perception_window_range
        if math.isfinite(float(msg.range_max)) and msg.range_max > 0.0:
            range_max = min(range_max, float(msg.range_max))
        valid &= ranges <= range_max
        if not np.any(valid):
            self._apply_global_path_perception_points(np.empty((0, 3), dtype=np.float32), msg.header.frame_id)
            return

        ranges = ranges[valid]
        angles = angles[valid]
        if self.global_path_perception_max_points > 0 and ranges.size > self.global_path_perception_max_points:
            step = max(1, int(math.ceil(ranges.size / self.global_path_perception_max_points)))
            ranges = ranges[::step][: self.global_path_perception_max_points]
            angles = angles[::step][: self.global_path_perception_max_points]

        points = np.zeros((ranges.size, 3), dtype=np.float32)
        points[:, 0] = ranges * np.cos(angles)
        points[:, 1] = ranges * np.sin(angles)
        self._apply_global_path_perception_points(points, msg.header.frame_id)

    def _on_global_path_perception_cloud(self, msg):
        if not self._global_path_perception_update_due():
            return

        points = []
        max_points = self.global_path_perception_max_points
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            distance_xy = math.hypot(x, y)
            if distance_xy < self.global_path_perception_min_range or distance_xy > self.global_path_perception_window_range:
                continue
            points.append((x, y, z))
            if max_points > 0 and len(points) >= max_points:
                break
        if points:
            points_np = np.asarray(points, dtype=np.float32)
        else:
            points_np = np.empty((0, 3), dtype=np.float32)
        self._apply_global_path_perception_points(points_np, msg.header.frame_id)

    def _global_path_perception_update_due(self):
        now = time.monotonic()
        if now - self.last_perception_update_wall_time < self.global_path_perception_update_interval:
            return False
        self.last_perception_update_wall_time = now
        return True

    def _apply_global_path_perception_points(self, points_sensor, source_frame):
        source_frame = (source_frame or self.robot_frame).lstrip("/")
        points_map = self._transform_points_to_map(points_sensor, source_frame)
        if points_map is None:
            return

        perception_indices = self._build_global_path_perception_indices(points_map)
        clear_center = self._current_robot_grid_index()
        changed_cells = self.planner.update_global_path_perception(
            perception_indices,
            self.global_path_perception_inflation_radius,
            self.global_path_perception_inscribed_radius,
            self.global_path_perception_cost,
            self.global_path_perception_cost_scaling_factor,
            self._now_seconds(),
            self.global_path_perception_persistence,
            clear_center,
            self.global_path_perception_clear_robot_radius,
        )
        if changed_cells < self.global_path_perception_min_changed_cells:
            self.last_perception_update_time = self.get_clock().now()
            return

        self.perception_layer_seq += 1
        active_cells = self.planner.global_path_perception_cell_count()
        self.get_logger().info(
            f"Updated C++ PCT global path perception: active_cells={active_cells}, "
            f"changed_cells={changed_cells}, seq={self.perception_layer_seq}",
            throttle_duration_sec=1.0,
        )
        self.last_perception_update_time = self.get_clock().now()

    def _now_seconds(self):
        now = self.get_clock().now().nanoseconds
        return float(now) * 1.0e-9

    def _current_robot_grid_index(self):
        layer = int(self.planner.match_best_layer(self.start_pos[0], self.start_pos[1], self.start_pos[2]))
        grid_idx = self.planner.pos2idx(self.start_pos[:2]).astype(int)
        return np.array([layer, int(grid_idx[1]), int(grid_idx[0])], dtype=np.int32)

    def _transform_points_to_map(self, points, source_frame):
        if points.size == 0:
            return points.reshape(0, 3).astype(np.float32, copy=False)
        target_frame = self.global_frame or self.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except Exception as exc:
            self.get_logger().warn(
                f"Dynamic layer waiting for TF {target_frame}<-{source_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        q = transform.transform.rotation
        rot = self._quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        return (points.astype(np.float32, copy=False) @ rot.T) + translation

    @staticmethod
    def _quat_to_rot_matrix(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-9 or not math.isfinite(norm):
            return np.eye(3, dtype=np.float32)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

    def _build_global_path_perception_indices(self, points_map):
        if points_map.size == 0:
            return np.zeros((0, 3), dtype=np.int32)

        resolution = float(self.planner.resolution)
        if resolution <= 0.0:
            return np.zeros((0, 3), dtype=np.int32)
        layer_heights = self.planner.layer_elev_grids
        n_layers, size_x, size_y = layer_heights.shape

        robot_xy = self.start_pos[:2].astype(np.float32, copy=False)
        indices = []
        for point in points_map:
            delta_xy = point[:2] - robot_xy
            if (
                abs(float(delta_xy[0])) > self.global_path_perception_half_width
                or abs(float(delta_xy[1])) > self.global_path_perception_half_height
            ):
                continue
            if np.linalg.norm(delta_xy) < self.global_path_perception_clear_robot_radius:
                continue

            grid_idx = self.planner.pos2idx(point[:2]).astype(int)
            center_row = int(grid_idx[1])
            center_col = int(grid_idx[0])
            if center_row < 0 or center_row >= size_x or center_col < 0 or center_col >= size_y:
                continue

            point_height = float(point[2])
            candidate_layers = self._global_path_perception_candidate_layers(
                layer_heights, center_row, center_col, point_height
            )
            if not candidate_layers:
                continue

            for layer in candidate_layers:
                indices.append((int(layer), center_row, center_col))

        if not indices:
            return np.zeros((0, 3), dtype=np.int32)
        return np.unique(np.asarray(indices, dtype=np.int32), axis=0)

    def _global_path_perception_candidate_layers(self, layer_heights, row, col, point_height):
        heights = layer_heights[:, row, col]
        valid = np.isfinite(heights) & (heights > -99.0)
        if not np.any(valid):
            return []
        if self.global_path_perception_mark_all_layers:
            return np.flatnonzero(valid).astype(int).tolist()

        diff = np.abs(heights - point_height)
        mask = valid & (diff <= self.global_path_perception_height_tolerance)
        if np.any(mask):
            return np.flatnonzero(mask).astype(int).tolist()

        valid_indices = np.flatnonzero(valid)
        best = int(valid_indices[np.argmin(diff[valid_indices])])
        return [best]

    def _expire_global_path_perception_if_stale(self):
        if not self.global_path_perception_enabled:
            return
        if self.last_perception_update_time is None:
            return
        if self.global_path_perception_persistence <= 0.0:
            return
        age = (self.get_clock().now() - self.last_perception_update_time).nanoseconds * 1.0e-9
        if age <= self.global_path_perception_persistence:
            return
        changed_cells = self.planner.decay_global_path_perception(
            self._now_seconds(),
            self.global_path_perception_persistence,
        )
        if changed_cells > 0:
            self.perception_layer_seq += 1
            if self.planner.global_path_perception_cell_count() == 0:
                self.last_perception_update_time = None
            self.get_logger().info(
                f"Decayed stale C++ PCT global path perception after {age:.2f}s, "
                f"changed_cells={changed_cells}, seq={self.perception_layer_seq}",
                throttle_duration_sec=1.0,
            )

    def _on_goal_pose(self, msg):
        position = msg.pose.position
        self.goal_pos = np.array([position.x, position.y, position.z], dtype=np.float32)
        self._sync_marker_pose("end_pos", self.goal_pos)

    def _on_clicked_point(self, msg):
        point = msg.point
        self._set_goal_point(point)

    def _on_start_point(self, msg):
        if self.start_source in ("odom", "tf"):
            self.get_logger().warn(f"Ignoring clicked start point because start_source is '{self.start_source}'")
            return
        point = msg.point
        self.start_pos = np.array([point.x, point.y, point.z], dtype=np.float32)
        self._sync_marker_pose("start_pos", self.start_pos)

    def _on_goal_point(self, msg):
        point = msg.point
        self._set_goal_point(point)

    def _set_goal_point(self, point):
        self.goal_pos = np.array([point.x, point.y, point.z], dtype=np.float32)
        self._sync_marker_pose("end_pos", self.goal_pos)

    def _init_interactive_markers(self):
        namespace = self.get_parameter("marker_namespace").value
        self.marker_server = InteractiveMarkerServer(self, namespace)
        if self.start_source not in ("odom", "tf"):
            self._make_6dof_marker(self.start_pos, "start_pos")
        self._make_6dof_marker(self.goal_pos, "end_pos")
        self.marker_server.applyChanges()
        self.get_logger().info(f"Interactive markers ready in namespace '{namespace}'")

    def _sync_marker_pose(self, name, actual_pos):
        if self.marker_server is None:
            return
        pose = PoseStamped().pose
        pose.position.x = float(actual_pos[0])
        pose.position.y = float(actual_pos[1])
        pose.position.z = float(actual_pos[2] + self.marker_z_offset)
        pose.orientation.w = 1.0
        self.marker_server.setPose(name, pose)
        self.marker_server.applyChanges()

    def _process_marker_feedback(self, feedback):
        p = feedback.pose.position
        actual = np.array([p.x, p.y, p.z - self.marker_z_offset], dtype=np.float32)
        if feedback.marker_name == "start_pos" and self.start_source != "odom":
            self.start_pos = actual
        elif feedback.marker_name == "end_pos":
            self.goal_pos = actual

    def _make_box(self):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.r = 0.0
        marker.color.g = 0.25
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _make_box_control(self, int_marker):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self._make_box())
        int_marker.controls.append(control)

    def _make_6dof_marker(self, actual_pos, name):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = name
        int_marker.description = name
        int_marker.scale = 1.0
        int_marker.pose.position = Point(
            x=float(actual_pos[0]),
            y=float(actual_pos[1]),
            z=float(actual_pos[2] + self.marker_z_offset),
        )
        int_marker.pose.orientation.w = 1.0
        self._make_box_control(int_marker)

        axes = [
            ("rotate_x", InteractiveMarkerControl.ROTATE_AXIS, (1.0, 1.0, 0.0, 0.0)),
            ("move_x", InteractiveMarkerControl.MOVE_AXIS, (1.0, 1.0, 0.0, 0.0)),
            ("rotate_z", InteractiveMarkerControl.ROTATE_AXIS, (1.0, 0.0, 1.0, 0.0)),
            ("move_z", InteractiveMarkerControl.MOVE_AXIS, (1.0, 0.0, 1.0, 0.0)),
            ("rotate_y", InteractiveMarkerControl.ROTATE_AXIS, (1.0, 0.0, 0.0, 1.0)),
            ("move_y", InteractiveMarkerControl.MOVE_AXIS, (1.0, 0.0, 0.0, 1.0)),
        ]
        for control_name, mode, orientation in axes:
            control = InteractiveMarkerControl()
            control.name = control_name
            control.interaction_mode = mode
            control.orientation_mode = InteractiveMarkerControl.FIXED
            self._set_normalized_orientation(control, orientation)
            int_marker.controls.append(control)

        self.marker_server.insert(int_marker, feedback_callback=self._process_marker_feedback)

    @staticmethod
    def _set_normalized_orientation(control, values):
        w, x, y, z = values
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        control.orientation.w = w / norm
        control.orientation.x = x / norm
        control.orientation.y = y / norm
        control.orientation.z = z / norm


def main(args=None):
    rclpy.init(args=args)
    node = PctPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
