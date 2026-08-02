import importlib
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from m20_navigation_msgs.msg import NavigationMode, TerrainState
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty
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
        self.declare_parameter("reference_path_topic", "/pct/reference_path")
        self.declare_parameter("navigation_mode_topic", "/navigation_mode")
        self.declare_parameter("start_source", "fixed")
        self.declare_parameter("odom_topic", "/odom_body")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("cancel_goal_topic", "/pct/cancel_goal")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("start_point_topic", "/pct_start_point")
        self.declare_parameter("goal_point_topic", "/pct_goal_point")
        self.declare_parameter("initial_start", [-5.5, 6.0, 0.5])
        self.declare_parameter("initial_goal", [2.0, -3.0, 4.5])
        self.declare_parameter("use_interactive_markers", True)
        self.declare_parameter("marker_namespace", "basic_controls")
        self.declare_parameter("marker_z_offset", 0.5)
        self.declare_parameter("replan_interval", 1.0)
        self.declare_parameter("auto_plan", True)
        self.declare_parameter("always_replan", True)
        self.declare_parameter("wait_for_goal", False)
        self.declare_parameter("a_star_cost_threshold", 20.0)
        self.declare_parameter("safe_cost_margin", 15.0)
        self.declare_parameter("step_cost_weight", 1.0)
        self.declare_parameter("max_heading_rate", 10.0)
        self.declare_parameter("layer_match_height_tolerance", 1.0)
        self.declare_parameter("path_ground_offset", 0.10)
        self.declare_parameter("robot_ground_offset", 0.45)
        self.declare_parameter("global_path_perception_enabled", False)
        self.declare_parameter("global_path_perception_scan_topic", "/scan")
        self.declare_parameter("global_path_perception_cloud_topic", "")
        self.declare_parameter("global_path_perception_min_range", 0.15)
        self.declare_parameter("global_path_perception_width", 8.0)
        self.declare_parameter("global_path_perception_height", 8.0)
        self.declare_parameter("global_path_perception_inflation_radius", 1.0)
        self.declare_parameter("global_path_perception_inscribed_radius", 0.45)
        self.declare_parameter("global_path_perception_cost", -1.0)
        self.declare_parameter("global_path_perception_cost_scaling_factor", 5.0)
        self.declare_parameter("global_path_perception_height_tolerance", 0.75)
        self.declare_parameter("global_path_perception_update_interval", 0.2)
        self.declare_parameter("global_path_perception_persistence", 1.0)
        self.declare_parameter("global_path_perception_min_changed_cells", 3)
        self.declare_parameter("global_path_perception_max_points", 720)
        self.declare_parameter("global_path_perception_mark_all_layers", False)
        self.declare_parameter("global_path_perception_raytrace_enabled", True)
        self.declare_parameter("global_path_perception_raytrace_max_range", 0.0)
        self.declare_parameter("global_path_perception_raytrace_max_rays", 360)
        self.declare_parameter("local_replan_enabled", True)
        self.declare_parameter("local_replan_forward_distance", 6.0)
        self.declare_parameter("local_replan_join_extension", 2.0)
        self.declare_parameter("reference_rebuild_distance", 2.0)
        self.declare_parameter("reference_rebuild_backtrack_distance", 0.6)

        self.pct_root = self.get_parameter("pct_root").value
        self.tomogram_file = tomogram_stem(self.get_parameter("tomogram_file").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.start_source = self.get_parameter("start_source").value
        self.robot_ground_offset = max(
            0.0,
            float(self.get_parameter("robot_ground_offset").value),
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.marker_z_offset = float(self.get_parameter("marker_z_offset").value)
        self.auto_plan = bool(self.get_parameter("auto_plan").value)
        self.always_replan = bool(self.get_parameter("always_replan").value)
        self.goal_received = not bool(self.get_parameter("wait_for_goal").value)
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
        self.global_path_perception_raytrace_enabled = bool(
            self.get_parameter("global_path_perception_raytrace_enabled").value
        )
        self.global_path_perception_raytrace_max_range = float(
            self.get_parameter("global_path_perception_raytrace_max_range").value
        )
        if self.global_path_perception_raytrace_max_range <= 0.0:
            self.global_path_perception_raytrace_max_range = self.global_path_perception_window_range
        self.global_path_perception_raytrace_max_rays = int(
            self.get_parameter("global_path_perception_raytrace_max_rays").value
        )
        self.local_replan_enabled = bool(self.get_parameter("local_replan_enabled").value)
        self.local_replan_forward_distance = max(
            0.5,
            float(self.get_parameter("local_replan_forward_distance").value),
        )
        self.local_replan_join_extension = max(
            0.0,
            float(self.get_parameter("local_replan_join_extension").value),
        )
        self.reference_rebuild_distance = max(
            0.1,
            float(self.get_parameter("reference_rebuild_distance").value),
        )
        self.reference_rebuild_backtrack_distance = max(
            0.0,
            float(self.get_parameter("reference_rebuild_backtrack_distance").value),
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
        self.planning = False
        self.last_perception_update_time = None
        self.last_perception_update_wall_time = 0.0
        self.reference_path = None
        self.reference_layers = None
        self.reference_arc = None
        self.reference_goal = None
        self.reference_progress_index = 0
        self.pct_dynamic_enabled = True
        self.terrain_current = TerrainState.UNKNOWN

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, path_qos)
        self.reference_path_pub = self.create_publisher(
            Path,
            str(self.get_parameter("reference_path_topic").value),
            path_qos,
        )
        self.create_subscription(
            NavigationMode,
            str(self.get_parameter("navigation_mode_topic").value),
            self._on_navigation_mode,
            path_qos,
        )
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener_node = rclpy.create_node(
            f"{self.get_name()}_tf_listener",
            use_global_arguments=False,
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.tf_listener_node,
            spin_thread=True,
        )

        self._load_pct_core()
        self.planner.loadTomogram(self.tomogram_file)
        self.get_logger().info(f"Loaded tomogram '{self.tomogram_file}' from {self.pct_root}")
        self.get_logger().info(
            f"Path ground offset: {float(self.get_parameter('path_ground_offset').value):.3f} m"
        )
        self.get_logger().info(
            f"Robot ground offset used for layer matching: {self.robot_ground_offset:.3f} m"
        )
        self._init_global_path_perception_inputs()

        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            self._on_goal_pose,
            10,
        )
        self.create_subscription(
            Empty,
            self.get_parameter("cancel_goal_topic").value,
            self._on_cancel_goal,
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

        replan_interval = float(self.get_parameter("replan_interval").value)
        self.get_logger().info(
            "PCT planning loop: "
            f"auto_plan={int(self.auto_plan)}, always_replan={int(self.always_replan)}, "
            f"replan_interval={replan_interval:.3f}s, start_source={self.start_source}, "
            f"global_frame={self.global_frame}, robot_frame={self.robot_frame}, "
            f"odom_topic={self.get_parameter('odom_topic').value}"
        )
        self.get_logger().info(
            "PCT terrain policy is external: "
            f"reference_path={self.get_parameter('reference_path_topic').value}, "
            f"navigation_mode={self.get_parameter('navigation_mode_topic').value}"
        )
        self.timer = self.create_timer(replan_interval, self._timer_cb)

    def destroy_node(self):
        listener = getattr(self, "tf_listener", None)
        listener_executor = getattr(listener, "executor", None)
        if listener_executor is not None:
            listener_executor.shutdown()
        listener_thread = getattr(listener, "dedicated_listener_thread", None)
        if listener_thread is not None:
            listener_thread.join(timeout=1.0)
        tf_listener_node = getattr(self, "tf_listener_node", None)
        if tf_listener_node is not None:
            tf_listener_node.destroy_node()
            self.tf_listener_node = None
        return super().destroy_node()

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
                f"cost={self.global_path_perception_cost:.1f}, "
                f"min_range={self.global_path_perception_min_range:.2f}m, "
                f"raytrace={int(self.global_path_perception_raytrace_enabled)}"
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
        cfg.planner.path_ground_offset = float(self.get_parameter("path_ground_offset").value)
        self.planner = tomogram_planner(cfg)

    def _timer_cb(self):
        if not self.auto_plan or self.planning:
            return
        if not self.goal_received:
            return
        if not self._update_start_from_tf():
            return
        # The static route is maintained independently of dynamic local repairs.
        reference_nearest = self._prepare_static_reference_context()
        self._expire_global_path_perception_if_stale()
        if not self.always_replan and not self._position_changed():
            return
        self._plan_and_publish(reference_nearest)

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
            self._position_differs(self.start_pos, self.last_planned_start)
            or self._position_differs(self.goal_pos, self.last_planned_goal)
        )

    @staticmethod
    def _position_differs(first, second):
        return float(np.linalg.norm(np.asarray(first) - np.asarray(second))) > 1.0e-4

    def _start_ground_height_hint(self):
        if self.start_source in ("tf", "odom"):
            return float(self.start_pos[2]) - self.robot_ground_offset
        return float(self.start_pos[2])

    def _nearest_reference_index(self, reference_path):
        reference_path = np.asarray(reference_path, dtype=np.float32)
        if reference_path.ndim != 2 or reference_path.shape[0] == 0:
            return 0
        search_start = max(0, min(self.reference_progress_index, len(reference_path) - 1) - 5)
        distances = np.linalg.norm(
            reference_path[search_start:, :3] - self.start_pos[:3],
            axis=1,
        )
        nearest = search_start + int(np.argmin(distances))
        self.reference_progress_index = max(self.reference_progress_index, nearest)
        return nearest

    @staticmethod
    def _reference_arc_lengths(reference_path):
        reference_path = np.asarray(reference_path, dtype=np.float32)
        arc = np.zeros((len(reference_path),), dtype=np.float32)
        if len(reference_path) > 1:
            arc[1:] = np.cumsum(
                np.linalg.norm(np.diff(reference_path[:, :3], axis=0), axis=1)
            )
        return arc

    def _global_nearest_reference_index(self):
        reference_path = np.asarray(self.reference_path, dtype=np.float32)
        distances = np.linalg.norm(
            reference_path[:, :3] - self.start_pos[:3],
            axis=1,
        )
        nearest = int(np.argmin(distances))
        return nearest, float(distances[nearest])

    def _reference_anchor_index(self, reference_path, nearest_index, forward_distance=None):
        reference_path = np.asarray(reference_path, dtype=np.float32)
        if nearest_index >= len(reference_path) - 1:
            return len(reference_path) - 1
        if forward_distance is None:
            forward_distance = self.local_replan_forward_distance
        distance = 0.0
        for index in range(nearest_index + 1, len(reference_path)):
            distance += float(
                np.linalg.norm(reference_path[index, :3] - reference_path[index - 1, :3])
            )
            if distance >= forward_distance:
                return index
        return len(reference_path) - 1

    @staticmethod
    def _reference_join_heading(reference_path, anchor_index):
        reference_path = np.asarray(reference_path, dtype=np.float32)
        anchor_xy = reference_path[anchor_index, :2]
        for index in range(anchor_index + 1, len(reference_path)):
            delta = reference_path[index, :2] - anchor_xy
            if np.linalg.norm(delta) >= 0.3:
                return math.atan2(float(delta[1]), float(delta[0]))
        for index in range(anchor_index - 1, -1, -1):
            delta = anchor_xy - reference_path[index, :2]
            if np.linalg.norm(delta) >= 0.3:
                return math.atan2(float(delta[1]), float(delta[0]))
        return None

    def _reference_path_rebuild_reason(self):
        if (
            self.reference_path is None
            or self.reference_layers is None
            or self.reference_arc is None
            or self.reference_goal is None
        ):
            return "reference_missing"
        if self._position_differs(self.goal_pos, self.reference_goal):
            return "goal_changed"

        nearest, nearest_distance = self._global_nearest_reference_index()
        if nearest_distance > self.reference_rebuild_distance:
            return f"route_departure:{nearest_distance:.2f}m"

        progress = min(
            max(0, self.reference_progress_index),
            len(self.reference_path) - 1,
        )
        if nearest >= progress or self.reference_rebuild_backtrack_distance <= 0.0:
            return None

        backtrack_distance = float(
            self.reference_arc[progress] - self.reference_arc[nearest]
        )
        progress_distance = float(
            np.linalg.norm(
                self.reference_path[progress, :3] - self.start_pos[:3]
            )
        )
        if (
            backtrack_distance > self.reference_rebuild_backtrack_distance
            and progress_distance > self.reference_rebuild_backtrack_distance
        ):
            return f"route_backtrack:{backtrack_distance:.2f}m"
        return None

    def _ensure_reference_path(self):
        rebuild_reason = self._reference_path_rebuild_reason()
        if rebuild_reason is None:
            return True
        reference = self.planner.plan(
            self.start_pos,
            self.goal_pos,
            use_dynamic=False,
            search_bounds=None,
            start_height_hint=self._start_ground_height_hint(),
        )
        if reference is None:
            if rebuild_reason.startswith("route_backtrack:"):
                nearest, nearest_distance = self._global_nearest_reference_index()
                if nearest_distance <= self.reference_rebuild_distance:
                    previous_progress = self.reference_progress_index
                    self.reference_progress_index = nearest
                    self.get_logger().warn(
                        "Static PCT rebuild failed while moving backwards; "
                        f"reacquired the existing route at index {previous_progress}->{nearest}, "
                        f"distance={nearest_distance:.2f}m"
                    )
                    return True
            self._clear_reference_path(
                f"static PCT planning failed ({rebuild_reason})"
            )
            return False
        reference_path = np.asarray(reference, dtype=np.float32)
        reference_layers = getattr(self.planner, "last_traj_layers", None)
        if reference_layers is None:
            self.get_logger().error("PCT reference path did not provide layer metadata")
            self._clear_reference_path("reference layer metadata is missing")
            return False
        reference_layers = np.asarray(reference_layers, dtype=np.int32).reshape((-1,))
        if len(reference_layers) != len(reference_path):
            self.get_logger().error(
                f"PCT reference layer count mismatch: path={len(reference_path)}, "
                f"layers={len(reference_layers)}"
            )
            self._clear_reference_path("reference layer metadata length mismatch")
            return False
        self.reference_path = reference_path
        self.reference_layers = reference_layers
        self.reference_arc = self._reference_arc_lengths(reference_path)
        self.reference_goal = self.goal_pos.copy()
        self.reference_progress_index = 0
        self.reference_path_pub.publish(self._traj_to_path(reference_path))
        self.get_logger().info(
            f"Published static PCT reference path with {len(reference_path)} poses "
            f"after {rebuild_reason}"
        )
        return True

    def _clear_reference_path(self, reason):
        self.reference_path = None
        self.reference_layers = None
        self.reference_arc = None
        self.reference_goal = None
        self.reference_progress_index = 0
        self.reference_path_pub.publish(self._traj_to_path([]))
        self.get_logger().warn(f"Cleared static PCT reference path: {reason}")

    def _prepare_static_reference_context(self):
        if not self._ensure_reference_path():
            return None
        nearest = self._nearest_reference_index(self.reference_path)
        return nearest

    def _static_reference_fallback(self, nearest):
        fallback = np.asarray(self.reference_path[nearest:], dtype=np.float32).copy()
        if fallback.ndim != 2 or fallback.shape[0] == 0:
            return None
        start_waypoint = fallback[0].copy()
        start_waypoint[:2] = self.start_pos[:2]
        if np.linalg.norm(fallback[0, :2] - self.start_pos[:2]) <= 0.05:
            fallback[0] = start_waypoint
        else:
            fallback = np.concatenate([start_waypoint.reshape((1, -1)), fallback], axis=0)
        return fallback

    def _reference_point_has_dynamic_obstacle(self, index):
        return self.planner.has_lethal_global_path_perception(
            self.reference_path[index : index + 1, :2],
            self.reference_layers[index : index + 1],
        )

    def _select_dynamic_anchor(self, preferred_anchor, use_dynamic):
        if not use_dynamic:
            return preferred_anchor
        if self.planner.global_path_perception_cell_count() <= 0:
            return preferred_anchor

        self.planner.set_global_path_perception_enabled(True)
        if not self._reference_point_has_dynamic_obstacle(preferred_anchor):
            return preferred_anchor

        for candidate in range(preferred_anchor + 1, len(self.reference_path)):
            if not self._reference_point_has_dynamic_obstacle(candidate):
                self.get_logger().info(
                    f"Dynamic anchor occupied at index={preferred_anchor}; "
                    f"moved forward to index={candidate}"
                )
                return candidate

        self.get_logger().warn(
            f"Dynamic anchor occupied at index={preferred_anchor}; "
            "no free forward anchor is available"
        )
        return preferred_anchor

    def _plan_forward_window(self, nearest):
        if nearest is None:
            return None
        join_distance = (
            self.local_replan_forward_distance
            + self.local_replan_join_extension
        )
        preferred_anchor = self._reference_anchor_index(
            self.reference_path,
            nearest,
            forward_distance=join_distance,
        )
        use_dynamic = self._global_path_perception_active()
        anchor = self._select_dynamic_anchor(
            preferred_anchor,
            use_dynamic,
        )
        goal_heading = self._reference_join_heading(self.reference_path, anchor)
        local_path = self.planner.plan(
            self.start_pos,
            self.reference_path[anchor, :3],
            use_dynamic=use_dynamic,
            search_bounds=None,
            goal_heading=goal_heading,
            start_layer_hint=int(self.reference_layers[nearest]),
            end_layer_hint=int(self.reference_layers[anchor]),
            start_height_hint=self._start_ground_height_hint(),
        )
        if local_path is None:
            if self._terrain_guard_active():
                self.get_logger().warn(
                    "Local PCT repair failed with a stair ahead; publishing the "
                    "remaining static reference path"
                )
                return self._static_reference_fallback(nearest)
            return None

        local_path = np.asarray(local_path, dtype=np.float32)
        suffix = self.reference_path[anchor + 1 :]
        if suffix.size == 0:
            combined = local_path
        else:
            combined = np.concatenate([local_path, suffix], axis=0)
        self.get_logger().debug(
            f"Local PCT repair: nearest={nearest}, preferred_anchor={preferred_anchor}, "
            f"anchor={anchor}, local={len(local_path)}, suffix={len(suffix)}, "
            f"forward={self.local_replan_forward_distance:.2f}m, "
            f"join_extension={self.local_replan_join_extension:.2f}m, "
            f"unbounded_search=1"
        )
        return combined

    def _plan_and_publish(self, reference_nearest):
        self.planning = True
        start_time = time.perf_counter()
        try:
            if self.local_replan_enabled:
                traj_3d = self._plan_forward_window(reference_nearest)
            else:
                nearest = reference_nearest
                traj_3d = self.planner.plan(
                    self.start_pos,
                    self.goal_pos,
                    use_dynamic=self._global_path_perception_active(),
                    start_height_hint=self._start_ground_height_hint(),
                )
                if (
                    traj_3d is None
                    and nearest is not None
                    and self._terrain_guard_active()
                ):
                    self.get_logger().warn(
                        "Full PCT planning failed with a stair ahead; publishing "
                        "the remaining static reference path"
                    )
                    traj_3d = self._static_reference_fallback(nearest)
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
            traj_np = np.asarray(traj_3d, dtype=np.float32)
            dt_ms = (time.perf_counter() - start_time) * 1000.0
            traj_first = traj_np[0, :3].tolist() if traj_np.ndim == 2 and traj_np.shape[0] > 0 else []
            plan_info = getattr(self.planner, "last_plan_info", {})
            geometry = plan_info.get("path_geometry", {})
            self.get_logger().info(
                f"Published /pct_path with {len(traj_3d)} poses in {dt_ms:.1f} ms, "
                f"start={self.start_pos.tolist()}, traj_first={traj_first}, "
                f"local_window={int(self.local_replan_enabled)}, "
                f"max_xy_step={geometry.get('max_xy_step', 0.0):.3f}m, "
                f"max_turn={geometry.get('max_turn_deg', 0.0):.1f}deg, "
                f"a_star_fallback={int(plan_info.get('used_a_star_fallback', False))}"
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
        if not self._global_path_perception_active():
            return
        if not self._global_path_perception_update_due():
            return

        ranges_raw = np.asarray(msg.ranges, dtype=np.float32)
        if ranges_raw.size == 0:
            return
        angles_raw = msg.angle_min + np.arange(ranges_raw.size, dtype=np.float32) * msg.angle_increment
        min_range = max(float(msg.range_min), self.global_path_perception_min_range)
        obstacle_range_max = self._global_path_perception_scan_max_range(msg)

        hit_valid = np.isfinite(ranges_raw)
        hit_valid &= ranges_raw >= min_range
        hit_valid &= ranges_raw <= obstacle_range_max
        hit_ranges = ranges_raw[hit_valid]
        hit_angles = angles_raw[hit_valid]
        hit_ranges, hit_angles = self._subsample_scan_vectors(
            hit_ranges, hit_angles, self.global_path_perception_max_points
        )

        hit_points = np.zeros((hit_ranges.size, 3), dtype=np.float32)
        if hit_ranges.size > 0:
            hit_points[:, 0] = hit_ranges * np.cos(hit_angles)
            hit_points[:, 1] = hit_ranges * np.sin(hit_angles)

        clear_points = None
        clear_hit_mask = None
        if self.global_path_perception_raytrace_enabled:
            clear_points, clear_hit_mask = self._build_global_path_perception_clear_rays(
                msg,
                ranges_raw,
                angles_raw,
                min_range,
            )

        self._apply_global_path_perception_points(
            hit_points,
            msg.header.frame_id,
            clear_points,
            clear_hit_mask,
            use_current_layer=True,
            stamp_msg=msg.header.stamp,
        )

    def _on_global_path_perception_cloud(self, msg):
        if not self._global_path_perception_active():
            return
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
        self._apply_global_path_perception_points(
            points_np,
            msg.header.frame_id,
            stamp_msg=msg.header.stamp,
        )

    def _global_path_perception_update_due(self):
        now = time.monotonic()
        if now - self.last_perception_update_wall_time < self.global_path_perception_update_interval:
            return False
        self.last_perception_update_wall_time = now
        return True

    def _global_path_perception_active(self):
        if not self.global_path_perception_enabled:
            return False
        return self.pct_dynamic_enabled

    def _terrain_guard_active(self):
        return (
            self.terrain_current in (TerrainState.STAIR_UP, TerrainState.STAIR_DOWN)
            or not self.pct_dynamic_enabled
        )

    def _on_navigation_mode(self, msg):
        previous_dynamic = self.pct_dynamic_enabled
        self.pct_dynamic_enabled = bool(msg.pct_dynamic_enabled)
        self.terrain_current = int(msg.current_terrain)
        if previous_dynamic and not self.pct_dynamic_enabled:
            self._clear_global_path_perception("external navigation mode")
        if previous_dynamic != self.pct_dynamic_enabled:
            self.get_logger().info(
                "PCT dynamic perception switched by navigation mode: "
                f"{int(previous_dynamic)} -> {int(self.pct_dynamic_enabled)}, "
                f"current={self.terrain_current}, upcoming={msg.upcoming_terrain}"
            )

    def _global_path_perception_scan_max_range(self, msg):
        range_max = self.global_path_perception_window_range
        if math.isfinite(float(msg.range_max)) and msg.range_max > 0.0:
            range_max = min(range_max, float(msg.range_max))
        return range_max

    def _build_global_path_perception_clear_rays(self, msg, ranges, angles, min_range):
        clear_range_max = min(
            self._global_path_perception_scan_max_range(msg),
            self.global_path_perception_raytrace_max_range,
        )
        if clear_range_max <= min_range:
            return None, None

        finite = np.isfinite(ranges)
        positive_inf = np.isposinf(ranges)
        clear_valid = positive_inf | (finite & (ranges >= min_range))
        if not np.any(clear_valid):
            return None, None

        clear_ranges = np.where(finite, np.minimum(ranges, clear_range_max), clear_range_max)
        clear_valid &= np.isfinite(clear_ranges)
        clear_valid &= clear_ranges >= min_range

        ray_ranges = clear_ranges[clear_valid]
        ray_angles = angles[clear_valid]
        hit_mask = finite[clear_valid] & (ranges[clear_valid] <= clear_range_max)
        if ray_ranges.size == 0:
            return None, None

        ray_ranges, ray_angles, hit_mask = self._subsample_scan_vectors(
            ray_ranges,
            ray_angles,
            self.global_path_perception_raytrace_max_rays,
            hit_mask,
        )
        clear_points = np.zeros((ray_ranges.size, 3), dtype=np.float32)
        clear_points[:, 0] = ray_ranges * np.cos(ray_angles)
        clear_points[:, 1] = ray_ranges * np.sin(ray_angles)
        return clear_points, hit_mask.astype(bool, copy=False)

    @staticmethod
    def _subsample_scan_vectors(ranges, angles, max_count, extra=None):
        if max_count <= 0 or ranges.size <= max_count:
            if extra is None:
                return ranges, angles
            return ranges, angles, extra
        step = max(1, int(math.ceil(ranges.size / max_count)))
        ranges = ranges[::step][:max_count]
        angles = angles[::step][:max_count]
        if extra is None:
            return ranges, angles
        return ranges, angles, extra[::step][:max_count]

    def _apply_global_path_perception_points(
        self,
        points_sensor,
        source_frame,
        clear_points_sensor=None,
        clear_hit_mask=None,
        use_current_layer=False,
        stamp_msg=None,
    ):
        source_frame = (source_frame or self.robot_frame).lstrip("/")
        transform = self._lookup_transform_to_map(source_frame, stamp_msg=stamp_msg)
        if transform is None:
            return

        self._update_start_from_stamped_tf(stamp_msg)
        points_map = self._transform_points_with_transform(points_sensor, transform)

        clear_indices = np.zeros((0, 3), dtype=np.int32)
        if clear_points_sensor is not None and clear_hit_mask is not None:
            clear_points_map = self._transform_points_with_transform(
                clear_points_sensor,
                transform,
            )
            origin_map = self._transform_origin_with_transform(transform)
            clear_indices = self._build_global_path_perception_clear_indices(
                origin_map,
                clear_points_map,
                clear_hit_mask,
                use_current_layer=use_current_layer,
            )

        perception_indices = self._build_global_path_perception_indices(
            points_map,
            use_current_layer=use_current_layer,
        )
        # Marking is already limited by the configured perception width/height.
        # Do not clip dynamic cells or A* search to a reference-path corridor.
        window_bounds = np.full(4, -1, dtype=np.int32)
        clear_center = self._current_robot_grid_index()
        changed_cells = self.planner.apply_global_path_perception(
            perception_indices,
            clear_indices,
            self.global_path_perception_inflation_radius,
            self.global_path_perception_inscribed_radius,
            self.global_path_perception_cost,
            self.global_path_perception_cost_scaling_factor,
            self._stamp_seconds(stamp_msg),
            self.global_path_perception_persistence,
            clear_center,
            -1.0,
            window_bounds,
        )
        if changed_cells < self.global_path_perception_min_changed_cells:
            self.last_perception_update_time = self.get_clock().now()
            return

        active_cells = self.planner.global_path_perception_cell_count()
        self.get_logger().info(
            f"Updated C++ PCT global path perception: active_cells={active_cells}, "
            f"changed_cells={changed_cells}, clear_cells={clear_indices.shape[0]}, "
            f"mark_cells={perception_indices.shape[0]}, "
            "path_corridor=disabled",
            throttle_duration_sec=1.0,
        )
        self.last_perception_update_time = self.get_clock().now()

    def _stamp_seconds(self, stamp_msg):
        if stamp_msg is None:
            return self._now_seconds()
        stamp = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1.0e-9
        return stamp if stamp > 0.0 else self._now_seconds()

    def _now_seconds(self):
        now = self.get_clock().now().nanoseconds
        return float(now) * 1.0e-9

    def _current_robot_grid_index(self):
        layer = int(
            self.planner.match_best_layer(
                self.start_pos[0],
                self.start_pos[1],
                self._start_ground_height_hint(),
            )
        )
        grid_idx = self.planner.pos2idx(self.start_pos[:2]).astype(int)
        return np.array([layer, int(grid_idx[1]), int(grid_idx[0])], dtype=np.int32)

    def _update_start_from_stamped_tf(self, stamp_msg):
        if self.start_source != "tf":
            return None
        transform = self._lookup_transform_to_map(
            self.robot_frame,
            stamp_msg=stamp_msg,
            context="Robot pose for dynamic layer",
        )
        if transform is None:
            return None
        translation = transform.transform.translation
        self.start_pos = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )
        return transform

    def _transform_points_to_map(self, points, source_frame):
        if points.size == 0:
            return points.reshape(0, 3).astype(np.float32, copy=False)
        transform = self._lookup_transform_to_map(source_frame)
        if transform is None:
            return None
        return self._transform_points_with_transform(points, transform)

    def _lookup_transform_to_map(
        self,
        source_frame,
        stamp_msg=None,
        context="Dynamic layer",
    ):
        target_frame = self.global_frame or self.frame_id
        lookup_time = Time()
        if stamp_msg is not None and (stamp_msg.sec != 0 or stamp_msg.nanosec != 0):
            lookup_time = Time.from_msg(stamp_msg)
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} waiting for TF {target_frame}<-{source_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def _transform_points_with_transform(self, points, transform):
        if points.size == 0:
            return points.reshape(0, 3).astype(np.float32, copy=False)
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
    def _transform_origin_with_transform(transform):
        translation = transform.transform.translation
        return np.array([translation.x, translation.y, translation.z], dtype=np.float32)

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

    def _build_global_path_perception_indices(self, points_map, use_current_layer=False):
        if points_map.size == 0:
            return np.zeros((0, 3), dtype=np.int32)
        if use_current_layer:
            return self._build_global_path_perception_indices_cpp(points_map)

        resolution = float(self.planner.resolution)
        if resolution <= 0.0:
            return np.zeros((0, 3), dtype=np.int32)
        layer_heights = self.planner.layer_elev_grids
        _, size_x, size_y = layer_heights.shape
        current_layer = self._global_path_perception_current_layer() if use_current_layer else None

        robot_xy = self.start_pos[:2].astype(np.float32, copy=False)
        indices = []
        for point in points_map:
            delta_xy = point[:2] - robot_xy
            if (
                abs(float(delta_xy[0])) > self.global_path_perception_half_width
                or abs(float(delta_xy[1])) > self.global_path_perception_half_height
            ):
                continue
            center_row, center_col = self._global_path_perception_row_col(point[:2])
            if center_row < 0 or center_row >= size_x or center_col < 0 or center_col >= size_y:
                continue

            point_height = float(point[2])
            candidate_layers = self._global_path_perception_candidate_layers(
                layer_heights,
                center_row,
                center_col,
                point_height,
                use_current_layer=use_current_layer,
                current_layer=current_layer,
            )
            if not candidate_layers:
                continue

            for layer in candidate_layers:
                indices.append((int(layer), center_row, center_col))

        if not indices:
            return np.zeros((0, 3), dtype=np.int32)
        return np.unique(np.asarray(indices, dtype=np.int32), axis=0)

    def _build_global_path_perception_indices_cpp(self, points_map):
        if points_map.size == 0:
            return np.zeros((0, 3), dtype=np.int32)

        robot_xy = self.start_pos[:2].astype(np.float32, copy=False)
        points_xy = points_map[:, :2].astype(np.float32, copy=False)
        delta_xy = points_xy - robot_xy
        mask = (
            (np.abs(delta_xy[:, 0]) <= self.global_path_perception_half_width)
            & (np.abs(delta_xy[:, 1]) <= self.global_path_perception_half_height)
        )
        if not np.any(mask):
            return np.zeros((0, 3), dtype=np.int32)

        mark_cells = self.planner.points2rowcol(points_xy[mask])
        if mark_cells.size == 0:
            return np.zeros((0, 3), dtype=np.int32)

        return np.asarray(
            self.planner.build_global_path_perception_mark_indices(
                mark_cells,
                self._global_path_perception_current_layer(),
                float(self.start_pos[2]),
                False,
                0.0,
                self.global_path_perception_height_tolerance,
                self.global_path_perception_mark_all_layers,
            ),
            dtype=np.int32,
        ).reshape((-1, 3))

    def _build_global_path_perception_clear_indices(
        self,
        origin_map,
        endpoints_map,
        hit_mask,
        use_current_layer=False,
    ):
        if endpoints_map is None or endpoints_map.size == 0:
            return np.zeros((0, 3), dtype=np.int32)
        if use_current_layer:
            return self._build_global_path_perception_clear_indices_cpp(
                origin_map,
                endpoints_map,
                hit_mask,
            )

        layer_heights = self.planner.layer_elev_grids
        n_layers, size_x, size_y = layer_heights.shape
        current_layer = self._global_path_perception_current_layer() if use_current_layer else None
        start_row, start_col = self._global_path_perception_row_col(origin_map[:2])
        if start_row < 0 or start_row >= size_x or start_col < 0 or start_col >= size_y:
            return np.zeros((0, 3), dtype=np.int32)

        indices = []
        for endpoint, ray_has_hit in zip(endpoints_map, hit_mask):
            end_row, end_col = self._global_path_perception_row_col(endpoint[:2])
            cells = self._bresenham_cells(start_row, start_col, end_row, end_col)
            if not cells:
                continue
            if ray_has_hit and len(cells) > 1:
                cells = cells[:-1]

            denom = max(1, len(cells) - 1)
            for offset, (row, col) in enumerate(cells):
                if row < 0 or row >= size_x or col < 0 or col >= size_y:
                    continue
                ratio = float(offset) / float(denom)
                point_height = float(origin_map[2] + ratio * (endpoint[2] - origin_map[2]))
                candidate_layers = self._global_path_perception_candidate_layers(
                    layer_heights,
                    row,
                    col,
                    point_height,
                    use_current_layer=use_current_layer,
                    current_layer=current_layer,
                )
                for layer in candidate_layers:
                    indices.append((int(layer), int(row), int(col)))

        if not indices:
            return np.zeros((0, 3), dtype=np.int32)
        return np.unique(np.asarray(indices, dtype=np.int32), axis=0)

    def _build_global_path_perception_clear_indices_cpp(
        self,
        origin_map,
        endpoints_map,
        hit_mask,
    ):
        origin_cell = self.planner.points2rowcol(np.asarray(origin_map[:2], dtype=np.float32).reshape(1, 2))[0]
        endpoint_cells = self.planner.points2rowcol(endpoints_map[:, :2])
        if endpoint_cells.size == 0:
            return np.zeros((0, 3), dtype=np.int32)
        hit_values = np.asarray(hit_mask, dtype=np.int32).reshape((-1, 1))
        endpoint_cells = np.concatenate([endpoint_cells, hit_values], axis=1)
        return np.asarray(
            self.planner.build_global_path_perception_clear_indices(
                origin_cell,
                endpoint_cells,
                self._global_path_perception_current_layer(),
                float(self.start_pos[2]),
                self.global_path_perception_height_tolerance,
                self.global_path_perception_mark_all_layers,
            ),
            dtype=np.int32,
        ).reshape((-1, 3))

    def _global_path_perception_row_col(self, point_xy):
        grid_idx = self.planner.pos2idx(np.asarray(point_xy, dtype=np.float32)).astype(int)
        return int(grid_idx[1]), int(grid_idx[0])

    @staticmethod
    def _bresenham_cells(row0, col0, row1, col1):
        cells = []
        dcol = abs(col1 - col0)
        drow = -abs(row1 - row0)
        step_col = 1 if col0 < col1 else -1
        step_row = 1 if row0 < row1 else -1
        error = dcol + drow
        row = row0
        col = col0

        while True:
            cells.append((row, col))
            if row == row1 and col == col1:
                break
            double_error = 2 * error
            if double_error >= drow:
                error += drow
                col += step_col
            if double_error <= dcol:
                error += dcol
                row += step_row
        return cells

    def _global_path_perception_current_layer(self):
        return int(
            self.planner.match_best_layer(
                float(self.start_pos[0]),
                float(self.start_pos[1]),
                self._start_ground_height_hint(),
            )
        )

    def _global_path_perception_candidate_layers(
        self,
        layer_heights,
        row,
        col,
        point_height,
        use_current_layer=False,
        current_layer=None,
    ):
        heights = layer_heights[:, row, col]
        valid = np.isfinite(heights) & (heights > -99.0)
        if not np.any(valid):
            return []
        if use_current_layer:
            if current_layer is None:
                current_layer = self._global_path_perception_current_layer()
            if 0 <= current_layer < layer_heights.shape[0] and bool(valid[current_layer]):
                return [current_layer]

            valid_indices = np.flatnonzero(valid)
            robot_height = float(self.start_pos[2])
            best = int(valid_indices[np.argmin(np.abs(heights[valid_indices] - robot_height))])
            return [best]
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
        if not self._global_path_perception_active():
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
            if self.planner.global_path_perception_cell_count() == 0:
                self.last_perception_update_time = None
            self.get_logger().info(
                f"Decayed stale C++ PCT global path perception after {age:.2f}s, "
                f"changed_cells={changed_cells}",
                throttle_duration_sec=1.0,
            )

    def _clear_global_path_perception(self, reason):
        if not self.global_path_perception_enabled:
            return
        try:
            active_cells = self.planner.global_path_perception_cell_count()
            self.planner.clear_global_path_perception()
            self.last_perception_update_time = None
            if active_cells > 0:
                self.get_logger().info(
                    f"Cleared PCT global path perception ({active_cells} cells) for {reason}"
                )
        except Exception as exc:
            self.get_logger().warn(f"Failed to clear PCT global path perception for {reason}: {exc}")

    def _on_goal_pose(self, msg):
        position = msg.pose.position
        self.goal_pos = np.array([position.x, position.y, position.z], dtype=np.float32)
        self.goal_received = True
        self._sync_marker_pose("end_pos", self.goal_pos)

    def _on_cancel_goal(self, _msg):
        self.goal_received = False
        self.last_planned_start = None
        self.last_planned_goal = None
        self.reference_path = None
        self.reference_layers = None
        self.reference_arc = None
        self.reference_goal = None
        self.reference_progress_index = 0

        empty = Path()
        empty.header.frame_id = self.frame_id
        empty.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty)
        self.reference_path_pub.publish(empty)
        self.get_logger().info("Cancelled active PCT goal and cleared published paths")

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
        self.goal_received = True
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
            self.goal_received = True

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
