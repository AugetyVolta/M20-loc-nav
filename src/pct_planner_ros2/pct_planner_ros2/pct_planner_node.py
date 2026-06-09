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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
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
        self.declare_parameter("a_star_cost_threshold", 20.0)
        self.declare_parameter("safe_cost_margin", 15.0)
        self.declare_parameter("step_cost_weight", 1.0)
        self.declare_parameter("layer_match_height_tolerance", 1.0)

        self.pct_root = self.get_parameter("pct_root").value
        self.tomogram_file = tomogram_stem(self.get_parameter("tomogram_file").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.start_source = self.get_parameter("start_source").value
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.marker_z_offset = float(self.get_parameter("marker_z_offset").value)
        self.position_epsilon = float(self.get_parameter("position_epsilon").value)
        self.auto_plan = bool(self.get_parameter("auto_plan").value)

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

    def _load_pct_core(self):
        missing = missing_ld_library_dirs(self.pct_root)
        if missing:
            self.get_logger().warn(
                "PCT library paths are not in LD_LIBRARY_PATH. Source scripts/pct_env.sh before running. "
                + "Missing: "
                + ", ".join(missing)
            )

        configure_planner_imports(self.pct_root)
        for module_name in ("config", "utils", "planner_wrapper", "a_star", "ele_planner", "traj_opt"):
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
        cfg.planner.layer_match_height_tolerance = float(
            self.get_parameter("layer_match_height_tolerance").value
        )
        self.planner = tomogram_planner(cfg)

    def _timer_cb(self):
        if not self.auto_plan or self.planning:
            return
        if not self._update_start_from_tf():
            return
        if not self._position_changed():
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
