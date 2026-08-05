import pickle
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .pct_paths import default_pct_root, expand_path, tomogram_stem
from .pct_tomography_node import POINT_FIELDS_XYZI
from .tomogram_editor_core import (
    WallSegment,
    apply_virtual_walls,
    save_edited_tomogram,
    snap_point_to_surface,
)
from .tomogram_surface import build_surface_points, export_surface_from_pickle


class PctTomogramEditorNode(Node):
    def __init__(self):
        super().__init__("pct_tomogram_editor")
        self.declare_parameter("pct_root", default_pct_root())
        self.declare_parameter("tomogram_file", "m20_3d_map")
        self.declare_parameter("output_tomogram_name", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("clicked_point_topic", "/pct_tomogram_editor/clicked_point")
        self.declare_parameter("preview_topic", "/tomogram")
        self.declare_parameter("wall_marker_topic", "/pct_tomogram_editor/walls")
        self.declare_parameter("wall_width", 0.30)
        self.declare_parameter("inflation_radius", 0.50)
        self.declare_parameter("cost_scaling_factor", 5.0)
        self.declare_parameter("barrier_cost", 50.0)
        self.declare_parameter("layer_height_tolerance", 0.75)
        self.declare_parameter("surface_snap_radius", 0.50)

        self.pct_root = expand_path(self.get_parameter("pct_root").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.source_path = self._resolve_source_path(
            self.get_parameter("tomogram_file").value
        )
        output_name = str(self.get_parameter("output_tomogram_name").value).strip()
        if not output_name:
            output_name = f"{self.source_path.stem}_edited"
        self.output_path = self._resolve_output_path(output_name)
        if self.output_path.resolve() == self.source_path.resolve():
            raise RuntimeError("output_tomogram_name must not overwrite the source tomogram")

        with self.source_path.open("rb") as handle:
            self.source_data = pickle.load(handle)
        self.segments = []
        self.pending_point = None
        self.edited_data = self.source_data
        self.last_stats = {"matched_cells": 0, "hard_cells": 0}

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.preview_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("preview_topic").value),
            latched_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("wall_marker_topic").value),
            latched_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self._on_clicked_point,
            10,
        )
        self.create_service(Trigger, "~/undo", self._on_undo)
        self.create_service(Trigger, "~/clear", self._on_clear)
        self.create_service(Trigger, "~/save", self._on_save)

        self._publish_preview()
        self._publish_markers()
        self.get_logger().info(
            f"Tomogram editor loaded {self.source_path}; output={self.output_path}"
        )
        self.get_logger().info(
            "Use RViz Publish Point twice per wall segment, then call ~/save"
        )

    def _resolve_source_path(self, value):
        candidate = expand_path(value)
        if candidate.is_file():
            return candidate
        path = self.pct_root / "rsc/tomogram" / f"{tomogram_stem(value)}.pickle"
        if not path.is_file():
            raise RuntimeError(f"Tomogram pickle does not exist: {path}")
        return path

    def _resolve_output_path(self, value):
        candidate = Path(str(value)).expanduser()
        if candidate.is_absolute() or candidate.parent != Path("."):
            return candidate.with_suffix(".pickle") if not candidate.suffix else candidate
        return self.source_path.parent / f"{tomogram_stem(candidate.name)}.pickle"

    def _editor_parameters(self):
        return {
            "wall_width": float(self.get_parameter("wall_width").value),
            "inflation_radius": float(self.get_parameter("inflation_radius").value),
            "cost_scaling_factor": float(
                self.get_parameter("cost_scaling_factor").value
            ),
            "barrier_cost": float(self.get_parameter("barrier_cost").value),
            "layer_height_tolerance": float(
                self.get_parameter("layer_height_tolerance").value
            ),
        }

    def _on_clicked_point(self, message):
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"Clicked point frame is {message.header.frame_id}, expected {self.frame_id}"
            )
            return
        raw = np.array(
            [message.point.x, message.point.y, message.point.z], dtype=np.float64
        )
        snapped, found = snap_point_to_surface(
            self.source_data,
            raw,
            search_radius=float(self.get_parameter("surface_snap_radius").value),
        )
        if not found:
            self.get_logger().warning(
                "No tomogram ground surface near clicked point; keeping clicked Z"
            )
        if self.pending_point is None:
            self.pending_point = snapped
            self.get_logger().info(
                "Wall start: " + ", ".join(f"{value:.3f}" for value in snapped)
            )
            self._publish_markers()
            return

        if np.linalg.norm(snapped[:2] - self.pending_point[:2]) < 0.05:
            self.get_logger().warning("Wall segment is shorter than 0.05m; point ignored")
            return
        self.segments.append(WallSegment.from_points(self.pending_point, snapped))
        self.pending_point = None
        self._rebuild()
        self.get_logger().info(
            f"Added wall {len(self.segments)}; matched={self.last_stats['matched_cells']} "
            f"hard={self.last_stats['hard_cells']}"
        )

    def _rebuild(self):
        self.edited_data, self.last_stats = apply_virtual_walls(
            self.source_data,
            self.segments,
            **self._editor_parameters(),
        )
        self._publish_preview()
        self._publish_markers()

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def _publish_preview(self):
        data = np.asarray(self.edited_data["data"], dtype=np.float32)
        points = build_surface_points(
            data[0],
            data[3],
            float(self.edited_data["resolution"]),
            self.edited_data["center"],
            float(self.edited_data["slice_dh"]),
        )
        message = point_cloud2.create_cloud(
            self._header(), POINT_FIELDS_XYZI, points
        )
        self.preview_pub.publish(message)

    def _point(self, values, z_offset=0.06):
        point = Point()
        point.x = float(values[0])
        point.y = float(values[1])
        point.z = float(values[2]) + z_offset
        return point

    def _line_marker(self, marker_id, segment, width, color):
        marker = Marker()
        marker.header = self._header()
        marker.ns = "pct_tomogram_virtual_walls"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.01, float(width))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [self._point(segment.start), self._point(segment.end)]
        return marker

    def _publish_markers(self):
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header = self._header()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        parameters = self._editor_parameters()
        for index, segment in enumerate(self.segments):
            outer_width = parameters["wall_width"] + 2.0 * parameters["inflation_radius"]
            markers.markers.append(
                self._line_marker(2 * index, segment, outer_width, (1.0, 0.55, 0.0, 0.22))
            )
            markers.markers.append(
                self._line_marker(
                    2 * index + 1,
                    segment,
                    parameters["wall_width"],
                    (1.0, 0.05, 0.05, 0.95),
                )
            )
        if self.pending_point is not None:
            marker = Marker()
            marker.header = self._header()
            marker.ns = "pct_tomogram_virtual_walls"
            marker.id = 1000000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = self._point(self.pending_point)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.20
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.0
            marker.color.a = 1.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _on_undo(self, _request, response):
        if self.pending_point is not None:
            self.pending_point = None
            self._publish_markers()
            response.success = True
            response.message = "Cancelled pending wall start"
        elif self.segments:
            self.segments.pop()
            self._rebuild()
            response.success = True
            response.message = f"Removed last wall; remaining={len(self.segments)}"
        else:
            response.success = False
            response.message = "No wall or pending point to undo"
        return response

    def _on_clear(self, _request, response):
        self.pending_point = None
        self.segments.clear()
        self._rebuild()
        response.success = True
        response.message = "Cleared all virtual walls"
        return response

    def _on_save(self, _request, response):
        if not self.segments:
            response.success = False
            response.message = "No virtual walls to save"
            return response
        if self.last_stats["hard_cells"] == 0:
            response.success = False
            response.message = "No wall cells matched a tomogram surface; nothing saved"
            return response
        try:
            destination, record = save_edited_tomogram(
                self.output_path,
                self.edited_data,
                source_path=self.source_path,
                segments=self.segments,
                parameters=self._editor_parameters(),
                frame_id=self.frame_id,
            )
            surface, point_count = export_surface_from_pickle(
                destination,
                frame_id=self.frame_id,
            )
        except Exception as error:
            response.success = False
            response.message = f"Save failed: {error}"
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = (
            f"Saved {destination}, {record}, {surface} ({point_count} points)"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PctTomogramEditorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
