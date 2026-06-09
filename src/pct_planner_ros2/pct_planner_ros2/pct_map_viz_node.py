import pickle
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .pct_paths import default_pct_root, expand_path, tomogram_stem
from .pct_tomography_node import POINT_FIELDS_XYZI, grid_points_xyzi


class PctMapVizNode(Node):
    def __init__(self):
        super().__init__("pct_map_viz_node")

        self.declare_parameter("pct_root", default_pct_root())
        self.declare_parameter("pcd_file", "/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd")
        self.declare_parameter("tomogram_file", "m20_3d_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pointcloud_topic", "/global_points")
        self.declare_parameter("tomogram_topic", "/tomogram")
        self.declare_parameter("publish_pcd", True)
        self.declare_parameter("publish_tomogram", True)
        self.declare_parameter("tomogram_visual_cost_max", 50.0)
        self.declare_parameter("republish_period", 5.0)

        self.pct_root = expand_path(self.get_parameter("pct_root").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.tomogram_visual_cost_max = float(self.get_parameter("tomogram_visual_cost_max").value)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("pointcloud_topic").value,
            qos,
        )
        self.tomogram_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("tomogram_topic").value,
            qos,
        )

        self.pcd_msg = self._make_pcd_msg() if self.get_parameter("publish_pcd").value else None
        self.tomogram_msg = (
            self._make_tomogram_msg()
            if self.get_parameter("publish_tomogram").value
            else None
        )

        self._publish()
        period = max(0.5, float(self.get_parameter("republish_period").value))
        self.timer = self.create_timer(period, self._publish)

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        return header

    def _stamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        return msg

    def _make_pcd_msg(self):
        import open3d as o3d

        pcd_path = expand_path(self.get_parameter("pcd_file").value)
        points = np.asarray(o3d.io.read_point_cloud(str(pcd_path)).points).astype(np.float32)
        if points.size == 0:
            raise RuntimeError(f"PCD file has no points: {pcd_path}")
        points = points[:, :3]
        self.get_logger().info(f"Loaded PCD for RViz: {pcd_path} ({points.shape[0]} points)")
        return point_cloud2.create_cloud_xyz32(self._header(), points)

    def _make_tomogram_msg(self):
        tomogram_file = tomogram_stem(self.get_parameter("tomogram_file").value)
        tomogram_path = self.pct_root / "rsc/tomogram" / f"{tomogram_file}.pickle"
        if not tomogram_path.exists():
            raise RuntimeError(f"Tomogram pickle does not exist: {tomogram_path}")

        with tomogram_path.open("rb") as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict["data"], dtype=np.float32)
        resolution = float(data_dict["resolution"])
        center = np.asarray(data_dict["center"], dtype=np.float32)

        layers_t = tomogram[0]
        layers_g = tomogram[3]
        index_proto, point_proto = grid_points_xyzi(
            resolution,
            layers_g.shape[1],
            layers_g.shape[2],
        )
        point_proto[:, :2] += center

        global_points = None
        vis_g = layers_g.copy()
        vis_t = layers_t.copy()
        for i in range(layers_g.shape[0] - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < float(data_dict["slice_dh"])
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points = self._layer_points(point_proto, index_proto, vis_g[i], vis_t[i])
            global_points = layer_points if global_points is None else np.concatenate((global_points, layer_points), axis=0)

        layer_points = self._layer_points(point_proto, index_proto, vis_g[-1], vis_t[-1])
        global_points = layer_points if global_points is None else np.concatenate((global_points, layer_points), axis=0)
        self.get_logger().info(
            f"Loaded tomogram for RViz: {tomogram_path} "
            f"({layers_g.shape[0]} layers, {global_points.shape[0]} visible cells, "
            f"visual_cost_max={self.tomogram_visual_cost_max:.1f})"
        )
        return point_cloud2.create_cloud(self._header(), POINT_FIELDS_XYZI, global_points)

    def _layer_points(self, point_proto, index_proto, height_grid, intensity_grid):
        layer_points = point_proto.copy()
        layer_points[:, 2] = height_grid[index_proto[:, 0], index_proto[:, 1]]
        layer_points[:, 3] = intensity_grid[index_proto[:, 0], index_proto[:, 1]]
        valid = ~np.isnan(layer_points).any(axis=-1)
        if self.tomogram_visual_cost_max > 0:
            valid &= layer_points[:, 3] <= self.tomogram_visual_cost_max
        return layer_points[valid]

    def _publish(self):
        if self.pcd_msg is not None:
            self.pointcloud_pub.publish(self._stamp(self.pcd_msg))
        if self.tomogram_msg is not None:
            self.tomogram_pub.publish(self._stamp(self.tomogram_msg))


def main(args=None):
    rclpy.init(args=args)
    node = PctMapVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
