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


def _read_pcd_xyz32(pcd_path: Path) -> np.ndarray:
    with pcd_path.open("rb") as handle:
        header_lines = []
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError(f"PCD header is incomplete: {pcd_path}")
            text = line.decode("utf-8", errors="replace").strip()
            header_lines.append(text)
            if text.startswith("DATA"):
                data_offset = handle.tell()
                break

    header = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]

    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(fields):
        raise RuntimeError(f"PCD must contain x/y/z fields: {pcd_path}")
    data_kind = header.get("DATA", [""])[0].lower()
    if data_kind not in {"ascii", "binary"}:
        raise RuntimeError(f"Unsupported PCD DATA type '{data_kind}' in {pcd_path}")

    if data_kind == "ascii":
        raw = np.loadtxt(str(pcd_path), comments="#", skiprows=len(header_lines), dtype=np.float32)
        if raw.ndim == 1:
            raw = raw.reshape(1, -1)
        return raw[:, [fields.index("x"), fields.index("y"), fields.index("z")]].astype(np.float32)

    sizes = [int(v) for v in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    point_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
    if len(sizes) != len(fields) or len(types) != len(fields) or len(counts) != len(fields):
        raise RuntimeError(f"PCD binary field metadata is incomplete: {pcd_path}")

    dtype_fields = []
    type_map = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "<i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "<u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    for field, field_type, size, count in zip(fields, types, sizes, counts):
        dtype = type_map.get((field_type.upper(), size))
        if dtype is None:
            raise RuntimeError(f"Unsupported PCD field type {field_type}{size} for {field}: {pcd_path}")
        dtype_fields.append((field, dtype, (count,) if count > 1 else ()))

    structured_dtype = np.dtype(dtype_fields)
    with pcd_path.open("rb") as handle:
        handle.seek(data_offset)
        cloud = np.fromfile(handle, dtype=structured_dtype, count=point_count)
    if cloud.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float32)


class PctMapVizNode(Node):
    def __init__(self):
        super().__init__("pct_map_viz_node")

        self.declare_parameter("pct_root", default_pct_root())
        self.declare_parameter("pcd_file", "/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/m20_3d_map.pcd")
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
        pcd_path = expand_path(self.get_parameter("pcd_file").value)
        try:
            import open3d as o3d

            points = np.asarray(o3d.io.read_point_cloud(str(pcd_path)).points).astype(np.float32)
        except ModuleNotFoundError:
            points = _read_pcd_xyz32(pcd_path)
            self.get_logger().warn(
                "open3d is not installed; loaded PCD with the built-in x/y/z reader"
            )
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
