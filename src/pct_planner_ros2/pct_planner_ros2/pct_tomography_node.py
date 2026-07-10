import os
import pickle
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .pct_paths import configure_tomography_imports, default_pct_root, expand_path, tomogram_stem


SCENE_DEFAULTS = {
    "Building": {
        "pcd_file": "building2_9.pcd",
        "resolution": 0.1,
        "ground_h": 0.0,
        "slice_dh": 0.5,
        "kernel_size": 7,
        "interval_min": 0.5,
        "interval_free": 0.5,
        "slope_max": 0.4,
        "step_max": 0.3,
        "standable_ratio": 0.5,
        "cost_barrier": 50.0,
        "safe_margin": 0.5,
        "safe_margin_gamma": 1.0,
        "inflation": 0.1,
    },
    "Spiral": {
        "pcd_file": "spiral0.3_2.pcd",
        "resolution": 0.2,
        "ground_h": 0.0,
        "slice_dh": 0.5,
        "kernel_size": 7,
        "interval_min": 0.5,
        "interval_free": 0.65,
        "slope_max": 0.4,
        "step_max": 0.3,
        "standable_ratio": 0.4,
        "cost_barrier": 50.0,
        "safe_margin": 1.2,
        "safe_margin_gamma": 1.0,
        "inflation": 0.2,
    },
    "Plaza": {
        "pcd_file": "plaza3_10.pcd",
        "resolution": 0.1,
        "ground_h": 0.0,
        "slice_dh": 0.5,
        "kernel_size": 7,
        "interval_min": 0.5,
        "interval_free": 0.65,
        "slope_max": 0.36,
        "step_max": 0.17,
        "standable_ratio": 0.2,
        "cost_barrier": 50.0,
        "safe_margin": 0.4,
        "safe_margin_gamma": 1.0,
        "inflation": 0.2,
    },
}

POINT_FIELDS_XYZI = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def grid_points_xyzi(resolution, dim_x, dim_y):
    index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
    lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
    ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
    ix, iy = np.meshgrid(lx, ly)
    index_proto[:, 0] = ix.flatten()
    index_proto[:, 1] = iy.flatten()

    point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
    point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
    point_proto[:, 0] -= 0.5 * dim_x
    point_proto[:, 1] -= 0.5 * dim_y
    point_proto[:, :2] *= resolution
    point_proto[:, 3] = 1.0
    return index_proto, point_proto


def _pcd_scalar_dtype(type_code, size):
    dtype_codes = {
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
    try:
        return np.dtype(dtype_codes[(type_code.upper(), int(size))])
    except KeyError as exc:
        raise RuntimeError(
            f"Unsupported PCD field type: TYPE={type_code}, SIZE={size}"
        ) from exc


def read_pcd_xyz(pcd_file):
    path = Path(pcd_file)
    if not path.is_file():
        raise FileNotFoundError(f"PCD file not found: {path}")

    with path.open("rb") as stream:
        header = {}
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise RuntimeError(f"PCD header is incomplete: {path}")
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise RuntimeError(f"PCD header is not valid ASCII: {path}") from exc
            if not line or line.startswith("#"):
                continue

            key, *values = line.split()
            key = key.upper()
            header[key] = values
            if key == "DATA":
                break

        fields = header.get("FIELDS") or header.get("FIELD")
        if not fields:
            raise RuntimeError(f"PCD header has no FIELDS entry: {path}")
        field_names = [name.lower() for name in fields]
        for required_name in ("x", "y", "z"):
            if required_name not in field_names:
                raise RuntimeError(
                    f"PCD file is missing required field '{required_name}': {path}"
                )

        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise RuntimeError(f"PCD field metadata lengths do not match: {path}")

        point_count = int(
            (header.get("POINTS") or [
                int(header.get("WIDTH", ["0"])[0])
                * int(header.get("HEIGHT", ["1"])[0])
            ])[0]
        )
        data_format = header["DATA"][0].lower()

        if data_format == "binary":
            dtype_fields = []
            for name, size, type_code, count in zip(fields, sizes, types, counts):
                scalar_dtype = _pcd_scalar_dtype(type_code, size)
                if count == 1:
                    dtype_fields.append((name, scalar_dtype))
                else:
                    dtype_fields.append((name, scalar_dtype, (count,)))
            records = np.fromfile(stream, dtype=np.dtype(dtype_fields), count=point_count)
            if records.shape[0] != point_count:
                raise RuntimeError(
                    f"PCD binary payload is truncated: expected {point_count} points, "
                    f"read {records.shape[0]} from {path}"
                )
            field_lookup = {name.lower(): name for name in fields}
            points = np.column_stack(
                [
                    records[field_lookup["x"]],
                    records[field_lookup["y"]],
                    records[field_lookup["z"]],
                ]
            )
        elif data_format == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            field_offsets = {}
            offset = 0
            for name, count in zip(field_names, counts):
                field_offsets[name] = offset
                offset += count
            points = values[
                :,
                [
                    field_offsets["x"],
                    field_offsets["y"],
                    field_offsets["z"],
                ],
            ]
        elif data_format == "binary_compressed":
            raise RuntimeError(
                "PCD DATA binary_compressed is not supported by the built-in loader. "
                "Convert the map to DATA binary or DATA ascii first."
            )
        else:
            raise RuntimeError(f"Unsupported PCD DATA format '{data_format}': {path}")

    points = np.asarray(points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    if points.size == 0:
        raise RuntimeError(f"PCD file has no finite XYZ points: {path}")
    return points


class PctTomographyNode(Node):
    def __init__(self):
        super().__init__("pct_tomography_node")

        self.declare_parameter("pct_root", default_pct_root())
        self.declare_parameter("scene_name", "Building")
        self.declare_parameter("pcd_file", "")
        self.declare_parameter("output_tomogram_name", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_visualization", True)
        self.declare_parameter("benchmark_repeats", 10)
        self.declare_parameter("pointcloud_topic", "/global_points")
        self.declare_parameter("layer_g_topic_prefix", "/layer_G_")
        self.declare_parameter("layer_c_topic_prefix", "/layer_C_")
        self.declare_parameter("tomogram_topic", "/tomogram")
        self.declare_parameter("exit_after_process", False)

        scene_defaults = SCENE_DEFAULTS.get(
            self.get_parameter("scene_name").value,
            SCENE_DEFAULTS["Building"],
        )
        for name, default in scene_defaults.items():
            if name != "pcd_file":
                self.declare_parameter(name, default)

        self.pct_root = expand_path(self.get_parameter("pct_root").value)
        self.scene_cfg = self._make_scene_config()
        self.map_frame = self.get_parameter("map_frame").value
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)
        self.exit_after_process = bool(self.get_parameter("exit_after_process").value)

        configure_tomography_imports(self.pct_root)
        try:
            from tomogram import Tomogram
        except Exception as exc:
            raise RuntimeError(
                "Failed to import PCT tomography modules. Check PCT_PLANNER_ROOT, PYTHONPATH, "
                "and that the m20_nav_cupy venv is visible on PYTHONPATH."
            ) from exc

        self.tomogram = Tomogram(self.scene_cfg)
        self.pointcloud_pub = None
        self.tomogram_pub = None
        self.layer_g_pub_list = []
        self.layer_c_pub_list = []

        self._process()

    def _make_scene_config(self):
        scene_name = self.get_parameter("scene_name").value
        defaults = SCENE_DEFAULTS.get(scene_name, SCENE_DEFAULTS["Building"]).copy()
        pcd_file = self.get_parameter("pcd_file").value or defaults["pcd_file"]

        pcd_path = Path(os.path.expandvars(os.path.expanduser(str(pcd_file))))
        if not pcd_path.is_absolute():
            pcd_path = self.pct_root / "rsc/pcd" / pcd_path

        map_cfg = SimpleNamespace(
            resolution=float(self.get_parameter("resolution").value),
            ground_h=float(self.get_parameter("ground_h").value),
            slice_dh=float(self.get_parameter("slice_dh").value),
        )
        trav_cfg = SimpleNamespace(
            kernel_size=int(self.get_parameter("kernel_size").value),
            interval_min=float(self.get_parameter("interval_min").value),
            interval_free=float(self.get_parameter("interval_free").value),
            slope_max=float(self.get_parameter("slope_max").value),
            step_max=float(self.get_parameter("step_max").value),
            standable_ratio=float(self.get_parameter("standable_ratio").value),
            cost_barrier=float(self.get_parameter("cost_barrier").value),
            safe_margin=float(self.get_parameter("safe_margin").value),
            safe_margin_gamma=float(self.get_parameter("safe_margin_gamma").value),
            inflation=float(self.get_parameter("inflation").value),
        )
        output_name = self.get_parameter("output_tomogram_name").value
        if not output_name:
            output_name = tomogram_stem(pcd_path.stem)
        output_name = tomogram_stem(output_name)

        return SimpleNamespace(
            pcd=SimpleNamespace(file_name=str(pcd_path), output_name=output_name),
            map=map_cfg,
            trav=trav_cfg,
        )

    def _process(self):
        points = self._load_pcd(self.scene_cfg.pcd.file_name)
        layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c = self._build_tomogram(points)
        self._export_tomogram(
            np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)),
            self.scene_cfg.pcd.output_name,
        )

        if self.publish_visualization:
            self._init_publishers(layers_g.shape[0])
            self._publish_points(points)
            self._publish_layers(self.layer_g_pub_list, layers_g, layers_t)
            self._publish_layers(self.layer_c_pub_list, layers_c, None)
            self._publish_tomogram(layers_g, layers_t)

    def _load_pcd(self, pcd_file):
        points = read_pcd_xyz(pcd_file)

        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)
        self.points_min[-1] = self.scene_cfg.map.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.scene_cfg.map.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.scene_cfg.map.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.scene_cfg.map.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.scene_cfg.map.slice_dh
        self.tomogram.initMappingEnv(
            self.center,
            self.map_dim_x,
            self.map_dim_y,
            n_slice_init,
            self.slice_h0,
        )
        self.visproto_i, self.visproto_p = grid_points_xyzi(
            self.scene_cfg.map.resolution,
            self.map_dim_x,
            self.map_dim_y,
        )

        self.get_logger().info(f"PCD points: {points.shape[0]}")
        self.get_logger().info(f"Map center: [{self.center[0]:.2f}, {self.center[1]:.2f}]")
        self.get_logger().info(f"Dim_x: {self.map_dim_x}, Dim_y: {self.map_dim_y}, init slices: {n_slice_init}")
        return points

    def _build_tomogram(self, points):
        repeat_count = max(1, int(self.get_parameter("benchmark_repeats").value))
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        latest = None

        for i in range(repeat_count + 1):
            t_start = time.time()
            latest = self.tomogram.point2map(points)
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = latest
            if i > 0:
                t_map += t_gpu["t_map"]
                t_trav += t_gpu["t_trav"]
                t_simp += t_gpu["t_simp"]
                t_all += (time.time() - t_start) * 1e3

        layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, _ = latest
        self.get_logger().info(f"Simplified slices: {layers_g.shape[0]}")
        self.get_logger().info(f"Benchmark repeats: {repeat_count}")
        self.get_logger().info(f"avg t_map  (ms): {t_map / repeat_count:.3f}")
        self.get_logger().info(f"avg t_trav (ms): {t_trav / repeat_count:.3f}")
        self.get_logger().info(f"avg t_simp (ms): {t_simp / repeat_count:.3f}")
        self.get_logger().info(f"avg t_all  (ms): {t_all / repeat_count:.3f}")
        return layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c

    def _export_tomogram(self, tomogram, map_file):
        output_dir = self.pct_root / "rsc/tomogram"
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{tomogram_stem(map_file)}.pickle"
        data_dict = {
            "data": tomogram.astype(np.float16),
            "resolution": self.scene_cfg.map.resolution,
            "center": self.center,
            "slice_h0": self.slice_h0,
            "slice_dh": self.scene_cfg.map.slice_dh,
        }
        with output_path.open("wb") as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
        self.get_logger().info(f"Tomogram exported: {output_path}")

    def _init_publishers(self, n_slice):
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
        layer_g_prefix = self.get_parameter("layer_g_topic_prefix").value
        layer_c_prefix = self.get_parameter("layer_c_topic_prefix").value
        for i in range(n_slice):
            self.layer_g_pub_list.append(self.create_publisher(PointCloud2, f"{layer_g_prefix}{i}", qos))
            self.layer_c_pub_list.append(self.create_publisher(PointCloud2, f"{layer_c_prefix}{i}", qos))

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        return header

    def _publish_points(self, points):
        self.pointcloud_pub.publish(point_cloud2.create_cloud_xyz32(self._header(), points))

    def _publish_layers(self, pub_list, layers, color):
        layer_points = self.visproto_p.copy()
        layer_points[:, :2] += self.center
        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            else:
                layer_points[:, 3] = 1.0
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            pub_list[i].publish(point_cloud2.create_cloud(self._header(), POINT_FIELDS_XYZI, valid_points))

    def _publish_tomogram(self, layers_g, layers_t):
        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy()
        layer_points = self.visproto_p.copy()
        layer_points[:, :2] += self.center

        global_points = None
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.scene_cfg.map.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points[:, 2] = vis_g[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            layer_points[:, 3] = vis_t[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            global_points = valid_points if global_points is None else np.concatenate((global_points, valid_points), axis=0)

        layer_points[:, 2] = vis_g[-1, self.visproto_i[:, 0], self.visproto_i[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.visproto_i[:, 0], self.visproto_i[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        global_points = valid_points if global_points is None else np.concatenate((global_points, valid_points), axis=0)
        self.tomogram_pub.publish(point_cloud2.create_cloud(self._header(), POINT_FIELDS_XYZI, global_points))


def main(args=None):
    rclpy.init(args=args)
    node = PctTomographyNode()
    try:
        if not node.exit_after_process:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
