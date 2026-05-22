#!/usr/bin/env python3
"""
Rotate a PCD map to level it using the base_link -> livox_frame extrinsics.
This compensates for the tilted MID360 mounting so that Z-slicing in pcd2pgm
produces a level 2D occupancy map.

Supported usage:
    python3 level_pcd.py /path/to/in.pcd /path/to/out.pcd
    python3 level_pcd.py --input /path/to/in.pcd --output /path/to/out.pcd
"""
import argparse
import os
import sys
import numpy as np

QUATERNION = [-0.00394028, 0.24367785, 0.00970223, 0.96979969]  # x,y,z,w
DEFAULT_INPUT = "/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd"
DEFAULT_OUTPUT = "/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd"


def parse_args():
    parser = argparse.ArgumentParser(description="Rotate a PCD map into a gravity-aligned frame.")
    parser.add_argument("input_pos", nargs="?", help="Input PCD file path.")
    parser.add_argument("output_pos", nargs="?", help="Output PCD file path.")
    parser.add_argument("--input", dest="input_opt", help="Input PCD file path.")
    parser.add_argument("--output", dest="output_opt", help="Output PCD file path.")
    args = parser.parse_args()

    in_path = args.input_opt or args.input_pos or DEFAULT_INPUT
    if args.output_opt:
        out_path = args.output_opt
    elif args.output_pos:
        out_path = args.output_pos
    elif in_path.lower().endswith(".pcd"):
        out_path = in_path[:-4] + "_leveled.pcd"
    else:
        out_path = DEFAULT_OUTPUT

    return in_path, out_path


def main():
    in_path, out_path = parse_args()

    try:
        import open3d as o3d
    except ImportError as exc:
        print("Failed to import open3d. Activate the Python environment that has open3d installed.")
        raise SystemExit(1) from exc

    if not os.path.exists(in_path):
        print(f"Input file does not exist: {in_path}")
        raise SystemExit(1)

    # Load PCD
    print(f"Loading {in_path}...")
    pcd = o3d.io.read_point_cloud(in_path)
    print(f"  Points: {len(pcd.points)}")
    if len(pcd.points) == 0:
        print("Input point cloud is empty or could not be parsed.")
        raise SystemExit(1)

    # The extrinsics define base_link -> livox_frame.
    # camera_init frame ≈ body frame ≈ livox_frame (orientation-wise).
    # To level, rotate points from livox-frame orientation to base-link orientation.
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(QUATERNION).as_matrix()

    pts = np.asarray(pcd.points)
    pts_leveled = (R @ pts.T).T  # rotate each point
    pcd.points = o3d.utility.Vector3dVector(pts_leveled)

    print(f"Saving {out_path}...")
    if not o3d.io.write_point_cloud(out_path, pcd):
        print(f"Failed to write point cloud: {out_path}")
        raise SystemExit(1)
    print("Done.")


if __name__ == "__main__":
    main()
