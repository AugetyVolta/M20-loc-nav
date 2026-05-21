#!/usr/bin/env python3
"""
Rotate a PCD map to level it using the base_link -> livox_frame extrinsics.
This compensates for the tilted MID360 mounting so that Z-slicing in pcd2pgm
produces a level 2D occupancy map.

Usage:
    python3 level_pcd.py /path/to/m20_map.pcd /path/to/m20_map_leveled.pcd
"""
import sys
import numpy as np
import open3d as o3d

# base_link -> livox_frame extrinsics (same as in launch files)
TRANSLATION = [0.32713234, 0.01413551, 0.31238696]
QUATERNION = [-0.00394028, 0.24367785, 0.00970223, 0.96979969]  # x,y,z,w


def main():
    if len(sys.argv) < 2:
        in_path = "/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd"
        out_path = "/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd"
    elif len(sys.argv) == 2:
        in_path = sys.argv[1]
        out_path = in_path.replace(".pcd", "_leveled.pcd")
    else:
        in_path = sys.argv[1]
        out_path = sys.argv[2]

    # Load PCD
    print(f"Loading {in_path}...")
    pcd = o3d.io.read_point_cloud(in_path)
    print(f"  Points: {len(pcd.points)}")

    # The extrinsics define base_link -> livox_frame.
    # camera_init frame ≈ body frame ≈ livox_frame (orientation-wise).
    # To level: apply R_inv = R^T, rotating from livox_frame orientation
    # to base_link orientation (gravity-aligned).
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(QUATERNION).as_matrix()  # livox -> base
    R_inv = R.T  # base -> livox, but we apply to points: p_base = R @ p_livox

    pts = np.asarray(pcd.points)
    pts_leveled = (R @ pts.T).T  # rotate each point
    pcd.points = o3d.utility.Vector3dVector(pts_leveled)

    print(f"Saving {out_path}...")
    o3d.io.write_point_cloud(out_path, pcd)
    print("Done.")


if __name__ == "__main__":
    main()
