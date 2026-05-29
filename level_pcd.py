#!/usr/bin/env python3
"""
Rotate a PCD map to level it using the base_link -> livox_frame extrinsics.
This compensates for the tilted MID360 mounting so that Z-slicing in pcd2pgm
produces a level 2D occupancy map.

The normal 2D map workflow no longer needs this script: pcd2pgm can now perform
the same leveling and ground-zero alignment from pcd2pgm_m20.yaml. Keep this
script only as an offline inspection/export helper for 3D PCD files.

Supported usage:
    python3 level_pcd.py /path/to/in.pcd /path/to/out.pcd
    python3 level_pcd.py --input /path/to/in.pcd --output /path/to/out.pcd

By default the script applies two steps:
1. Coarse leveling using the known base_link -> livox_frame extrinsics.
2. Fine leveling by fitting the dominant near-horizontal ground plane and
   applying a small corrective rotation so the plane normal aligns with +Z.
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
    parser.add_argument(
        "--no-ground-refine",
        action="store_true",
        help="Skip fine leveling from the fitted ground plane and only apply the fixed extrinsics.",
    )
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

    return in_path, out_path, not args.no_ground_refine


def fit_ground_plane(o3d, pcd):
    # Work on a downsampled copy to keep runtime reasonable while preserving the
    # dominant floor plane structure.
    sample = pcd.voxel_down_sample(voxel_size=0.08)
    if len(sample.points) < 1000:
        sample = pcd
    plane_model, inliers = sample.segment_plane(
        distance_threshold=0.03,
        ransac_n=3,
        num_iterations=3000,
    )
    plane_model = np.asarray(plane_model, dtype=float)
    plane_model /= np.linalg.norm(plane_model[:3])
    if plane_model[2] < 0.0:
        plane_model = -plane_model
    normal = plane_model[:3]
    return normal, plane_model, inliers


def rotation_aligning_vectors(src_vec, dst_vec, Rotation):
    src = np.asarray(src_vec, dtype=float)
    dst = np.asarray(dst_vec, dtype=float)
    src /= np.linalg.norm(src)
    dst /= np.linalg.norm(dst)
    cross = np.cross(src, dst)
    cross_norm = np.linalg.norm(cross)
    dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))

    if cross_norm < 1e-12:
        if dot > 0.0:
            return Rotation.identity()
        ortho = np.array([1.0, 0.0, 0.0])
        if abs(src[0]) > 0.9:
            ortho = np.array([0.0, 1.0, 0.0])
        axis = np.cross(src, ortho)
        axis /= np.linalg.norm(axis)
        return Rotation.from_rotvec(axis * np.pi)

    axis = cross / cross_norm
    angle = np.arccos(dot)
    return Rotation.from_rotvec(axis * angle)


def main():
    in_path, out_path, refine_ground = parse_args()

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

    from scipy.spatial.transform import Rotation

    # The extrinsics define base_link -> livox_frame.
    # camera_init frame ≈ body frame ≈ livox_frame (orientation-wise).
    # To level, rotate points from livox-frame orientation to base-link orientation.
    R = Rotation.from_quat(QUATERNION).as_matrix()

    pts = np.asarray(pcd.points)
    pts_leveled = (R @ pts.T).T  # rotate each point
    pcd.points = o3d.utility.Vector3dVector(pts_leveled)

    coarse_normal, coarse_plane, coarse_inliers = fit_ground_plane(o3d, pcd)
    coarse_tilt_deg = np.degrees(np.arccos(np.clip(abs(coarse_normal[2]), -1.0, 1.0)))
    print(
        "After coarse leveling:"
        f" ground tilt={coarse_tilt_deg:.3f} deg"
        f" normal=({coarse_normal[0]:+.6f}, {coarse_normal[1]:+.6f}, {coarse_normal[2]:+.6f})"
        f" inliers={len(coarse_inliers)}"
    )

    if refine_ground:
        if abs(coarse_normal[2]) < 0.95:
            print(
                "Skipping ground refinement because the fitted dominant plane is not near horizontal "
                f"(normal z={coarse_normal[2]:.4f})."
            )
        else:
            correction = rotation_aligning_vectors(coarse_normal, np.array([0.0, 0.0, 1.0]), Rotation)
            correction_matrix = correction.as_matrix()
            pts_refined = (correction_matrix @ np.asarray(pcd.points).T).T
            pcd.points = o3d.utility.Vector3dVector(pts_refined)

            refined_normal, refined_plane, refined_inliers = fit_ground_plane(o3d, pcd)
            refined_tilt_deg = np.degrees(np.arccos(np.clip(abs(refined_normal[2]), -1.0, 1.0)))
            corr_euler_deg = correction.as_euler("xyz", degrees=True)
            print(
                "Applied fine ground correction:"
                f" roll={corr_euler_deg[0]:+.3f} deg"
                f" pitch={corr_euler_deg[1]:+.3f} deg"
                f" yaw={corr_euler_deg[2]:+.3f} deg"
            )
            print(
                "After fine leveling:"
                f" ground tilt={refined_tilt_deg:.3f} deg"
                f" normal=({refined_normal[0]:+.6f}, {refined_normal[1]:+.6f}, {refined_normal[2]:+.6f})"
                f" inliers={len(refined_inliers)}"
            )
            coarse_plane = refined_plane

    print(f"Saving {out_path}...")
    if not o3d.io.write_point_cloud(out_path, pcd):
        print(f"Failed to write point cloud: {out_path}")
        raise SystemExit(1)
    print("Done.")


if __name__ == "__main__":
    main()
