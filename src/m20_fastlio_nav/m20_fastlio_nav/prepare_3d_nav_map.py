#!/usr/bin/env python3
"""Prepare the 3D PCD map used by localization and external 3D planning."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import numpy as np


BASE_TO_LIVOX_QUAT_XYZW = (-0.00394028, 0.24367785, 0.00970223, 0.96979969)
DEFAULT_INPUTS = (
    "maps/fastlio/global_map.pcd",
    "maps/fastlio/m20_map_raw.pcd",
)
DEFAULT_OUTPUT = "maps/fastlio/m20_3d_map.pcd"


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def resolve_path(path: str | Path) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate
    return workspace_root() / candidate


def resolve_input(input_path: str | None) -> Path:
    if input_path:
        candidate = resolve_path(input_path)
        if not candidate.exists():
            raise FileNotFoundError(f"input PCD does not exist: {candidate}")
        return candidate

    for default in DEFAULT_INPUTS:
        candidate = resolve_path(default)
        if candidate.exists():
            return candidate
    searched = "\n  ".join(str(resolve_path(path)) for path in DEFAULT_INPUTS)
    raise FileNotFoundError(f"no default input PCD found; searched:\n  {searched}")


def quat_xyzw_to_matrix(quat: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm < 1.0e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_aligning_vectors(src_vec: np.ndarray, dst_vec: np.ndarray) -> np.ndarray:
    src = np.asarray(src_vec, dtype=np.float64)
    dst = np.asarray(dst_vec, dtype=np.float64)
    src /= np.linalg.norm(src)
    dst /= np.linalg.norm(dst)

    cross = np.cross(src, dst)
    cross_norm = float(np.linalg.norm(cross))
    dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))
    if cross_norm < 1.0e-12:
        if dot > 0.0:
            return np.eye(3)
        ortho = np.array([1.0, 0.0, 0.0])
        if abs(src[0]) > 0.9:
            ortho = np.array([0.0, 1.0, 0.0])
        axis = np.cross(src, ortho)
        axis /= np.linalg.norm(axis)
        return rotation_matrix_from_axis_angle(axis, np.pi)

    axis = cross / cross_norm
    angle = float(np.arccos(dot))
    return rotation_matrix_from_axis_angle(axis, angle)


def rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def apply_rotation(o3d, pcd, rotation: np.ndarray) -> None:
    points = np.asarray(pcd.points)
    pcd.points = o3d.utility.Vector3dVector((rotation @ points.T).T)


def _point_count(pcd) -> int:
    return len(pcd.points)


def filter_by_z_bounds(pcd, z_min: float | None, z_max: float | None):
    if z_min is None and z_max is None:
        return pcd, 0
    points = np.asarray(pcd.points)
    mask = np.isfinite(points).all(axis=1)
    if z_min is not None:
        mask &= points[:, 2] >= float(z_min)
    if z_max is not None:
        mask &= points[:, 2] <= float(z_max)
    keep = np.flatnonzero(mask)
    removed = points.shape[0] - keep.shape[0]
    return pcd.select_by_index(keep.tolist()), removed


def filter_outlier_points(pcd, args):
    before_all = _point_count(pcd)
    pcd, removed_z = filter_by_z_bounds(pcd, args.filter_z_min, args.filter_z_max)
    if removed_z:
        print(
            "Filtered by z bounds: "
            f"removed={removed_z}, remaining={_point_count(pcd)}"
        )

    if not args.filter_outliers:
        return pcd

    before_stat = _point_count(pcd)
    if before_stat >= max(args.statistical_nb_neighbors, 3):
        pcd, inliers = pcd.remove_statistical_outlier(
            nb_neighbors=args.statistical_nb_neighbors,
            std_ratio=args.statistical_std_ratio,
        )
        print(
            "Statistical outlier filter: "
            f"nb_neighbors={args.statistical_nb_neighbors}, "
            f"std_ratio={args.statistical_std_ratio:.2f}, "
            f"removed={before_stat - len(inliers)}, remaining={_point_count(pcd)}"
        )
    else:
        print("Skipped statistical outlier filter because the cloud is too small")

    if args.radius_outlier_filter:
        before_radius = _point_count(pcd)
        if before_radius >= max(args.radius_nb_points, 3):
            pcd, inliers = pcd.remove_radius_outlier(
                nb_points=args.radius_nb_points,
                radius=args.radius_radius,
            )
            print(
                "Radius outlier filter: "
                f"radius={args.radius_radius:.2f}m, "
                f"nb_points={args.radius_nb_points}, "
                f"removed={before_radius - len(inliers)}, remaining={_point_count(pcd)}"
            )
        else:
            print("Skipped radius outlier filter because the cloud is too small")

    removed_all = before_all - _point_count(pcd)
    print(
        "Point cloud filtering summary: "
        f"input={before_all}, output={_point_count(pcd)}, removed={removed_all} "
        f"({100.0 * removed_all / max(before_all, 1):.2f}%)"
    )
    return pcd


def _normalize_plane(plane_model: Iterable[float]) -> tuple[np.ndarray, np.ndarray, float]:
    plane = np.asarray(plane_model, dtype=np.float64)
    normal_norm = float(np.linalg.norm(plane[:3]))
    if normal_norm < 1.0e-12:
        raise RuntimeError("ground plane fit returned a zero normal")
    plane /= normal_norm
    if plane[2] < 0.0:
        plane = -plane
    normal = plane[:3]
    tilt_deg = float(np.degrees(np.arccos(np.clip(abs(normal[2]), -1.0, 1.0))))
    return plane, normal, tilt_deg


def fit_ground_plane(
    o3d,
    pcd,
    voxel_size: float,
    distance_threshold: float,
    iterations: int,
    max_tilt_deg: float | None = None,
    max_attempts: int = 8,
):
    sample = pcd.voxel_down_sample(voxel_size=voxel_size) if voxel_size > 0.0 else pcd
    if len(sample.points) < 1000:
        sample = pcd

    remaining = sample
    best_plane = None
    best_normal = None
    best_inliers = None
    best_tilt = np.inf
    attempts = max(1, int(max_attempts))

    for attempt in range(attempts):
        if len(remaining.points) < 1000:
            break
        plane_model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=iterations,
        )
        plane, normal, tilt_deg = _normalize_plane(plane_model)

        if tilt_deg < best_tilt:
            best_plane = plane
            best_normal = normal
            best_inliers = inliers
            best_tilt = tilt_deg

        if max_tilt_deg is None or tilt_deg <= max_tilt_deg:
            if attempt > 0:
                print(
                    "Selected horizontal ground candidate: "
                    f"attempt={attempt + 1}, tilt={tilt_deg:.3f}deg, inliers={len(inliers)}"
                )
            return normal, plane, inliers, tilt_deg

        print(
            "Rejected non-horizontal plane candidate: "
            f"attempt={attempt + 1}, tilt={tilt_deg:.3f}deg, inliers={len(inliers)}"
        )
        remaining = remaining.select_by_index(inliers, invert=True)

    if best_plane is None or best_normal is None or best_inliers is None:
        raise RuntimeError("ground plane fit found no usable plane")
    raise RuntimeError(
        "ground plane fit found no sufficiently horizontal plane: "
        f"best_tilt={best_tilt:.3f}deg > {float(max_tilt_deg):.3f}deg"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Create the ground-aligned 3D PCD used by Open3D localization and external 3D planning. "
            "The map remains 3D; this tool does not project it into a 2D occupancy map."
        )
    )
    parser.add_argument("input_pos", nargs="?", help="Input PCD. Defaults to global_map.pcd, then raw Fast-LIO map.")
    parser.add_argument("--input", dest="input_opt", help="Input PCD path.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output aligned 3D PCD path.")
    parser.add_argument("--skip-align", action="store_true", help="Only filter and rewrite/copy the PCD.")
    parser.add_argument(
        "--no-fixed-extrinsic",
        action="store_false",
        dest="fixed_extrinsic",
        help="Do not apply the known base_link->livox_frame coarse leveling rotation.",
    )
    parser.add_argument(
        "--no-ground-refine",
        action="store_false",
        dest="ground_refine",
        help="Skip RANSAC ground-plane refinement.",
    )
    parser.add_argument(
        "--ground-zero",
        action="store_true",
        dest="ground_zero",
        help="Translate the fitted dominant ground plane to z=0. Keep disabled for normal 3D localization maps.",
    )
    parser.add_argument("--voxel-size", type=float, default=0.08)
    parser.add_argument("--ground-distance-threshold", type=float, default=0.03)
    parser.add_argument("--ground-ransac-iterations", type=int, default=3000)
    parser.add_argument(
        "--ground-plane-max-tilt-deg",
        type=float,
        default=25.0,
        help="Reject RANSAC planes whose normal is too far from vertical; prevents walls from being used as ground.",
    )
    parser.add_argument(
        "--ground-plane-max-attempts",
        type=int,
        default=8,
        help="Number of dominant plane candidates to try before giving up on horizontal ground fitting.",
    )
    parser.add_argument("--max-ground-correction-deg", type=float, default=8.0)
    parser.add_argument(
        "--no-filter-outliers",
        action="store_false",
        dest="filter_outliers",
        help="Disable statistical outlier removal before writing the 3D navigation map.",
    )
    parser.add_argument(
        "--statistical-nb-neighbors",
        type=int,
        default=24,
        help="Neighbor count for Open3D statistical outlier removal.",
    )
    parser.add_argument(
        "--statistical-std-ratio",
        type=float,
        default=2.5,
        help="Standard deviation ratio for Open3D statistical outlier removal.",
    )
    parser.add_argument(
        "--radius-outlier-filter",
        action="store_true",
        help="Also remove points that have too few neighbors inside a fixed radius.",
    )
    parser.add_argument("--radius-radius", type=float, default=0.35)
    parser.add_argument("--radius-nb-points", type=int, default=4)
    parser.add_argument("--filter-z-min", type=float, default=None)
    parser.add_argument("--filter-z-max", type=float, default=None)
    parser.set_defaults(
        fixed_extrinsic=True,
        ground_refine=True,
        ground_zero=False,
        filter_outliers=True,
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    input_path = resolve_input(args.input_opt or args.input_pos)
    output_path = resolve_path(args.output)

    try:
        import open3d as o3d
    except ImportError as exc:
        raise SystemExit("open3d is required. Activate the Python environment that has open3d installed.") from exc

    pcd = o3d.io.read_point_cloud(str(input_path))
    if len(pcd.points) == 0:
        raise SystemExit(f"input PCD is empty or unreadable: {input_path}")

    print(f"Input: {input_path}")
    print(f"  points={len(pcd.points)}")

    if not args.skip_align:
        if args.fixed_extrinsic:
            rotation = quat_xyzw_to_matrix(BASE_TO_LIVOX_QUAT_XYZW)
            apply_rotation(o3d, pcd, rotation)
            print("Applied coarse base_link->livox_frame leveling rotation")

        normal, plane, inliers, tilt_deg = fit_ground_plane(
            o3d,
            pcd,
            args.voxel_size,
            args.ground_distance_threshold,
            args.ground_ransac_iterations,
            max_tilt_deg=args.ground_plane_max_tilt_deg,
            max_attempts=args.ground_plane_max_attempts,
        )
        print(
            "Ground after coarse step: "
            f"tilt={tilt_deg:.3f}deg, "
            f"normal=({normal[0]:+.5f}, {normal[1]:+.5f}, {normal[2]:+.5f}), "
            f"inliers={len(inliers)}"
        )

        if args.ground_refine:
            correction = rotation_aligning_vectors(normal, np.array([0.0, 0.0, 1.0], dtype=np.float64))
            correction_angle = float(np.degrees(np.arccos(np.clip((np.trace(correction) - 1.0) * 0.5, -1.0, 1.0))))
            if correction_angle <= args.max_ground_correction_deg:
                apply_rotation(o3d, pcd, correction)
                print(f"Applied fine ground correction: {correction_angle:.3f}deg")
                plane = np.asarray([0.0, 0.0, 1.0, plane[3]], dtype=np.float64)
            else:
                print(
                    "Skipped fine ground correction because it is too large: "
                    f"{correction_angle:.3f}deg > {args.max_ground_correction_deg:.3f}deg"
                )

        if args.ground_zero:
            if abs(float(plane[2])) < 0.5:
                raise RuntimeError(
                    "refusing --ground-zero because the selected ground plane is near vertical: "
                    f"normal=({plane[0]:+.5f}, {plane[1]:+.5f}, {plane[2]:+.5f})"
                )
            ground_z = float(-plane[3] / plane[2])
            if abs(ground_z) > 20.0:
                raise RuntimeError(
                    "refusing --ground-zero because the required z shift is implausibly large: "
                    f"ground_z={ground_z:+.3f}m"
                )
            pcd.translate((0.0, 0.0, -ground_z))
            print(
                "Shifted ground plane to z=0: "
                f"ground_z={ground_z:+.3f}m, tilt={tilt_deg:.3f}deg, inliers={len(inliers)}"
            )

    pcd = filter_outlier_points(pcd, args)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(output_path), pcd, write_ascii=False, compressed=False):
        raise SystemExit(f"failed to write output PCD: {output_path}")
    print(f"3D navigation map: {output_path}")


if __name__ == "__main__":
    main()
