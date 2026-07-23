#!/usr/bin/env python3

import argparse
import pickle
from pathlib import Path

import numpy as np

from .pct_paths import default_pct_root, expand_path, tomogram_stem


def build_surface_points(
    layers_t,
    layers_g,
    resolution,
    center,
    slice_dh,
):
    """Convert tomogram cost/height layers into finite XYZI surface samples."""
    costs = np.asarray(layers_t, dtype=np.float32).copy()
    heights = np.asarray(layers_g, dtype=np.float32).copy()
    if costs.shape != heights.shape or heights.ndim != 3:
        raise ValueError(
            f"Expected matching [layers, x, y] arrays, got {costs.shape} and {heights.shape}"
        )

    dim_x = heights.shape[1]
    dim_y = heights.shape[2]
    grid_x, grid_y = np.meshgrid(
        np.arange(dim_x, dtype=np.float32),
        np.arange(dim_y, dtype=np.float32),
        indexing="ij",
    )
    center_xy = np.asarray(center, dtype=np.float32).reshape(-1)
    if center_xy.size < 2:
        raise ValueError(f"Tomogram center must contain x/y, got {center}")

    point_proto = np.empty((dim_x, dim_y, 4), dtype=np.float32)
    point_proto[..., 0] = (grid_x - 0.5 * dim_x) * float(resolution) + center_xy[0]
    point_proto[..., 1] = (grid_y - 0.5 * dim_y) * float(resolution) + center_xy[1]
    surface_layers = []

    def append_layer(height_grid, cost_grid):
        points = point_proto.copy()
        points[..., 2] = height_grid
        points[..., 3] = cost_grid
        valid = np.isfinite(points).all(axis=-1)
        if np.any(valid):
            surface_layers.append(points[valid])

    for layer_index in range(heights.shape[0] - 1):
        hidden = (heights[layer_index + 1] - heights[layer_index]) < float(slice_dh)
        heights[layer_index, hidden] = np.nan
        costs[layer_index + 1, hidden] = np.minimum(
            costs[layer_index, hidden],
            costs[layer_index + 1, hidden],
        )
        append_layer(heights[layer_index], costs[layer_index])
    append_layer(heights[-1], costs[-1])

    if not surface_layers:
        return np.empty((0, 4), dtype=np.float32)
    return np.ascontiguousarray(np.concatenate(surface_layers, axis=0), dtype="<f4")


def write_binary_xyzi_pcd(
    output_path,
    points,
    resolution=None,
    frame_id="map",
):
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    xyzi = np.ascontiguousarray(points, dtype="<f4")
    if xyzi.ndim != 2 or xyzi.shape[1] != 4:
        raise ValueError(f"Expected an Nx4 XYZI array, got {xyzi.shape}")

    metadata = ""
    if resolution is not None:
        metadata += f"# TOMOGRAM_RESOLUTION {float(resolution):.9g}\n"
    if frame_id:
        metadata += f"# TOMOGRAM_FRAME {frame_id}\n"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        f"{metadata}"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {xyzi.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyzi.shape[0]}\n"
        "DATA binary\n"
    ).encode("ascii")
    with output.open("wb") as handle:
        handle.write(header)
        handle.write(xyzi.tobytes(order="C"))
    return output


def export_surface_from_pickle(pickle_path, output_path=None, frame_id="map"):
    source = Path(pickle_path)
    with source.open("rb") as handle:
        tomogram = pickle.load(handle)

    data = np.asarray(tomogram["data"], dtype=np.float32)
    points = build_surface_points(
        data[0],
        data[3],
        float(tomogram["resolution"]),
        tomogram["center"],
        float(tomogram["slice_dh"]),
    )
    if points.shape[0] == 0:
        raise RuntimeError(f"Tomogram has no finite surface points: {source}")

    destination = (
        Path(output_path)
        if output_path
        else source.with_suffix("").with_suffix(".surface.pcd")
    )
    return (
        write_binary_xyzi_pcd(
            destination,
            points,
            resolution=float(tomogram["resolution"]),
            frame_id=frame_id,
        ),
        points.shape[0],
    )


def _resolve_pickle(value, pct_root):
    candidate = expand_path(value)
    if candidate.is_file():
        return candidate
    return expand_path(pct_root) / "rsc/tomogram" / f"{tomogram_stem(value)}.pickle"


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Export a static XYZI surface PCD from a PCT tomogram pickle."
    )
    parser.add_argument("tomogram", help="Tomogram pickle path or tomogram stem")
    parser.add_argument("--pct-root", default=default_pct_root())
    parser.add_argument("--output", default="")
    parser.add_argument("--frame", default="map")
    parsed = parser.parse_args(args=args)

    pickle_path = _resolve_pickle(parsed.tomogram, parsed.pct_root)
    if not pickle_path.is_file():
        raise FileNotFoundError(f"Tomogram pickle does not exist: {pickle_path}")
    output_path, point_count = export_surface_from_pickle(
        pickle_path,
        parsed.output or None,
        frame_id=parsed.frame,
    )
    print(f"Exported {point_count} surface points: {output_path}")


if __name__ == "__main__":
    main()
