import copy
import json
import os
import pickle
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class WallSegment:
    start: tuple
    end: tuple

    @classmethod
    def from_points(cls, start, end):
        return cls(
            tuple(float(value) for value in start[:3]),
            tuple(float(value) for value in end[:3]),
        )


def _validate_tomogram(data_dict):
    data = np.asarray(data_dict["data"])
    if data.ndim != 4 or data.shape[0] < 5:
        raise ValueError(
            "Tomogram data must have shape [at least 5, layers, x, y], "
            f"got {data.shape}"
        )
    center = np.asarray(data_dict["center"], dtype=np.float64).reshape(-1)
    if center.size < 2:
        raise ValueError(f"Tomogram center must contain x/y, got {center}")
    resolution = float(data_dict["resolution"])
    if resolution <= 0.0:
        raise ValueError(f"Tomogram resolution must be positive, got {resolution}")
    return data, center[:2], resolution


def recompute_cost_gradients(data):
    """Rebuild PCT cost gradients with the tomography core's convention."""
    data[1].fill(0.0)
    data[2].fill(0.0)
    data[1, :, 1:-1, :] = data[0, :, 2:, :] - data[0, :, :-2, :]
    data[2, :, :, 1:-1] = data[0, :, :, 2:] - data[0, :, :, :-2]


def _segment_grid_bounds(start, end, radius, center, resolution, dimensions):
    offset = np.asarray(dimensions, dtype=np.int64) // 2
    lower_xy = np.minimum(start[:2], end[:2]) - radius
    upper_xy = np.maximum(start[:2], end[:2]) + radius
    lower = np.floor((lower_xy - center) / resolution).astype(np.int64) + offset
    upper = np.ceil((upper_xy - center) / resolution).astype(np.int64) + offset
    lower = np.maximum(lower, 0)
    upper = np.minimum(upper, np.asarray(dimensions, dtype=np.int64) - 1)
    return lower, upper, offset


def snap_point_to_surface(data_dict, point, search_radius=0.5):
    """Snap a clicked XYZ point to the nearest tomogram ground surface at that XY."""
    data, center, resolution = _validate_tomogram(data_dict)
    point = np.asarray(point, dtype=np.float64).reshape(3)
    dimensions = np.asarray(data.shape[2:], dtype=np.int64)
    offset = dimensions // 2
    center_index = np.rint((point[:2] - center) / resolution).astype(np.int64) + offset
    radius_cells = max(0, int(np.ceil(float(search_radius) / resolution)))
    lower = np.maximum(center_index - radius_cells, 0)
    upper = np.minimum(center_index + radius_cells, dimensions - 1)
    if np.any(lower > upper):
        return point.copy(), False

    heights = data[3, :, lower[0]:upper[0] + 1, lower[1]:upper[1] + 1]
    valid = np.isfinite(heights)
    if not np.any(valid):
        return point.copy(), False

    layer_indices, local_x, local_y = np.nonzero(valid)
    candidate_heights = heights[layer_indices, local_x, local_y]
    grid_x = local_x + lower[0]
    grid_y = local_y + lower[1]
    world_x = (grid_x - offset[0]) * resolution + center[0]
    world_y = (grid_y - offset[1]) * resolution + center[1]
    xy_distance = np.hypot(world_x - point[0], world_y - point[1])
    score = np.abs(candidate_heights - point[2]) + 0.25 * xy_distance
    best = int(np.argmin(score))
    snapped = point.copy()
    snapped[2] = float(candidate_heights[best])
    return snapped, True


def apply_virtual_walls(
    source_data_dict,
    segments,
    *,
    wall_width=0.30,
    inflation_radius=0.50,
    cost_scaling_factor=5.0,
    barrier_cost=50.0,
    layer_height_tolerance=0.75,
):
    """Return a tomogram copy with height-aware virtual wall costs applied."""
    if wall_width <= 0.0:
        raise ValueError("wall_width must be positive")
    if inflation_radius < 0.0:
        raise ValueError("inflation_radius cannot be negative")
    if cost_scaling_factor < 0.0:
        raise ValueError("cost_scaling_factor cannot be negative")
    if layer_height_tolerance <= 0.0:
        raise ValueError("layer_height_tolerance must be positive")

    source_data, center, resolution = _validate_tomogram(source_data_dict)
    edited = {
        key: copy.deepcopy(value)
        for key, value in source_data_dict.items()
        if key != "data"
    }
    data = source_data.astype(np.float32, copy=True)
    costs = data[0]
    heights = data[3]
    dimensions = costs.shape[1:]
    hard_radius = 0.5 * float(wall_width)
    total_radius = hard_radius + float(inflation_radius)
    matched_cells = 0
    hard_cells = 0

    for segment in segments:
        start = np.asarray(segment.start, dtype=np.float64)
        end = np.asarray(segment.end, dtype=np.float64)
        direction_xy = end[:2] - start[:2]
        length_squared = float(np.dot(direction_xy, direction_xy))
        if length_squared < 1e-8:
            continue

        lower, upper, offset = _segment_grid_bounds(
            start,
            end,
            total_radius,
            center,
            resolution,
            dimensions,
        )
        if np.any(lower > upper):
            continue

        x_indices = np.arange(lower[0], upper[0] + 1, dtype=np.int64)
        y_indices = np.arange(lower[1], upper[1] + 1, dtype=np.int64)
        grid_x, grid_y = np.meshgrid(x_indices, y_indices, indexing="ij")
        world_x = (grid_x - offset[0]) * resolution + center[0]
        world_y = (grid_y - offset[1]) * resolution + center[1]
        relative_x = world_x - start[0]
        relative_y = world_y - start[1]
        projection = np.clip(
            (relative_x * direction_xy[0] + relative_y * direction_xy[1])
            / length_squared,
            0.0,
            1.0,
        )
        closest_x = start[0] + projection * direction_xy[0]
        closest_y = start[1] + projection * direction_xy[1]
        distance = np.hypot(world_x - closest_x, world_y - closest_y)
        affected_xy = distance <= total_radius
        if not np.any(affected_xy):
            continue

        expected_height = start[2] + projection * (end[2] - start[2])
        outside_hard = np.maximum(distance - hard_radius, 0.0)
        contribution = float(barrier_cost) * np.exp(
            -float(cost_scaling_factor) * outside_hard
        )
        contribution[distance <= hard_radius] = float(barrier_cost)
        contribution[~affected_xy] = 0.0

        x_slice = slice(lower[0], upper[0] + 1)
        y_slice = slice(lower[1], upper[1] + 1)
        for layer in range(costs.shape[0]):
            local_heights = heights[layer, x_slice, y_slice]
            layer_mask = (
                affected_xy
                & np.isfinite(local_heights)
                & (np.abs(local_heights - expected_height) <= layer_height_tolerance)
            )
            if not np.any(layer_mask):
                continue
            local_costs = costs[layer, x_slice, y_slice]
            local_costs[layer_mask] = np.maximum(
                local_costs[layer_mask], contribution[layer_mask]
            )
            matched_cells += int(np.count_nonzero(layer_mask))
            hard_cells += int(np.count_nonzero(layer_mask & (distance <= hard_radius)))

    recompute_cost_gradients(data)
    edited["data"] = data
    return edited, {"matched_cells": matched_cells, "hard_cells": hard_cells}


def save_edited_tomogram(
    output_path,
    data_dict,
    *,
    source_path,
    segments,
    parameters,
    frame_id="map",
):
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    serialized = {
        key: copy.deepcopy(value) for key, value in data_dict.items() if key != "data"
    }
    serialized["data"] = np.asarray(data_dict["data"], dtype=np.float16)

    temporary = destination.with_name(f".{destination.name}.tmp")
    with temporary.open("wb") as handle:
        pickle.dump(serialized, handle, protocol=pickle.HIGHEST_PROTOCOL)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, destination)

    edit_record = {
        "format": "pct_tomogram_virtual_walls/v1",
        "source_tomogram": str(Path(source_path).resolve()),
        "output_tomogram": str(destination.resolve()),
        "frame_id": str(frame_id),
        "parameters": {key: float(value) for key, value in parameters.items()},
        "walls": [asdict(segment) for segment in segments],
    }
    record_path = destination.with_suffix(".edits.yaml")
    with record_path.open("w", encoding="utf-8") as handle:
        # JSON is valid YAML and avoids adding a runtime parser dependency.
        json.dump(edit_record, handle, indent=2, ensure_ascii=True)
        handle.write("\n")
    return destination, record_path
