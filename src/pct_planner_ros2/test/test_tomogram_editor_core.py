import pickle

import numpy as np

from pct_planner_ros2.tomogram_editor_core import (
    WallSegment,
    apply_virtual_walls,
    save_edited_tomogram,
)


def make_tomogram():
    data = np.zeros((5, 2, 21, 21), dtype=np.float32)
    data[3, 0] = 0.0
    data[3, 1] = 3.0
    data[4] = 10.0
    return {
        "data": data,
        "resolution": 0.1,
        "center": np.array([0.0, 0.0]),
        "slice_h0": 0.0,
        "slice_dh": 1.0,
    }


def test_virtual_wall_only_modifies_matching_height_layer():
    edited, stats = apply_virtual_walls(
        make_tomogram(),
        [WallSegment.from_points((-0.5, 0.0, 0.0), (0.5, 0.0, 0.0))],
        wall_width=0.2,
        inflation_radius=0.3,
        barrier_cost=50.0,
        layer_height_tolerance=0.5,
    )

    costs = edited["data"][0]
    assert costs[0, 10, 10] == 50.0
    assert 0.0 < costs[0, 10, 12] < 50.0
    assert np.count_nonzero(costs[1]) == 0
    assert stats["hard_cells"] > 0


def test_virtual_wall_recomputes_tomography_gradients():
    edited, _ = apply_virtual_walls(
        make_tomogram(),
        [WallSegment.from_points((-0.5, 0.0, 0.0), (0.5, 0.0, 0.0))],
        wall_width=0.2,
        inflation_radius=0.3,
    )
    data = edited["data"]

    np.testing.assert_allclose(
        data[1, :, 1:-1, :],
        data[0, :, 2:, :] - data[0, :, :-2, :],
    )
    np.testing.assert_allclose(
        data[2, :, :, 1:-1],
        data[0, :, :, 2:] - data[0, :, :, :-2],
    )


def test_save_keeps_source_and_writes_float16_copy(tmp_path):
    source = tmp_path / "map.pickle"
    output = tmp_path / "map_edited.pickle"
    source_data = make_tomogram()
    with source.open("wb") as handle:
        pickle.dump(source_data, handle)

    edited, _ = apply_virtual_walls(
        source_data,
        [WallSegment.from_points((-0.5, 0.0, 0.0), (0.5, 0.0, 0.0))],
    )
    destination, record = save_edited_tomogram(
        output,
        edited,
        source_path=source,
        segments=[WallSegment.from_points((-0.5, 0.0, 0.0), (0.5, 0.0, 0.0))],
        parameters={"wall_width": 0.3},
    )

    assert source.exists()
    assert destination.exists()
    assert record.exists()
    with destination.open("rb") as handle:
        saved = pickle.load(handle)
    assert saved["data"].dtype == np.float16
