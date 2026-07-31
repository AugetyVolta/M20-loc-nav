import os
import sys

import numpy as np


PLANNER_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "PCT_planner", "planner")
)
sys.path.insert(0, os.path.join(PLANNER_ROOT, "scripts"))

from planner_wrapper import TomogramPlanner


def make_planner():
    planner = TomogramPlanner.__new__(TomogramPlanner)
    planner.n_slice = 2
    planner.resolution = 1.0
    planner.center = np.array([0.0, 0.0], dtype=np.float64)
    planner.map_dim = [3, 3]
    planner.offset = np.array([1, 1], dtype=np.int32)
    planner.a_star_cost_threshold = 45.0
    planner.layer_match_height_tolerance = 0.5
    planner.path_ground_offset = 0.1
    planner.tomogram = np.ones((1, 2, 3, 3), dtype=np.float32)
    planner.layer_elev_grids = np.zeros((2, 3, 3), dtype=np.float32)
    planner.layer_elev_grids[1, :, :] = 3.0
    return planner


def test_invalid_layer_hint_falls_back_to_matching_surface():
    planner = make_planner()

    assert planner.resolve_layer(0.0, 0.0, 0.0, layer_hint=1) == 0


def test_layer_matching_does_not_clip_outside_map():
    planner = make_planner()

    assert planner.match_best_layer(10.0, 0.0, 0.0) is None
    info = planner.get_layer_cost_info(0, 10.0, 0.0)
    assert not info["in_bounds"]


def test_path_geometry_rejects_single_point_reversal():
    planner = make_planner()
    trajectory = np.array(
        [
            [0.0, 0.0, 0.1],
            [0.2, 0.0, 0.1],
            [0.1, 0.0, 0.1],
        ],
        dtype=np.float32,
    )

    metrics, issue = planner._path_geometry(
        trajectory,
        np.zeros(3, dtype=np.int32),
        expected_surface=np.full(3, 0.1, dtype=np.float32),
    )

    assert issue == "path_reversal"
    assert metrics["max_turn_deg"] > 150.0


def test_path_geometry_accepts_short_surface_following_path():
    planner = make_planner()
    trajectory = np.array(
        [
            [-0.2, 0.0, 0.1],
            [0.0, 0.0, 0.15],
            [0.2, 0.1, 0.2],
        ],
        dtype=np.float32,
    )

    metrics, issue = planner._path_geometry(
        trajectory,
        np.zeros(3, dtype=np.int32),
        expected_surface=np.array([0.1, 0.1, 0.1], dtype=np.float32),
    )

    assert issue is None
    assert np.isclose(metrics["max_surface_error"], 0.1)
