import numpy as np

from m20_fastlio_nav.navigation_mode_manager import NavigationModeManager
from m20_fastlio_nav.terrain_state_estimator import TerrainStateEstimator
from m20_navigation_msgs.msg import NavigationMode, TerrainState


def _terrain_estimator():
    estimator = TerrainStateEstimator.__new__(TerrainStateEstimator)
    estimator.min_path_length = 0.6
    estimator.slope_window_max = 1.5
    estimator.stair_up_slope = 0.14
    estimator.stair_up_dz = 0.28
    estimator.stair_down_slope = 0.18
    estimator.stair_down_dz = 0.35
    estimator.platform_merge_distance = 3.0
    estimator.projection_max_distance = 2.0
    estimator.projection_backtrack_points = 2
    return estimator


def _path(parts):
    points = []
    x = 0.0
    z = 0.0
    for length, height_change in parts:
        steps = max(1, int(round(length / 0.2)))
        for index in range(steps):
            ratio = index / steps
            points.append(
                [
                    x + length * ratio,
                    0.0,
                    z + height_change * ratio,
                ]
            )
        x += length
        z += height_change
    points.append([x, 0.0, z])

    path = np.asarray(points, dtype=np.float64)
    arc = np.zeros((len(path),), dtype=np.float64)
    arc[1:] = np.cumsum(
        np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    )
    return path, arc


def _mode_manager():
    manager = NavigationModeManager.__new__(NavigationModeManager)
    manager.pct_dynamic_guard_distance = 5.0
    manager.costmap_stair_pre_distance = 3.0
    manager.scan_stair_pre_distance = 3.0
    manager.heading_guard_pre_distance = 3.0
    manager.gait_switch_pre_distance = 3.0
    return manager


def _terrain_state(current, upcoming, distance):
    state = TerrainState()
    state.current_mode = current
    state.upcoming_mode = upcoming
    state.distance_to_entry = distance
    state.valid = True
    state.route_id = 1
    return state


def test_stair_zones_merge_same_direction_flights_across_platform():
    estimator = _terrain_estimator()
    path, arc = _path(
        [
            (2.0, 0.0),
            (2.0, 1.2),
            (2.8, 0.0),
            (2.0, 1.2),
            (2.0, 0.0),
        ]
    )

    zones = estimator._extract_stair_zones(path, arc)

    assert len(zones) == 1
    assert zones[0]["mode"] == TerrainState.STAIR_UP
    assert abs(zones[0]["start"] - 2.0) < 0.21
    assert abs(zones[0]["end"] - 8.8) < 0.21


def test_stair_zones_keep_opposite_directions_separate():
    estimator = _terrain_estimator()
    path, arc = _path(
        [
            (1.0, 0.0),
            (2.0, 1.2),
            (3.0, 0.0),
            (2.0, -1.2),
            (1.0, 0.0),
        ]
    )

    zones = estimator._extract_stair_zones(path, arc)

    assert [zone["mode"] for zone in zones] == [
        TerrainState.STAIR_UP,
        TerrainState.STAIR_DOWN,
    ]


def test_route_projection_reacquires_an_earlier_point_after_reversing():
    estimator = _terrain_estimator()
    estimator.path = np.asarray(
        [[float(index), 0.0, 0.0] for index in range(11)],
        dtype=np.float64,
    )
    estimator.progress_index = 9

    nearest, distance = estimator._nearest_progress_index(
        np.array([2.1, 0.0, 0.0], dtype=np.float64)
    )

    assert nearest == 2
    assert distance < 0.11
    assert estimator.progress_index == 2


def test_route_projection_stays_invalid_when_robot_is_off_route():
    estimator = _terrain_estimator()
    estimator.path = np.asarray(
        [[float(index), 0.0, 0.0] for index in range(11)],
        dtype=np.float64,
    )
    estimator.progress_index = 9

    nearest, distance = estimator._nearest_progress_index(
        np.array([2.0, 0.0, 5.0], dtype=np.float64)
    )

    assert nearest == 9
    assert distance > estimator.projection_max_distance
    assert estimator.progress_index == 9


def test_navigation_policies_use_independent_distances():
    manager = _mode_manager()

    far = manager._mode_from_terrain(
        _terrain_state(TerrainState.FLAT, TerrainState.STAIR_UP, 6.0)
    )
    dynamic_guard = manager._mode_from_terrain(
        _terrain_state(TerrainState.FLAT, TerrainState.STAIR_UP, 4.0)
    )
    before_stair_profile = manager._mode_from_terrain(
        _terrain_state(TerrainState.FLAT, TerrainState.STAIR_UP, 3.5)
    )
    near = manager._mode_from_terrain(
        _terrain_state(TerrainState.FLAT, TerrainState.STAIR_UP, 2.8)
    )

    assert far.pct_dynamic_enabled
    assert far.costmap_profile == NavigationMode.PROFILE_FLAT
    assert not dynamic_guard.pct_dynamic_enabled
    assert dynamic_guard.gait_terrain == NavigationMode.TERRAIN_FLAT
    assert before_stair_profile.costmap_profile == NavigationMode.PROFILE_FLAT
    assert before_stair_profile.gait_terrain == NavigationMode.TERRAIN_FLAT
    assert near.costmap_profile == NavigationMode.PROFILE_STAIR
    assert near.scan_profile == NavigationMode.SCAN_TRAVERSABILITY
    assert near.heading_guard_enabled
    assert near.gait_terrain == NavigationMode.TERRAIN_STAIR_UP
