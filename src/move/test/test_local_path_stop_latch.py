from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty

from move.priest_rl_publisher_nav_cmd import RLLocalPlannerNodeROS2


def _path_with_pose():
    path = Path()
    path.poses.append(PoseStamped())
    return path


def _path_to(x, y, z):
    path = _path_with_pose()
    path.poses[-1].pose.position.x = float(x)
    path.poses[-1].pose.position.y = float(y)
    path.poses[-1].pose.position.z = float(z)
    return path


def _plan_active(harness):
    return RLLocalPlannerNodeROS2._global_plan_active(harness)


def test_empty_global_path_clears_subgoal_and_local_path():
    cleared = []
    harness = SimpleNamespace(
        latest_global_plan=_path_with_pose(),
        subgoal_position=(1.0, 0.0),
        _global_plan_generation=4,
        _navigation_cancelled=False,
        _awaiting_goal_plan=False,
        _publish_empty_local_paths=lambda: cleared.append(True),
    )

    RLLocalPlannerNodeROS2._on_global_plan(harness, Path())

    assert harness._global_plan_generation == 5
    assert harness.latest_global_plan.poses == []
    assert harness.subgoal_position is None
    assert cleared == [True]


def test_queued_subgoal_cannot_restart_after_global_path_is_cleared():
    harness = SimpleNamespace(
        latest_global_plan=Path(),
        subgoal_position=None,
        _navigation_cancelled=False,
    )
    harness._global_plan_active = lambda: _plan_active(harness)
    subgoal = PoseStamped()
    subgoal.pose.position.x = 2.0
    subgoal.pose.position.y = 0.5

    RLLocalPlannerNodeROS2._on_subgoal(harness, subgoal)

    assert harness.subgoal_position is None


def test_in_flight_result_is_discarded_when_global_path_is_cancelled():
    cleared = []
    published = []
    harness = SimpleNamespace(
        cfg=SimpleNamespace(use_scan=False),
        latest_odom=object(),
        latest_scan=None,
        latest_global_plan=_path_with_pose(),
        _global_plan_generation=8,
        _navigation_cancelled=False,
        pub_local_path_rl=None,
        pub_local_path=SimpleNamespace(publish=lambda msg: published.append(msg)),
        get_logger=lambda: SimpleNamespace(debug=lambda _msg: None, error=lambda _msg: None),
        _publish_empty_local_paths=lambda: cleared.append(True),
    )
    harness._global_plan_active = lambda: _plan_active(harness)

    def compute_then_cancel():
        harness.latest_global_plan = Path()
        harness._global_plan_generation += 1
        return _path_with_pose(), _path_with_pose()

    harness._compute_local_path_and_priest = compute_then_cancel

    RLLocalPlannerNodeROS2._on_timer(harness)

    assert cleared == [True]
    assert published == []


def test_cancel_latches_local_planner_stopped_until_a_new_goal():
    cleared = []
    harness = SimpleNamespace(
        latest_global_plan=_path_with_pose(),
        subgoal_position=(1.0, 0.0),
        _global_plan_generation=2,
        _navigation_cancelled=False,
        _navigation_goal_position=None,
        _awaiting_goal_plan=False,
        _publish_empty_local_paths=lambda: cleared.append(True),
    )
    harness._global_plan_active = lambda: _plan_active(harness)
    harness._navigation_goal_changed = lambda msg: (
        RLLocalPlannerNodeROS2._navigation_goal_changed(harness, msg)
    )
    harness._global_plan_matches_navigation_goal = lambda path: (
        RLLocalPlannerNodeROS2._global_plan_matches_navigation_goal(harness, path)
    )

    RLLocalPlannerNodeROS2._on_navigation_cancel(harness, Empty())
    RLLocalPlannerNodeROS2._on_global_plan(harness, _path_with_pose())

    assert harness._navigation_cancelled is True
    assert harness.latest_global_plan.poses == []
    assert harness.subgoal_position is None
    assert cleared == [True]

    next_goal = PoseStamped()
    next_goal.pose.position.x = 2.0
    RLLocalPlannerNodeROS2._on_navigation_goal(harness, next_goal)
    RLLocalPlannerNodeROS2._on_global_plan(harness, _path_to(0.0, 0.0, 0.0))

    assert harness._navigation_cancelled is False
    assert harness._awaiting_goal_plan is True
    assert len(harness.latest_global_plan.poses) == 0

    RLLocalPlannerNodeROS2._on_global_plan(harness, Path())
    RLLocalPlannerNodeROS2._on_global_plan(harness, _path_to(2.0, 0.0, 0.0))

    assert harness._awaiting_goal_plan is False
    assert len(harness.latest_global_plan.poses) == 1


def test_changed_goal_stops_and_accepts_matching_path_without_empty_reset_order():
    cleared = []
    harness = SimpleNamespace(
        latest_global_plan=_path_with_pose(),
        subgoal_position=(1.0, 0.0),
        _global_plan_generation=5,
        _navigation_cancelled=False,
        _navigation_goal_position=np.asarray([1.0, 2.0, 3.0]),
        _awaiting_goal_plan=False,
        _publish_empty_local_paths=lambda: cleared.append(True),
    )
    harness._navigation_goal_changed = lambda msg: (
        RLLocalPlannerNodeROS2._navigation_goal_changed(harness, msg)
    )
    harness._global_plan_matches_navigation_goal = lambda path: (
        RLLocalPlannerNodeROS2._global_plan_matches_navigation_goal(harness, path)
    )
    next_goal = PoseStamped()
    next_goal.pose.position.x = 4.0
    next_goal.pose.position.y = 5.0
    next_goal.pose.position.z = 6.0

    RLLocalPlannerNodeROS2._on_navigation_goal(harness, next_goal)
    RLLocalPlannerNodeROS2._on_global_plan(harness, _path_to(1.0, 2.0, 3.0))

    assert harness._awaiting_goal_plan is True
    assert len(harness.latest_global_plan.poses) == 0
    assert harness.subgoal_position is None
    assert cleared == [True]

    RLLocalPlannerNodeROS2._on_global_plan(harness, _path_to(4.0, 5.0, 6.0))

    assert harness._awaiting_goal_plan is False
    assert len(harness.latest_global_plan.poses) == 1
    assert cleared == [True]


def test_matching_pct_path_received_before_goal_callback_is_reused():
    cleared = []
    next_path = _path_to(4.0, 5.0, 6.0)
    harness = SimpleNamespace(
        latest_global_plan=next_path,
        subgoal_position=(1.0, 0.0),
        _global_plan_generation=7,
        _navigation_cancelled=False,
        _navigation_goal_position=np.asarray([1.0, 2.0, 3.0]),
        _awaiting_goal_plan=False,
        _publish_empty_local_paths=lambda: cleared.append(True),
    )
    harness._navigation_goal_changed = lambda msg: (
        RLLocalPlannerNodeROS2._navigation_goal_changed(harness, msg)
    )
    harness._global_plan_matches_navigation_goal = lambda path: (
        RLLocalPlannerNodeROS2._global_plan_matches_navigation_goal(harness, path)
    )
    next_goal = PoseStamped()
    next_goal.pose.position.x = 4.0
    next_goal.pose.position.y = 5.0
    next_goal.pose.position.z = 6.0

    RLLocalPlannerNodeROS2._on_navigation_goal(harness, next_goal)

    assert harness._awaiting_goal_plan is False
    assert harness.latest_global_plan is next_path
    assert harness.subgoal_position is None
    assert cleared == [True]


def test_repeated_same_goal_does_not_interrupt_active_local_planning():
    cleared = []
    harness = SimpleNamespace(
        latest_global_plan=_path_with_pose(),
        subgoal_position=(1.0, 0.0),
        _global_plan_generation=5,
        _navigation_cancelled=False,
        _navigation_goal_position=np.asarray([1.0, 2.0, 3.0]),
        _awaiting_goal_plan=False,
        _publish_empty_local_paths=lambda: cleared.append(True),
    )
    harness._navigation_goal_changed = lambda msg: (
        RLLocalPlannerNodeROS2._navigation_goal_changed(harness, msg)
    )
    harness._global_plan_matches_navigation_goal = lambda path: (
        RLLocalPlannerNodeROS2._global_plan_matches_navigation_goal(harness, path)
    )
    repeated_goal = PoseStamped()
    repeated_goal.pose.position.x = 1.0
    repeated_goal.pose.position.y = 2.0
    repeated_goal.pose.position.z = 3.0

    RLLocalPlannerNodeROS2._on_navigation_goal(harness, repeated_goal)

    assert harness._awaiting_goal_plan is False
    assert len(harness.latest_global_plan.poses) == 1
    assert cleared == []


def test_delayed_repeated_goal_cannot_release_cancel_latch():
    cleared = []
    harness = SimpleNamespace(
        latest_global_plan=Path(),
        subgoal_position=None,
        _global_plan_generation=9,
        _navigation_cancelled=True,
        _navigation_goal_position=np.asarray([1.0, 2.0, 3.0]),
        _awaiting_goal_plan=False,
        _publish_empty_local_paths=lambda: cleared.append(True),
    )
    harness._navigation_goal_changed = lambda msg: (
        RLLocalPlannerNodeROS2._navigation_goal_changed(harness, msg)
    )
    harness._global_plan_matches_navigation_goal = lambda path: (
        RLLocalPlannerNodeROS2._global_plan_matches_navigation_goal(harness, path)
    )
    delayed_goal = PoseStamped()
    delayed_goal.pose.position.x = 1.0
    delayed_goal.pose.position.y = 2.0
    delayed_goal.pose.position.z = 3.0

    RLLocalPlannerNodeROS2._on_navigation_goal(harness, delayed_goal)

    assert harness._navigation_cancelled is True
    assert harness._global_plan_generation == 9
    assert len(harness.latest_global_plan.poses) == 0
    assert cleared == []
