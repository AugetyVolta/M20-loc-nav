import numpy as np
from geometry_msgs.msg import PoseStamped

from pct_planner_ros2.pct_planner_node import PctPlannerNode


def _planner_node(start_x, progress_index):
    node = PctPlannerNode.__new__(PctPlannerNode)
    node.reference_path = np.asarray(
        [[float(index), 0.0, 0.0] for index in range(11)],
        dtype=np.float32,
    )
    node.reference_layers = np.zeros((11,), dtype=np.int32)
    node.reference_arc = node._reference_arc_lengths(node.reference_path)
    node.reference_goal = np.array([10.0, 0.0, 0.0], dtype=np.float32)
    node.goal_pos = node.reference_goal.copy()
    node.start_pos = np.array([start_x, 0.0, 0.0], dtype=np.float32)
    node.reference_progress_index = progress_index
    node.reference_rebuild_distance = 2.0
    node.reference_rebuild_backtrack_distance = 0.6
    return node


def test_reference_path_is_kept_during_normal_forward_progress():
    node = _planner_node(start_x=8.1, progress_index=8)

    assert node._reference_path_rebuild_reason() is None


def test_reference_path_rebuilds_after_material_reverse_progress():
    node = _planner_node(start_x=3.1, progress_index=8)

    reason = node._reference_path_rebuild_reason()

    assert reason.startswith("route_backtrack:")


def test_reference_path_rebuilds_when_robot_leaves_route():
    node = _planner_node(start_x=3.0, progress_index=3)
    node.start_pos[2] = 3.0

    reason = node._reference_path_rebuild_reason()

    assert reason.startswith("route_departure:")


def test_changed_goal_clears_old_paths_before_replanning():
    cleared = []
    synced = []
    node = PctPlannerNode.__new__(PctPlannerNode)
    node.goal_received = True
    node.goal_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    node._publish_empty_paths = lambda: cleared.append(True)
    node._sync_marker_pose = lambda name, position: synced.append((name, position.copy()))
    goal = PoseStamped()
    goal.pose.position.x = 4.0
    goal.pose.position.y = 5.0
    goal.pose.position.z = 6.0

    node._on_goal_pose(goal)

    assert cleared == [True]
    np.testing.assert_allclose(node.goal_pos, [4.0, 5.0, 6.0])
    assert synced[0][0] == "end_pos"


def test_repeated_goal_does_not_interrupt_active_path():
    cleared = []
    node = PctPlannerNode.__new__(PctPlannerNode)
    node.goal_received = True
    node.goal_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    node._publish_empty_paths = lambda: cleared.append(True)
    node._sync_marker_pose = lambda _name, _position: None
    goal = PoseStamped()
    goal.pose.position.x = 1.0
    goal.pose.position.y = 2.0
    goal.pose.position.z = 3.0

    node._on_goal_pose(goal)

    assert cleared == []
