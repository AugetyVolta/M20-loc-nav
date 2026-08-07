import math
from types import SimpleNamespace

import pytest
from std_msgs.msg import Header
from visualization_msgs.msg import (
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from move.global_path_seq_publisher import GlobalPathSequencePublisher, Waypoint


class _Logger:
    def info(self, _message):
        pass


class _MarkerHarness:
    waypoints = [Waypoint(1.0, 2.0, 0.5, marker_id=7)]
    current_index = 0
    sequence_done = False
    marker_z_offset = 0.2
    marker_point_diameter = 0.4
    marker_label_height = 0.35

    _set_waypoint_color = GlobalPathSequencePublisher._set_waypoint_color
    _interactive_label = GlobalPathSequencePublisher._interactive_label
    _interactive_marker_name = staticmethod(
        GlobalPathSequencePublisher._interactive_marker_name
    )
    _set_orientation = staticmethod(GlobalPathSequencePublisher._set_orientation)


def test_apply_transform_preserves_3d_height_and_rotation():
    half_sqrt = math.sqrt(0.5)
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=10.0, y=20.0, z=30.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=half_sqrt, w=half_sqrt),
        )
    )

    result = GlobalPathSequencePublisher._apply_transform((1.0, 0.0, 2.0), transform)

    assert result == pytest.approx((10.0, 21.0, 32.0))


def test_reached_waypoint_advances_to_next_3d_goal():
    published = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5), Waypoint(4.0, 5.0, 2.5)],
        current_index=0,
        goal_tolerance=0.8,
        loop=False,
        sequence_done=False,
        _interactive_markers_dirty=False,
        _current_goal_distance=lambda: 0.2,
        _publish_current_goal=lambda: published.append("goal"),
        _publish_visualization=lambda: None,
        _publish_status=lambda detail=None: statuses.append(detail),
        get_logger=lambda: _Logger(),
    )
    harness._mark_interactive_markers_dirty = lambda: setattr(
        harness, "_interactive_markers_dirty", True
    )

    advanced = GlobalPathSequencePublisher._advance_if_reached(harness)

    assert advanced is True
    assert harness.current_index == 1
    assert harness.sequence_done is False
    assert harness._interactive_markers_dirty is True
    assert published == ["goal"]
    assert "stopped old path and requested the next plan" in statuses[-1]


def test_final_waypoint_clears_paths_and_stops_navigation():
    cancelled = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5)],
        current_index=0,
        goal_tolerance=0.8,
        loop=False,
        sequence_done=False,
        _current_goal_distance=lambda: 0.2,
        _cancel_goal=lambda: cancelled.append("cancel"),
        _publish_visualization=lambda: None,
        _publish_status=lambda detail=None: statuses.append(detail),
        get_logger=lambda: _Logger(),
    )
    harness._mark_interactive_markers_dirty = lambda: None

    reached = GlobalPathSequencePublisher._advance_if_reached(harness)

    assert reached is True
    assert harness.sequence_done is True
    assert cancelled == ["cancel"]
    assert statuses[-1] == "sequence completed; paths cleared and navigation stopped"


def test_normal_waypoint_display_does_not_cover_interactive_sphere():
    harness = _MarkerHarness()

    markers = GlobalPathSequencePublisher._build_markers(harness, Header(frame_id="map"))

    spheres = [marker for marker in markers.markers if marker.type == Marker.SPHERE]
    labels = [
        marker for marker in markers.markers
        if marker.type == Marker.TEXT_VIEW_FACING
    ]
    lines = [marker for marker in markers.markers if marker.type == Marker.LINE_STRIP]
    assert spheres == []
    assert lines == []
    assert len(labels) == 1
    assert labels[0].pose.position.x == pytest.approx(1.0)
    assert labels[0].pose.position.y == pytest.approx(2.0)
    assert labels[0].text == "1"


def test_waypoint_has_context_menu_and_normal_mouse_move_controls():
    harness = _MarkerHarness()

    marker = GlobalPathSequencePublisher._make_interactive_marker(
        harness,
        0,
        harness.waypoints[0],
        Header(frame_id="map"),
    )

    assert marker.header.stamp.sec == 0
    assert marker.header.stamp.nanosec == 0
    assert marker.name == "waypoint_7"
    assert marker.controls[0].name == "move_xy"
    assert marker.controls[0].interaction_mode == InteractiveMarkerControl.MOVE_PLANE
    assert marker.controls[0].always_visible is True
    assert marker.controls[0].markers[0].type == Marker.SPHERE
    assert marker.controls[0].markers[0].color.a > 0.9
    assert {control.name for control in marker.controls[1:]} == {
        "move_x",
        "move_y",
        "move_z",
    }


def test_pose_update_keeps_normal_marker_in_sync_without_rebuilding_editor():
    visualization_calls = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5)],
        marker_z_offset=0.2,
        current_index=0,
        sequence_done=False,
        _interactive_markers_dirty=False,
        _interactive_drag_active=False,
        _interactive_drag_origin=None,
        _interactive_waypoint_index=lambda _name: 0,
        _publish_visualization=lambda **kwargs: visualization_calls.append(kwargs),
        _publish_current_goal=lambda: None,
        _publish_status=lambda _detail=None: None,
    )
    feedback = SimpleNamespace(
        marker_name="waypoint_0",
        event_type=InteractiveMarkerFeedback.POSE_UPDATE,
        pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=1.2)),
    )

    GlobalPathSequencePublisher._on_interactive_feedback(harness, feedback)

    assert harness.waypoints[0] == Waypoint(3.0, 4.0, 1.0)
    assert visualization_calls == [{"sync_interactive": False}]
    assert harness._interactive_markers_dirty is False
    assert harness._interactive_drag_active is True
    assert harness._interactive_drag_origin == (-1, Waypoint(1.0, 2.0, 0.5))


def test_marker_sync_mouse_up_does_not_reactivate_completed_sequence():
    published = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5, marker_id=7)],
        marker_z_offset=0.2,
        current_index=0,
        sequence_done=True,
        _interactive_markers_dirty=False,
        _interactive_drag_active=False,
        _interactive_drag_origin=None,
        _interactive_waypoint_index=lambda _name: 0,
        _mark_interactive_markers_dirty=lambda: None,
        _publish_visualization=lambda **_kwargs: None,
        _publish_current_goal=lambda: published.append("goal"),
        _publish_status=lambda detail=None: statuses.append(detail),
    )
    feedback = SimpleNamespace(
        marker_name="waypoint_7",
        event_type=InteractiveMarkerFeedback.MOUSE_UP,
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.7)),
    )

    GlobalPathSequencePublisher._on_interactive_feedback(harness, feedback)

    assert harness.sequence_done is True
    assert harness._interactive_drag_active is False
    assert published == []
    assert statuses == []


def test_real_marker_drag_reactivates_completed_sequence():
    published = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5, marker_id=7)],
        marker_z_offset=0.2,
        current_index=0,
        sequence_done=True,
        _interactive_markers_dirty=False,
        _interactive_drag_active=True,
        _interactive_drag_origin=(7, Waypoint(1.0, 2.0, 0.5, marker_id=7)),
        _interactive_waypoint_index=lambda _name: 0,
        _mark_interactive_markers_dirty=lambda: setattr(
            harness, "_interactive_markers_dirty", True
        ),
        _publish_visualization=lambda **_kwargs: None,
        _publish_current_goal=lambda: published.append("goal"),
        _publish_status=lambda detail=None: statuses.append(detail),
    )
    feedback = SimpleNamespace(
        marker_name="waypoint_7",
        event_type=InteractiveMarkerFeedback.MOUSE_UP,
        pose=SimpleNamespace(position=SimpleNamespace(x=1.5, y=2.0, z=0.7)),
    )

    GlobalPathSequencePublisher._on_interactive_feedback(harness, feedback)

    assert harness.waypoints[0].x == pytest.approx(1.5)
    assert harness.waypoints[0].y == pytest.approx(2.0)
    assert harness.waypoints[0].z == pytest.approx(0.5)
    assert harness.waypoints[0].marker_id == 7
    assert harness.sequence_done is False
    assert harness._interactive_markers_dirty is True
    assert published == ["goal"]
    assert "dragged waypoint 1" in statuses[-1]


def test_marker_timer_republishes_interactive_state_after_structure_change():
    visualization_calls = []
    harness = SimpleNamespace(
        _interactive_resync_remaining=2,
        _interactive_drag_active=False,
        _interactive_markers_dirty=False,
        _publish_visualization=lambda: visualization_calls.append("published"),
    )

    GlobalPathSequencePublisher._on_marker_timer(harness)

    assert harness._interactive_resync_remaining == 1
    assert harness._interactive_markers_dirty is True
    assert visualization_calls == ["published"]


def test_marker_timer_does_not_rebuild_editor_during_drag():
    visualization_calls = []
    harness = SimpleNamespace(
        _interactive_resync_remaining=2,
        _interactive_drag_active=True,
        _interactive_markers_dirty=False,
        _publish_visualization=lambda: visualization_calls.append("published"),
    )

    GlobalPathSequencePublisher._on_marker_timer(harness)

    assert harness._interactive_resync_remaining == 2
    assert harness._interactive_markers_dirty is False
    assert visualization_calls == ["published"]


def test_empty_queue_does_not_insert_invalid_ready_marker():
    inserted = []
    applied = []
    server = SimpleNamespace(
        insert=lambda marker, **_kwargs: inserted.append(marker),
        applyChanges=lambda: applied.append(True),
    )
    harness = SimpleNamespace(
        marker_server=server,
        waypoints=[],
        _interactive_markers_dirty=True,
        _interactive_drag_active=False,
    )

    GlobalPathSequencePublisher._sync_interactive_markers(
        harness,
        Header(frame_id="map"),
    )

    assert inserted == []
    assert applied == [True]
    assert harness._interactive_markers_dirty is False


def test_retired_marker_is_hidden_without_erasing_selected_rviz_object():
    set_pose_calls = []
    callback_calls = []
    marker = SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.7))
    )
    server = SimpleNamespace(
        get=lambda name: marker if name == "waypoint_9" else None,
        setPose=lambda name, pose: set_pose_calls.append((name, pose.position.z)),
        setCallback=lambda *args: callback_calls.append(args),
    )
    harness = SimpleNamespace(
        marker_server=server,
        _interactive_marker_name=GlobalPathSequencePublisher._interactive_marker_name,
    )

    GlobalPathSequencePublisher._retire_interactive_marker(
        harness,
        Waypoint(1.0, 2.0, 0.5, marker_id=9),
    )

    assert set_pose_calls == [("waypoint_9", -1000.0)]
    assert len(callback_calls) == 2


def test_menu_delete_is_deferred_until_after_rviz_feedback_callback():
    deferred = []
    harness = SimpleNamespace(
        delete_menu_entry=1,
        clear_menu_entry=2,
        _interactive_waypoint_index=lambda _name: 0,
        _defer_menu_action=lambda action, index: deferred.append((action, index)),
    )
    feedback = SimpleNamespace(
        marker_name="waypoint_0",
        menu_entry_id=1,
    )

    GlobalPathSequencePublisher._on_interactive_menu(harness, feedback)

    assert deferred == [("delete", 0)]
