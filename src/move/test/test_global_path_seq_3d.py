import math
from threading import Lock
from types import SimpleNamespace

import pytest
from std_msgs.msg import Header
from visualization_msgs.msg import (
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from move.global_path_seq_publisher import (
    GlobalPathSequencePublisher,
    SimTimeSafeInteractiveMarkerServer,
    Waypoint,
)


class _Logger:
    def info(self, _message):
        pass


class _MarkerHarness:
    waypoints = [Waypoint(1.0, 2.0, 0.5, marker_id=7)]
    reached_marker_ids = set()
    current_index = 0
    sequence_done = False
    marker_z_offset = 0.2
    marker_visual_z_offset = 0.12
    marker_point_diameter = 0.4
    marker_label_height = 0.35

    _set_waypoint_color = GlobalPathSequencePublisher._set_waypoint_color
    _waypoint_reached = GlobalPathSequencePublisher._waypoint_reached
    _interactive_label = GlobalPathSequencePublisher._interactive_label
    _interactive_marker_name = staticmethod(
        GlobalPathSequencePublisher._interactive_marker_name
    )
    _set_orientation = staticmethod(GlobalPathSequencePublisher._set_orientation)


def _attach_waypoint_state(harness, reached_marker_ids=()):
    harness.reached_marker_ids = set(reached_marker_ids)
    harness._waypoint_reached = lambda waypoint: (
        GlobalPathSequencePublisher._waypoint_reached(harness, waypoint)
    )
    harness._next_unreached_index = lambda start_index=0: (
        GlobalPathSequencePublisher._next_unreached_index(harness, start_index)
    )
    harness._finish_after_edit = lambda: (
        GlobalPathSequencePublisher._finish_after_edit(harness)
    )
    return harness


def test_interactive_server_buffers_burst_feedback(monkeypatch):
    init_calls = []
    monkeypatch.setattr(InteractiveMarkerServer, "shutdown", lambda _server: None)

    def record_init(_server, node, namespace, **kwargs):
        init_calls.append((node, namespace, kwargs))

    monkeypatch.setattr(InteractiveMarkerServer, "__init__", record_init)

    node = object()
    SimTimeSafeInteractiveMarkerServer(node, "waypoint_editor")

    assert len(init_calls) == 1
    assert init_calls[0][0] is node
    assert init_calls[0][1] == "waypoint_editor"
    assert init_calls[0][2]["feedback_sub_qos"].depth == 20


def test_interactive_server_accepts_feedback_after_rviz_client_reconnect(monkeypatch):
    forwarded = []
    monkeypatch.setattr(InteractiveMarkerServer, "shutdown", lambda _server: None)
    monkeypatch.setattr(
        InteractiveMarkerServer,
        "processFeedback",
        lambda _server, feedback: forwarded.append(feedback),
    )
    server = object.__new__(SimTimeSafeInteractiveMarkerServer)
    server.mutex = Lock()
    context = SimpleNamespace(last_client_id="old-rviz-client")
    server.marker_contexts = {"waypoint_7": context}
    feedback = SimpleNamespace(
        marker_name="waypoint_7",
        client_id="new-rviz-client",
    )

    SimTimeSafeInteractiveMarkerServer.processFeedback(server, feedback)

    assert context.last_client_id == "new-rviz-client"
    assert forwarded == [feedback]


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
        waypoints=[
            Waypoint(1.0, 2.0, 0.5, marker_id=1),
            Waypoint(4.0, 5.0, 2.5, marker_id=2),
        ],
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
    _attach_waypoint_state(harness)

    advanced = GlobalPathSequencePublisher._advance_if_reached(harness)

    assert advanced is True
    assert harness.current_index == 1
    assert harness.reached_marker_ids == {1}
    assert harness.sequence_done is False
    assert harness._interactive_markers_dirty is True
    assert published == ["goal"]
    assert "stopped old path and requested the next plan" in statuses[-1]


def test_final_waypoint_clears_paths_and_stops_navigation():
    cancelled = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5, marker_id=1)],
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
    _attach_waypoint_state(harness)

    reached = GlobalPathSequencePublisher._advance_if_reached(harness)

    assert reached is True
    assert harness.sequence_done is True
    assert harness.reached_marker_ids == {1}
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
    assert marker.controls[0].markers[0].pose.position.z == pytest.approx(0.12)
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
    _attach_waypoint_state(harness)
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


def test_real_marker_drag_does_not_reactivate_reached_waypoint():
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
    _attach_waypoint_state(harness, reached_marker_ids={7})
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
    assert harness.sequence_done is True
    assert harness._interactive_markers_dirty is True
    assert published == []
    assert "dragged waypoint 1" in statuses[-1]


def test_dragging_completed_historical_waypoint_does_not_reactivate_final_goal():
    published = []
    harness = SimpleNamespace(
        waypoints=[
            Waypoint(1.0, 2.0, 0.5, marker_id=7),
            Waypoint(4.0, 5.0, 0.5, marker_id=8),
        ],
        marker_z_offset=0.2,
        current_index=1,
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
        _publish_status=lambda _detail=None: None,
    )
    _attach_waypoint_state(harness, reached_marker_ids={7, 8})
    feedback = SimpleNamespace(
        marker_name="waypoint_7",
        event_type=InteractiveMarkerFeedback.MOUSE_UP,
        pose=SimpleNamespace(position=SimpleNamespace(x=1.5, y=2.0, z=0.7)),
    )

    GlobalPathSequencePublisher._on_interactive_feedback(harness, feedback)

    assert harness.waypoints[0].x == pytest.approx(1.5)
    assert harness.sequence_done is True
    assert harness._interactive_markers_dirty is True
    assert published == []


def test_deleting_later_waypoint_never_reactivates_completed_waypoint():
    cancelled = []
    published = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[
            Waypoint(1.0, 2.0, 0.5, marker_id=7),
            Waypoint(4.0, 5.0, 0.5, marker_id=8),
        ],
        current_index=0,
        sequence_done=True,
        _interactive_markers_dirty=False,
        _retire_interactive_marker=lambda _waypoint: None,
        _cancel_goal=lambda: cancelled.append("cancel"),
        _publish_current_goal=lambda: published.append("goal"),
        _mark_interactive_markers_dirty=lambda: setattr(
            harness, "_interactive_markers_dirty", True
        ),
        _publish_visualization=lambda: None,
        _publish_status=lambda detail=None: statuses.append(detail),
    )
    _attach_waypoint_state(harness, reached_marker_ids={7, 8})

    GlobalPathSequencePublisher._delete_waypoint(
        harness, 1, "deleted completed waypoint 2"
    )

    assert harness.waypoints == [Waypoint(1.0, 2.0, 0.5, marker_id=7)]
    assert harness.reached_marker_ids == {7}
    assert harness.current_index == 0
    assert harness.sequence_done is True
    assert cancelled == ["cancel"]
    assert published == []
    assert "deleted completed waypoint 2" in statuses[-1]


def test_deleting_active_tail_after_completed_prefix_cancels_instead_of_restarting():
    cancelled = []
    published = []
    harness = SimpleNamespace(
        waypoints=[
            Waypoint(1.0, 2.0, 0.5, marker_id=7),
            Waypoint(4.0, 5.0, 0.5, marker_id=8),
        ],
        current_index=1,
        sequence_done=False,
        _retire_interactive_marker=lambda _waypoint: None,
        _cancel_goal=lambda: cancelled.append("cancel"),
        _publish_current_goal=lambda: published.append("goal"),
        _mark_interactive_markers_dirty=lambda: None,
        _publish_visualization=lambda: None,
        _publish_status=lambda _detail=None: None,
    )
    _attach_waypoint_state(harness, reached_marker_ids={7})

    GlobalPathSequencePublisher._delete_waypoint(harness, 1, "deleted active tail")

    assert harness.waypoints == [Waypoint(1.0, 2.0, 0.5, marker_id=7)]
    assert harness.reached_marker_ids == {7}
    assert harness.current_index == 0
    assert harness.sequence_done is True
    assert cancelled == ["cancel"]
    assert published == []


def test_deleting_future_waypoint_keeps_current_goal_active_without_republishing():
    cancelled = []
    published = []
    harness = SimpleNamespace(
        waypoints=[
            Waypoint(1.0, 2.0, 0.5, marker_id=7),
            Waypoint(4.0, 5.0, 0.5, marker_id=8),
        ],
        current_index=0,
        sequence_done=False,
        _retire_interactive_marker=lambda _waypoint: None,
        _cancel_goal=lambda: cancelled.append("cancel"),
        _publish_current_goal=lambda: published.append("goal"),
        _mark_interactive_markers_dirty=lambda: None,
        _publish_visualization=lambda: None,
        _publish_status=lambda _detail=None: None,
    )
    _attach_waypoint_state(harness)

    GlobalPathSequencePublisher._delete_waypoint(harness, 1, "deleted future")

    assert harness.waypoints == [Waypoint(1.0, 2.0, 0.5, marker_id=7)]
    assert harness.current_index == 0
    assert harness.sequence_done is False
    assert cancelled == []
    assert published == []


def test_resume_cannot_reactivate_completed_queue():
    cancelled = []
    published = []
    statuses = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5, marker_id=7)],
        sequence_done=True,
        paused=True,
        _cancel_goal=lambda: cancelled.append("cancel"),
        _publish_current_goal=lambda: published.append("goal"),
        _publish_status=lambda detail=None: statuses.append(detail),
    )

    GlobalPathSequencePublisher._on_resume(harness, None)

    assert harness.sequence_done is True
    assert harness.paused is True
    assert cancelled == ["cancel"]
    assert published == []
    assert statuses == ["resume ignored: all remaining waypoints are reached"]


def test_new_waypoint_after_completion_activates_only_the_new_marker():
    published = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5, marker_id=7)],
        current_index=0,
        sequence_done=True,
        paused=False,
        auto_start_on_click=True,
        min_clicked_spacing=0.25,
        _interactive_markers_dirty=False,
        _point_to_waypoint=lambda _msg: Waypoint(4.0, 5.0, 0.5, marker_id=8),
        _distance_between=GlobalPathSequencePublisher._distance_between,
        _mark_interactive_markers_dirty=lambda: setattr(
            harness, "_interactive_markers_dirty", True
        ),
        _publish_current_goal=lambda: published.append(
            harness.waypoints[harness.current_index].marker_id
        ),
        _publish_visualization=lambda: None,
        _publish_status=lambda _detail=None: None,
    )
    _attach_waypoint_state(harness, reached_marker_ids={7})

    GlobalPathSequencePublisher._on_clicked_point(harness, None)

    assert harness.current_index == 1
    assert harness.sequence_done is False
    assert harness.reached_marker_ids == {7}
    assert published == [8]


def test_marker_timer_does_not_rebuild_stable_interactive_markers():
    visualization_calls = []
    harness = SimpleNamespace(
        _interactive_drag_active=False,
        _interactive_drag_last_feedback_time=None,
        interactive_drag_timeout=2.0,
        _interactive_markers_dirty=False,
        _publish_visualization=lambda: visualization_calls.append("published"),
    )

    GlobalPathSequencePublisher._on_marker_timer(harness)

    assert harness._interactive_markers_dirty is False
    assert visualization_calls == ["published"]


def test_marker_timer_recovers_drag_lock_when_mouse_up_feedback_is_lost(monkeypatch):
    warnings = []
    visualization_calls = []
    harness = SimpleNamespace(
        _interactive_drag_active=True,
        _interactive_drag_origin=(7, Waypoint(1.0, 2.0, 0.5, marker_id=7)),
        _interactive_drag_last_feedback_time=10.0,
        interactive_drag_timeout=2.0,
        get_logger=lambda: SimpleNamespace(warn=lambda message: warnings.append(message)),
        _publish_visualization=lambda: visualization_calls.append("published"),
    )
    monkeypatch.setattr(
        "move.global_path_seq_publisher.time.monotonic",
        lambda: 12.1,
    )

    GlobalPathSequencePublisher._on_marker_timer(harness)

    assert harness._interactive_drag_active is False
    assert harness._interactive_drag_origin is None
    assert harness._interactive_drag_last_feedback_time is None
    assert warnings == [
        "Recovered stale RViz waypoint drag lock after feedback timeout"
    ]
    assert visualization_calls == ["published"]


def test_menu_feedback_releases_drag_latch_before_deferring_action():
    menu_feedback = []
    logs = []
    harness = SimpleNamespace(
        waypoints=[Waypoint(1.0, 2.0, 0.5, marker_id=7)],
        _interactive_drag_active=True,
        _interactive_drag_origin=(7, Waypoint(1.0, 2.0, 0.5, marker_id=7)),
        _interactive_drag_last_feedback_time=1.0,
        _interactive_waypoint_index=lambda _name: 0,
        _on_interactive_menu=lambda feedback: menu_feedback.append(feedback),
        get_logger=lambda: SimpleNamespace(info=lambda message: logs.append(message)),
    )
    feedback = SimpleNamespace(
        marker_name="waypoint_7",
        event_type=InteractiveMarkerFeedback.MENU_SELECT,
    )

    GlobalPathSequencePublisher._on_interactive_feedback(harness, feedback)

    assert harness._interactive_drag_active is False
    assert harness._interactive_drag_origin is None
    assert harness._interactive_drag_last_feedback_time is None
    assert menu_feedback == [feedback]


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
        _interactive_drag_active=True,
        _interactive_drag_origin=(0, Waypoint(1.0, 2.0, 0.5)),
        _interactive_drag_last_feedback_time=1.0,
        _interactive_waypoint_index=lambda _name: 0,
        _defer_menu_action=lambda action, index: deferred.append((action, index)),
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None,
            warn=lambda _message: None,
        ),
    )
    feedback = SimpleNamespace(
        marker_name="waypoint_0",
        menu_entry_id=1,
    )

    GlobalPathSequencePublisher._on_interactive_menu(harness, feedback)

    assert deferred == [("delete", 0)]
    assert harness._interactive_drag_active is False
    assert harness._interactive_drag_origin is None
    assert harness._interactive_drag_last_feedback_time is None
