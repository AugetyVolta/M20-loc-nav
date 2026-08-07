from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

from move.priest_mppi_adapter_nav_cmd_dwb_smooth_responsive import (
    PriestMppiAdapterNavCmd,
)


class _Clock:
    def now(self):
        return "now"


class _AdapterHarness:
    _follow_path_active = PriestMppiAdapterNavCmd._follow_path_active
    _cancel_active_follow_path = PriestMppiAdapterNavCmd._cancel_active_follow_path
    _clear_navigation_state = PriestMppiAdapterNavCmd._clear_navigation_state

    def __init__(self, *, path_active):
        self.latest_path = object() if path_active else None
        self.latest_path_time = "stamp" if path_active else None
        self.latest_path_seq = 1
        self.last_sent_seq = 1 if path_active else -1
        self.min_path_points = 4
        self._active_goal_handle = object() if path_active else None
        self._active_goal_seq = 1 if path_active else -1
        self._path_epoch = 0
        self._cancel_requested = False
        self._pending_goal = False
        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = None
        self._cmd_timeout_active = True
        self.published_commands = []
        self.cancel_reasons = []
        self.empty_paths = 0
        self._publish_nav_cmd = (
            lambda x, y, yaw: self.published_commands.append((x, y, yaw))
        )
        self._cancel_follow_path_goal_handle = (
            lambda _goal, reason: self.cancel_reasons.append(reason)
        )
        self._publish_empty_mppi_path = lambda: setattr(
            self, "empty_paths", self.empty_paths + 1
        )

    @staticmethod
    def get_clock():
        return _Clock()


def test_cmd_vel_is_accepted_while_follow_path_is_active():
    adapter = _AdapterHarness(path_active=True)
    command = Twist()
    command.linear.x = 0.5

    PriestMppiAdapterNavCmd._on_cmd_vel(adapter, command)

    assert adapter.latest_cmd_vel.linear.x == 0.5
    assert adapter.latest_cmd_time == "now"
    assert adapter._cmd_timeout_active is False


def test_stale_cmd_vel_is_rejected_after_local_path_is_cleared():
    adapter = _AdapterHarness(path_active=False)
    command = Twist()
    command.linear.x = 0.5
    command.angular.z = 0.6

    PriestMppiAdapterNavCmd._on_cmd_vel(adapter, command)

    assert adapter.latest_cmd_vel.linear.x == 0.0
    assert adapter.latest_cmd_vel.angular.z == 0.0
    assert adapter.latest_cmd_time is None


def test_empty_local_path_clears_path_cancels_controller_and_publishes_zero():
    adapter = _AdapterHarness(path_active=True)
    adapter.latest_path_time = "stamp"
    adapter.latest_path_seq = 4
    adapter.last_sent_seq = 4
    adapter._cancel_requested = False

    PriestMppiAdapterNavCmd._on_path(adapter, Path())

    assert adapter.latest_path is None
    assert adapter._active_goal_handle is None
    assert adapter.published_commands[-1] == (0.0, 0.0, 0.0)
    assert adapter.cancel_reasons == ["local path cleared"]
    assert adapter.empty_paths == 1
    assert adapter._path_epoch == 1


def test_nonempty_local_path_is_accepted_after_empty_path_reset():
    adapter = _AdapterHarness(path_active=False)
    adapter.min_path_points = 2
    adapter.latest_path_time = None
    adapter.latest_path_seq = 3
    adapter.last_sent_seq = -1
    adapter._cancel_requested = True
    path = Path()
    path.poses = [PoseStamped(), PoseStamped()]

    PriestMppiAdapterNavCmd._on_path(adapter, Path())
    PriestMppiAdapterNavCmd._on_path(adapter, path)

    assert adapter.latest_path is path


class _AcceptedGoalHandle:
    accepted = True


class _CompletedFuture:
    @staticmethod
    def result():
        return _AcceptedGoalHandle()


def test_follow_path_response_from_before_a_path_reset_is_cancelled():
    adapter = _AdapterHarness(path_active=False)
    adapter.latest_path = Path()
    adapter.latest_path.poses = [PoseStamped(), PoseStamped()]
    adapter._path_epoch = 4
    adapter._pending_goal = True

    PriestMppiAdapterNavCmd._on_goal_response(
        adapter,
        _CompletedFuture(),
        goal_seq=adapter.latest_path_seq,
        path_epoch=3,
    )

    assert adapter._pending_goal is False
    assert adapter._active_goal_handle is None
    assert adapter.cancel_reasons == ["stale FollowPath response after path reset"]
