from geometry_msgs.msg import Twist

from move.priest_mppi_adapter_nav_cmd_dwb_smooth_responsive import (
    PriestMppiAdapterNavCmd,
)


class _Clock:
    def now(self):
        return "now"


class _AdapterHarness:
    _follow_path_active = PriestMppiAdapterNavCmd._follow_path_active

    def __init__(self, *, path_active):
        self.latest_path = object() if path_active else None
        self._active_goal_handle = object() if path_active else None
        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = None
        self._cmd_timeout_active = True

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
