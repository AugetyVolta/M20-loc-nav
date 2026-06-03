from .convertion import transTrajGrid2Map


def traj2ros(traj):
    from .vis_ros import traj2ros as _traj2ros

    return _traj2ros(traj)
