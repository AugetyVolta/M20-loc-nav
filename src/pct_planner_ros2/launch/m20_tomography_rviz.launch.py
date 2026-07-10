import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pct_planner_ros2.pct_launch_env import (
    DEFAULT_VENV_SITE,
    cuda_library_path_substitutions,
)


def generate_launch_description():
    pkg_share = FindPackageShare("pct_planner_ros2")
    rviz_config = PathJoinSubstitution([pkg_share, "config", "m20_pct.rviz"])
    config_file = PathJoinSubstitution([pkg_share, "config", "tomography.yaml"])
    default_pct_root = PathJoinSubstitution([pkg_share, "PCT_planner"])

    pct_root = LaunchConfiguration("pct_root")
    venv_site = LaunchConfiguration("venv_site")
    python_paths = [
        venv_site,
        ":",
        pct_root,
        "/tomography:",
        pct_root,
        "/tomography/scripts:",
        os.environ.get("PYTHONPATH", ""),
    ]
    library_paths = cuda_library_path_substitutions(venv_site) + [
        os.environ.get("LD_LIBRARY_PATH", "")
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("pct_root", default_value=default_pct_root),
            DeclareLaunchArgument("venv_site", default_value=DEFAULT_VENV_SITE),
            SetEnvironmentVariable("PCT_PLANNER_ROOT", pct_root),
            SetEnvironmentVariable("PYTHONPATH", python_paths),
            SetEnvironmentVariable("LD_LIBRARY_PATH", library_paths),
            Node(
                package="pct_planner_ros2",
                executable="pct_tomography_node",
                name="pct_tomography_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "pct_root": pct_root,
                    },
                ],
            ),
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config], output="screen"),
        ]
    )
