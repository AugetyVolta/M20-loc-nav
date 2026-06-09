import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("pct_planner_ros2")
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("pct_root", default_value=default_pct_root),
            DeclareLaunchArgument("venv_site", default_value="/home/orin/venv/m20_nav_cupy/lib/python3.10/site-packages"),
            SetEnvironmentVariable("PCT_PLANNER_ROOT", pct_root),
            SetEnvironmentVariable("PYTHONPATH", python_paths),
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
        ]
    )
