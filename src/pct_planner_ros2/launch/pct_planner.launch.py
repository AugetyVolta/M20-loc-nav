import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("pct_planner_ros2")
    config_file = PathJoinSubstitution([pkg_share, "config", "pct_planner.yaml"])
    default_pct_root = PathJoinSubstitution([pkg_share, "PCT_planner"])

    pct_root = LaunchConfiguration("pct_root")
    venv_site = LaunchConfiguration("venv_site")

    python_paths = [
        venv_site,
        ":",
        pct_root,
        "/planner/lib:",
        pct_root,
        "/planner:",
        pct_root,
        "/planner/scripts:",
        os.environ.get("PYTHONPATH", ""),
    ]
    library_paths = [
        pct_root,
        "/planner/lib/3rdparty/gtsam-4.1.1/install/lib:",
        pct_root,
        "/planner/lib/3rdparty/osqp/install/lib:",
        pct_root,
        "/planner/lib:",
        pct_root,
        "/planner/lib/build/src/common/smoothing:",
        os.environ.get("LD_LIBRARY_PATH", ""),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("pct_root", default_value=default_pct_root),
            DeclareLaunchArgument("venv_site", default_value="/home/orin/venv/m20_nav_cupy/lib/python3.10/site-packages"),
            DeclareLaunchArgument("tomogram_file", default_value="m20_3d_map"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("start_source", default_value="fixed"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_link"),
            DeclareLaunchArgument("use_interactive_markers", default_value="true"),
            DeclareLaunchArgument("a_star_cost_threshold", default_value="45.0"),
            DeclareLaunchArgument("safe_cost_margin", default_value="15.0"),
            DeclareLaunchArgument("step_cost_weight", default_value="1.0"),
            DeclareLaunchArgument("layer_match_height_tolerance", default_value="1.2"),
            SetEnvironmentVariable("PCT_PLANNER_ROOT", pct_root),
            SetEnvironmentVariable("PYTHONPATH", python_paths),
            SetEnvironmentVariable("LD_LIBRARY_PATH", library_paths),
            Node(
                package="pct_planner_ros2",
                executable="pct_planner_node",
                name="pct_planner_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "pct_root": pct_root,
                        "tomogram_file": LaunchConfiguration("tomogram_file"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "start_source": LaunchConfiguration("start_source"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "global_frame": LaunchConfiguration("global_frame"),
                        "robot_frame": LaunchConfiguration("robot_frame"),
                        "a_star_cost_threshold": ParameterValue(
                            LaunchConfiguration("a_star_cost_threshold"),
                            value_type=float,
                        ),
                        "safe_cost_margin": ParameterValue(
                            LaunchConfiguration("safe_cost_margin"),
                            value_type=float,
                        ),
                        "step_cost_weight": ParameterValue(
                            LaunchConfiguration("step_cost_weight"),
                            value_type=float,
                        ),
                        "layer_match_height_tolerance": ParameterValue(
                            LaunchConfiguration("layer_match_height_tolerance"),
                            value_type=float,
                        ),
                        "use_interactive_markers": ParameterValue(
                            LaunchConfiguration("use_interactive_markers"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
