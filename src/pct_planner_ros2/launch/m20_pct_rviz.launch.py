import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pct_planner_ros2.pct_launch_env import (
    DEFAULT_VENV_SITE,
    cuda_library_path_substitutions,
)


def generate_launch_description():
    pkg_share = FindPackageShare("pct_planner_ros2")
    rviz_config = PathJoinSubstitution([pkg_share, "config", "m20_pct.rviz"])
    config_file = PathJoinSubstitution([pkg_share, "config", "pct_planner.yaml"])
    default_pct_root = PathJoinSubstitution([pkg_share, "PCT_planner"])

    pct_root = LaunchConfiguration("pct_root")
    venv_site = LaunchConfiguration("venv_site")
    python_paths = [
        venv_site,
        ":",
        pct_root,
        "/planner/scripts:",
        pct_root,
        "/planner:",
        pct_root,
        "/planner/lib:",
        os.environ.get("PYTHONPATH", ""),
    ]
    library_paths = cuda_library_path_substitutions(venv_site) + [
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
            DeclareLaunchArgument("venv_site", default_value=DEFAULT_VENV_SITE),
            DeclareLaunchArgument("tomogram_file", default_value="m20_3d_map"),
            DeclareLaunchArgument(
                "pcd_file",
                default_value=EnvironmentVariable(
                    "M20_MAP_PCD",
                    default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd",
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_source", default_value="tf"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_link"),
            DeclareLaunchArgument("a_star_cost_threshold", default_value="45.0"),
            DeclareLaunchArgument("safe_cost_margin", default_value="15.0"),
            DeclareLaunchArgument("step_cost_weight", default_value="1.0"),
            DeclareLaunchArgument("layer_match_height_tolerance", default_value="1.2"),
            DeclareLaunchArgument("global_path_perception_enabled", default_value="false"),
            DeclareLaunchArgument("global_path_perception_scan_topic", default_value="/scan"),
            DeclareLaunchArgument("global_path_perception_width", default_value="4.0"),
            DeclareLaunchArgument("global_path_perception_height", default_value="4.0"),
            DeclareLaunchArgument("global_path_perception_inflation_radius", default_value="0.60"),
            DeclareLaunchArgument("global_path_perception_cost_scaling_factor", default_value="5.0"),
            DeclareLaunchArgument("global_path_perception_persistence", default_value="1.0"),
            DeclareLaunchArgument("tomogram_visual_cost_max", default_value="45.0"),
            DeclareLaunchArgument("publish_pcd", default_value="true"),
            DeclareLaunchArgument("publish_tomogram", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
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
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
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
                        "global_path_perception_enabled": ParameterValue(
                            LaunchConfiguration("global_path_perception_enabled"),
                            value_type=bool,
                        ),
                        "global_path_perception_scan_topic": LaunchConfiguration("global_path_perception_scan_topic"),
                        "global_path_perception_width": ParameterValue(
                            LaunchConfiguration("global_path_perception_width"),
                            value_type=float,
                        ),
                        "global_path_perception_height": ParameterValue(
                            LaunchConfiguration("global_path_perception_height"),
                            value_type=float,
                        ),
                        "global_path_perception_inflation_radius": ParameterValue(
                            LaunchConfiguration("global_path_perception_inflation_radius"),
                            value_type=float,
                        ),
                        "global_path_perception_cost_scaling_factor": ParameterValue(
                            LaunchConfiguration("global_path_perception_cost_scaling_factor"),
                            value_type=float,
                        ),
                        "global_path_perception_persistence": ParameterValue(
                            LaunchConfiguration("global_path_perception_persistence"),
                            value_type=float,
                        ),
                        "use_interactive_markers": True,
                    },
                ],
            ),
            Node(
                package="pct_planner_ros2",
                executable="pct_map_viz_node",
                name="pct_map_viz_node",
                output="screen",
                parameters=[
                    {
                        "pct_root": pct_root,
                        "pcd_file": LaunchConfiguration("pcd_file"),
                        "tomogram_file": LaunchConfiguration("tomogram_file"),
                        "map_frame": "map",
                        "publish_pcd": ParameterValue(LaunchConfiguration("publish_pcd"), value_type=bool),
                        "publish_tomogram": ParameterValue(
                            LaunchConfiguration("publish_tomogram"),
                            value_type=bool,
                        ),
                        "tomogram_visual_cost_max": ParameterValue(
                            LaunchConfiguration("tomogram_visual_cost_max"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
        ]
    )
