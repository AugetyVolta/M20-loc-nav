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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_source", default_value="tf"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_link"),
            DeclareLaunchArgument("use_interactive_markers", default_value="true"),
            DeclareLaunchArgument("a_star_cost_threshold", default_value="45.0"),
            DeclareLaunchArgument("safe_cost_margin", default_value="15.0"),
            DeclareLaunchArgument("step_cost_weight", default_value="1.0"),
            DeclareLaunchArgument("layer_match_height_tolerance", default_value="1.2"),
            DeclareLaunchArgument("path_ground_offset", default_value="0.10"),
            DeclareLaunchArgument("global_path_perception_enabled", default_value="false"),
            DeclareLaunchArgument("global_path_perception_scan_topic", default_value="/scan"),
            DeclareLaunchArgument("global_path_perception_width", default_value="8.0"),
            DeclareLaunchArgument("global_path_perception_height", default_value="8.0"),
            DeclareLaunchArgument("global_path_perception_inflation_radius", default_value="1.0"),
            DeclareLaunchArgument("global_path_perception_cost_scaling_factor", default_value="5.0"),
            DeclareLaunchArgument("global_path_perception_persistence", default_value="1.0"),
            DeclareLaunchArgument("stair_mode_enabled", default_value="true"),
            DeclareLaunchArgument("stair_disable_global_path_perception", default_value="true"),
            DeclareLaunchArgument("stair_lookahead", default_value="2.5"),
            DeclareLaunchArgument("stair_enter_slope", default_value="0.18"),
            DeclareLaunchArgument("stair_enter_dz", default_value="0.35"),
            DeclareLaunchArgument("stair_up_enter_slope", default_value="0.14"),
            DeclareLaunchArgument("stair_up_enter_dz", default_value="0.28"),
            DeclareLaunchArgument("stair_down_enter_slope", default_value="0.18"),
            DeclareLaunchArgument("stair_down_enter_dz", default_value="0.35"),
            DeclareLaunchArgument("stair_enter_hold_time", default_value="0.5"),
            DeclareLaunchArgument("stair_exit_hold_time", default_value="2.0"),
            DeclareLaunchArgument("stair_min_state_duration", default_value="5.0"),
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
                        "path_ground_offset": ParameterValue(
                            LaunchConfiguration("path_ground_offset"),
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
                        "stair_mode_enabled": ParameterValue(
                            LaunchConfiguration("stair_mode_enabled"),
                            value_type=bool,
                        ),
                        "stair_disable_global_path_perception": ParameterValue(
                            LaunchConfiguration("stair_disable_global_path_perception"),
                            value_type=bool,
                        ),
                        "stair_lookahead": ParameterValue(
                            LaunchConfiguration("stair_lookahead"),
                            value_type=float,
                        ),
                        "stair_enter_slope": ParameterValue(
                            LaunchConfiguration("stair_enter_slope"),
                            value_type=float,
                        ),
                        "stair_enter_dz": ParameterValue(
                            LaunchConfiguration("stair_enter_dz"),
                            value_type=float,
                        ),
                        "stair_up_enter_slope": ParameterValue(
                            LaunchConfiguration("stair_up_enter_slope"),
                            value_type=float,
                        ),
                        "stair_up_enter_dz": ParameterValue(
                            LaunchConfiguration("stair_up_enter_dz"),
                            value_type=float,
                        ),
                        "stair_down_enter_slope": ParameterValue(
                            LaunchConfiguration("stair_down_enter_slope"),
                            value_type=float,
                        ),
                        "stair_down_enter_dz": ParameterValue(
                            LaunchConfiguration("stair_down_enter_dz"),
                            value_type=float,
                        ),
                        "stair_enter_hold_time": ParameterValue(
                            LaunchConfiguration("stair_enter_hold_time"),
                            value_type=float,
                        ),
                        "stair_exit_hold_time": ParameterValue(
                            LaunchConfiguration("stair_exit_hold_time"),
                            value_type=float,
                        ),
                        "stair_min_state_duration": ParameterValue(
                            LaunchConfiguration("stair_min_state_duration"),
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
