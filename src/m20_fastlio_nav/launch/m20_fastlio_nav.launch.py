from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_livox = LaunchConfiguration("start_livox")
    fastlio_frontend = LaunchConfiguration("fastlio_frontend")
    map_pcd = LaunchConfiguration("map_pcd")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_pct_planner = LaunchConfiguration("start_pct_planner")
    start_external_nav = LaunchConfiguration("start_external_nav")
    start_pure_pursuit = LaunchConfiguration("start_pure_pursuit")
    start_rl_local_path = LaunchConfiguration("start_rl_local_path")
    start_adapter = LaunchConfiguration("start_adapter")
    scan_topic = LaunchConfiguration("scan_topic")
    rl_scan_topic = LaunchConfiguration("rl_scan_topic")
    body_scan_min_height = LaunchConfiguration("body_scan_min_height")
    body_scan_max_height = LaunchConfiguration("body_scan_max_height")
    start_initialpose_3d_marker = LaunchConfiguration("start_initialpose_3d_marker")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    heading_change_guard_enabled = LaunchConfiguration("heading_change_guard_enabled")
    heading_change_guard_stair_only = LaunchConfiguration("heading_change_guard_stair_only")
    max_heading_change_deg = LaunchConfiguration("max_heading_change_deg")
    turn_guard_min_lookahead = LaunchConfiguration("turn_guard_min_lookahead")
    turn_guard_pre_distance = LaunchConfiguration("turn_guard_pre_distance")
    adapter_path_timeout = LaunchConfiguration("adapter_path_timeout")
    rl_python_executable = LaunchConfiguration("rl_python_executable")
    pct_root = LaunchConfiguration("pct_root")
    pct_venv_site = LaunchConfiguration("pct_venv_site")
    pct_tomogram_file = LaunchConfiguration("pct_tomogram_file")
    pct_start_source = LaunchConfiguration("pct_start_source")
    pct_replan_interval = LaunchConfiguration("pct_replan_interval")
    pct_always_replan = LaunchConfiguration("pct_always_replan")
    pct_a_star_cost_threshold = LaunchConfiguration("pct_a_star_cost_threshold")
    pct_safe_cost_margin = LaunchConfiguration("pct_safe_cost_margin")
    pct_step_cost_weight = LaunchConfiguration("pct_step_cost_weight")
    pct_layer_match_height_tolerance = LaunchConfiguration("pct_layer_match_height_tolerance")
    pct_robot_ground_offset = LaunchConfiguration("pct_robot_ground_offset")
    pct_use_interactive_markers = LaunchConfiguration("pct_use_interactive_markers")
    pct_global_path_perception_enabled = LaunchConfiguration("pct_global_path_perception_enabled")
    pct_global_path_perception_scan_topic = LaunchConfiguration("pct_global_path_perception_scan_topic")
    pct_global_path_perception_min_range = LaunchConfiguration(
        "pct_global_path_perception_min_range"
    )
    pct_global_path_perception_width = LaunchConfiguration("pct_global_path_perception_width")
    pct_global_path_perception_height = LaunchConfiguration("pct_global_path_perception_height")
    pct_global_path_perception_inflation_radius = LaunchConfiguration("pct_global_path_perception_inflation_radius")
    pct_global_path_perception_inscribed_radius = LaunchConfiguration("pct_global_path_perception_inscribed_radius")
    pct_global_path_perception_cost_scaling_factor = LaunchConfiguration("pct_global_path_perception_cost_scaling_factor")
    pct_global_path_perception_persistence = LaunchConfiguration("pct_global_path_perception_persistence")
    pct_global_path_perception_raytrace_enabled = LaunchConfiguration(
        "pct_global_path_perception_raytrace_enabled"
    )
    pct_global_path_perception_raytrace_max_range = LaunchConfiguration(
        "pct_global_path_perception_raytrace_max_range"
    )
    pct_global_path_perception_raytrace_max_rays = LaunchConfiguration(
        "pct_global_path_perception_raytrace_max_rays"
    )
    pct_local_replan_enabled = LaunchConfiguration("pct_local_replan_enabled")
    pct_local_replan_forward_distance = LaunchConfiguration(
        "pct_local_replan_forward_distance"
    )
    pct_local_replan_join_extension = LaunchConfiguration(
        "pct_local_replan_join_extension"
    )
    pct_stair_mode_enabled = LaunchConfiguration("pct_stair_mode_enabled")
    pct_stair_disable_global_path_perception = LaunchConfiguration("pct_stair_disable_global_path_perception")
    pct_stair_lookahead = LaunchConfiguration("pct_stair_lookahead")
    pct_stair_dynamic_guard_lookahead = LaunchConfiguration(
        "pct_stair_dynamic_guard_lookahead"
    )
    pct_stair_enter_slope = LaunchConfiguration("pct_stair_enter_slope")
    pct_stair_enter_dz = LaunchConfiguration("pct_stair_enter_dz")
    pct_stair_up_enter_slope = LaunchConfiguration("pct_stair_up_enter_slope")
    pct_stair_up_enter_dz = LaunchConfiguration("pct_stair_up_enter_dz")
    pct_stair_down_enter_slope = LaunchConfiguration("pct_stair_down_enter_slope")
    pct_stair_down_enter_dz = LaunchConfiguration("pct_stair_down_enter_dz")
    pct_stair_enter_hold_time = LaunchConfiguration("pct_stair_enter_hold_time")
    pct_stair_exit_hold_time = LaunchConfiguration("pct_stair_exit_hold_time")
    pct_stair_min_state_duration = LaunchConfiguration("pct_stair_min_state_duration")
    pct_stair_gait_udp_enabled = LaunchConfiguration("pct_stair_gait_udp_enabled")
    pct_stair_gait_udp_ip = LaunchConfiguration("pct_stair_gait_udp_ip")
    pct_stair_gait_udp_port = LaunchConfiguration("pct_stair_gait_udp_port")
    pct_stair_gait_require_stationary = LaunchConfiguration("pct_stair_gait_require_stationary")
    pct_stair_gait_stationary_linear_threshold = LaunchConfiguration("pct_stair_gait_stationary_linear_threshold")
    pct_stair_gait_stationary_angular_threshold = LaunchConfiguration("pct_stair_gait_stationary_angular_threshold")
    pct_stair_gait_stationary_hold_time = LaunchConfiguration("pct_stair_gait_stationary_hold_time")
    pct_stair_gait_switch_cooldown = LaunchConfiguration("pct_stair_gait_switch_cooldown")
    pct_stair_gait_pause_nav_cmd_enabled = LaunchConfiguration("pct_stair_gait_pause_nav_cmd_enabled")
    pct_stair_gait_pause_nav_cmd_topic = LaunchConfiguration("pct_stair_gait_pause_nav_cmd_topic")
    pct_stair_gait_pause_before_switch_time = LaunchConfiguration("pct_stair_gait_pause_before_switch_time")
    pct_stair_gait_pause_after_switch_time = LaunchConfiguration("pct_stair_gait_pause_after_switch_time")
    pct_flat_gait_param = LaunchConfiguration("pct_flat_gait_param")
    pct_stair_up_gait_param = LaunchConfiguration("pct_stair_up_gait_param")
    pct_stair_down_gait_param = LaunchConfiguration("pct_stair_down_gait_param")

    pct_pkg_share = FindPackageShare("pct_planner_ros2")
    default_pct_root = PathJoinSubstitution([pct_pkg_share, "PCT_planner"])
    pct_config_file = PathJoinSubstitution([pct_pkg_share, "config", "pct_planner.yaml"])
    pct_python_paths = [
        pct_venv_site,
        ":",
        pct_root,
        "/planner/lib:",
        pct_root,
        "/planner:",
        pct_root,
        "/planner/scripts:",
        EnvironmentVariable("PYTHONPATH", default_value=""),
    ]
    pct_library_paths = [
        pct_root,
        "/planner/lib/3rdparty/gtsam-4.1.1/install/lib:",
        pct_root,
        "/planner/lib/3rdparty/osqp/install/lib:",
        pct_root,
        "/planner/lib:",
        pct_root,
        "/planner/lib/build/src/common/smoothing:",
        EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("m20_fastlio_nav"), "launch", "m20_fastlio_localization.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_livox": start_livox,
            "fastlio_frontend": fastlio_frontend,
            "map_pcd": map_pcd,
            "scan_topic": scan_topic,
            "output_odom_topic": output_odom_topic,
            "start_scan": "true",
            "scan_min_height": body_scan_min_height,
            "scan_max_height": body_scan_max_height,
            "rviz": rviz,
            "rviz_config": rviz_config,
            "start_initialpose_3d_marker": start_initialpose_3d_marker,
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("m20_fastlio_nav"), "launch", "m20_nav2_controller.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    external_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("move"), "launch", "priest_external_nav.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rl_python_executable": rl_python_executable,
            "start_rviz_waypoints": "false",
            "start_pure_pursuit": start_pure_pursuit,
            "start_rl_local_path": start_rl_local_path,
            "start_adapter": start_adapter,
            "global_path_topic": global_path_topic,
            "pure_pursuit_plan_topic": global_path_topic,
            "subgoal_topic": "subgoal",
            "final_goal_topic": "final_goal",
            "local_path_topic": "local_path",
            "cmd_vel_topic": "/cmd_vel",
            "nav_cmd_topic": "/NAV_CMD",
            "global_frame": "map",
            "robot_frame": "base_link",
            "odom_frame": "odom_body",
            "path_target_frame": "odom_body",
            "odom_topic": output_odom_topic,
            "scan_topic": rl_scan_topic,
            "global_plan_use_3d": "true",
            "adapter_path_transform_use_3d": "true",
            "adapter_path_timeout": adapter_path_timeout,
            "adapter_pause_nav_cmd_topic": pct_stair_gait_pause_nav_cmd_topic,
            "adapter_pause_nav_cmd_timeout": "0.5",
            "pure_pursuit_use_3d_path_distance": "true",
            "heading_change_guard_enabled": heading_change_guard_enabled,
            "heading_change_guard_stair_only": heading_change_guard_stair_only,
            "stair_state_topic": "/pct_stair_state",
            "max_heading_change_deg": max_heading_change_deg,
            "turn_guard_min_lookahead": turn_guard_min_lookahead,
            "turn_guard_pre_distance": turn_guard_pre_distance,
        }.items(),
    )

    nav2_group = GroupAction(
        condition=IfCondition(start_nav2),
        actions=[TimerAction(period=8.0, actions=[nav2_navigation])],
    )
    pct_planner_group = GroupAction(
        condition=IfCondition(start_pct_planner),
        actions=[
            SetEnvironmentVariable("PCT_PLANNER_ROOT", pct_root),
            SetEnvironmentVariable("PYTHONPATH", pct_python_paths),
            SetEnvironmentVariable("LD_LIBRARY_PATH", pct_library_paths),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="pct_planner_ros2",
                        executable="pct_planner_node",
                        name="pct_planner_node",
                        output="screen",
                        parameters=[
                            pct_config_file,
                            {
                                "pct_root": pct_root,
                                "tomogram_file": pct_tomogram_file,
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "frame_id": "map",
                                "path_topic": global_path_topic,
                                "start_source": pct_start_source,
                                "odom_topic": output_odom_topic,
                                "global_frame": "map",
                                "robot_frame": "base_link",
                                "replan_interval": ParameterValue(pct_replan_interval, value_type=float),
                                "always_replan": ParameterValue(pct_always_replan, value_type=bool),
                                "a_star_cost_threshold": ParameterValue(
                                    pct_a_star_cost_threshold,
                                    value_type=float,
                                ),
                                "safe_cost_margin": ParameterValue(
                                    pct_safe_cost_margin,
                                    value_type=float,
                                ),
                                "step_cost_weight": ParameterValue(
                                    pct_step_cost_weight,
                                    value_type=float,
                                ),
                                "layer_match_height_tolerance": ParameterValue(
                                    pct_layer_match_height_tolerance,
                                    value_type=float,
                                ),
                                "robot_ground_offset": ParameterValue(
                                    pct_robot_ground_offset,
                                    value_type=float,
                                ),
                                "use_interactive_markers": ParameterValue(
                                    pct_use_interactive_markers,
                                    value_type=bool,
                                ),
                                "global_path_perception_enabled": ParameterValue(
                                    pct_global_path_perception_enabled,
                                    value_type=bool,
                                ),
                                "global_path_perception_scan_topic": pct_global_path_perception_scan_topic,
                                "global_path_perception_min_range": ParameterValue(
                                    pct_global_path_perception_min_range,
                                    value_type=float,
                                ),
                                "global_path_perception_width": ParameterValue(
                                    pct_global_path_perception_width,
                                    value_type=float,
                                ),
                                "global_path_perception_height": ParameterValue(
                                    pct_global_path_perception_height,
                                    value_type=float,
                                ),
                                "global_path_perception_inflation_radius": ParameterValue(
                                    pct_global_path_perception_inflation_radius,
                                    value_type=float,
                                ),
                                "global_path_perception_inscribed_radius": ParameterValue(
                                    pct_global_path_perception_inscribed_radius,
                                    value_type=float,
                                ),
                                "global_path_perception_cost_scaling_factor": ParameterValue(
                                    pct_global_path_perception_cost_scaling_factor,
                                    value_type=float,
                                ),
                                "global_path_perception_persistence": ParameterValue(
                                    pct_global_path_perception_persistence,
                                    value_type=float,
                                ),
                                "global_path_perception_raytrace_enabled": ParameterValue(
                                    pct_global_path_perception_raytrace_enabled,
                                    value_type=bool,
                                ),
                                "global_path_perception_raytrace_max_range": ParameterValue(
                                    pct_global_path_perception_raytrace_max_range,
                                    value_type=float,
                                ),
                                "global_path_perception_raytrace_max_rays": ParameterValue(
                                    pct_global_path_perception_raytrace_max_rays,
                                    value_type=int,
                                ),
                                "local_replan_enabled": ParameterValue(
                                    pct_local_replan_enabled,
                                    value_type=bool,
                                ),
                                "local_replan_forward_distance": ParameterValue(
                                    pct_local_replan_forward_distance,
                                    value_type=float,
                                ),
                                "local_replan_join_extension": ParameterValue(
                                    pct_local_replan_join_extension,
                                    value_type=float,
                                ),
                                "stair_mode_enabled": ParameterValue(
                                    pct_stair_mode_enabled,
                                    value_type=bool,
                                ),
                                "stair_disable_global_path_perception": ParameterValue(
                                    pct_stair_disable_global_path_perception,
                                    value_type=bool,
                                ),
                                "stair_lookahead": ParameterValue(
                                    pct_stair_lookahead,
                                    value_type=float,
                                ),
                                "stair_dynamic_guard_lookahead": ParameterValue(
                                    pct_stair_dynamic_guard_lookahead,
                                    value_type=float,
                                ),
                                "stair_enter_slope": ParameterValue(
                                    pct_stair_enter_slope,
                                    value_type=float,
                                ),
                                "stair_enter_dz": ParameterValue(
                                    pct_stair_enter_dz,
                                    value_type=float,
                                ),
                                "stair_up_enter_slope": ParameterValue(
                                    pct_stair_up_enter_slope,
                                    value_type=float,
                                ),
                                "stair_up_enter_dz": ParameterValue(
                                    pct_stair_up_enter_dz,
                                    value_type=float,
                                ),
                                "stair_down_enter_slope": ParameterValue(
                                    pct_stair_down_enter_slope,
                                    value_type=float,
                                ),
                                "stair_down_enter_dz": ParameterValue(
                                    pct_stair_down_enter_dz,
                                    value_type=float,
                                ),
                                "stair_enter_hold_time": ParameterValue(
                                    pct_stair_enter_hold_time,
                                    value_type=float,
                                ),
                                "stair_exit_hold_time": ParameterValue(
                                    pct_stair_exit_hold_time,
                                    value_type=float,
                                ),
                                "stair_min_state_duration": ParameterValue(
                                    pct_stair_min_state_duration,
                                    value_type=float,
                                ),
                            },
                        ],
                    )
                ],
            ),
        ],
    )
    external_nav_group = GroupAction(
        condition=IfCondition(start_external_nav),
        actions=[TimerAction(period=11.0, actions=[external_nav])],
    )
    stair_gait_group = GroupAction(
        condition=IfCondition(pct_stair_gait_udp_enabled),
        actions=[
            TimerAction(
                period=11.0,
                actions=[
                    Node(
                        package="m20_fastlio_nav",
                        executable="stair_gait_manager",
                        name="stair_gait_manager",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "enabled": True,
                                "stair_state_topic": "/pct_stair_state",
                                "odom_topic": output_odom_topic,
                                "udp_ip": pct_stair_gait_udp_ip,
                                "udp_port": ParameterValue(pct_stair_gait_udp_port, value_type=int),
                                "require_stationary": ParameterValue(
                                    pct_stair_gait_require_stationary,
                                    value_type=bool,
                                ),
                                "stationary_linear_threshold": ParameterValue(
                                    pct_stair_gait_stationary_linear_threshold,
                                    value_type=float,
                                ),
                                "stationary_angular_threshold": ParameterValue(
                                    pct_stair_gait_stationary_angular_threshold,
                                    value_type=float,
                                ),
                                "stationary_hold_time": ParameterValue(
                                    pct_stair_gait_stationary_hold_time,
                                    value_type=float,
                                ),
                                "switch_cooldown": ParameterValue(
                                    pct_stair_gait_switch_cooldown,
                                    value_type=float,
                                ),
                                "pause_nav_cmd_enabled": ParameterValue(
                                    pct_stair_gait_pause_nav_cmd_enabled,
                                    value_type=bool,
                                ),
                                "pause_nav_cmd_topic": pct_stair_gait_pause_nav_cmd_topic,
                                "pause_before_switch_time": ParameterValue(
                                    pct_stair_gait_pause_before_switch_time,
                                    value_type=float,
                                ),
                                "pause_after_switch_time": ParameterValue(
                                    pct_stair_gait_pause_after_switch_time,
                                    value_type=float,
                                ),
                                "initial_gait_param": ParameterValue(
                                    pct_flat_gait_param,
                                    value_type=int,
                                ),
                                "flat_gait_param": ParameterValue(
                                    pct_flat_gait_param,
                                    value_type=int,
                                ),
                                "stair_up_gait_param": ParameterValue(
                                    pct_stair_up_gait_param,
                                    value_type=int,
                                ),
                                "stair_down_gait_param": ParameterValue(
                                    pct_stair_down_gait_param,
                                    value_type=int,
                                ),
                            }
                        ],
                    )
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_livox", default_value="false"),
            DeclareLaunchArgument(
                "fastlio_frontend",
                default_value="fast_lio",
                description="Fast-LIO frontend package for A/B testing: fast_lio_map or fast_lio.",
            ),
            DeclareLaunchArgument(
                "map_pcd",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("m20_fastlio_nav"), "config", "nav2_dwb_body_plane.yaml"]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("m20_fastlio_nav"), "config", "m20_nav3d.rviz"]
                ),
            ),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_pct_planner", default_value="true"),
            DeclareLaunchArgument("start_external_nav", default_value="true"),
            DeclareLaunchArgument("start_pure_pursuit", default_value="true"),
            DeclareLaunchArgument("start_rl_local_path", default_value="true"),
            DeclareLaunchArgument("start_adapter", default_value="true"),
            DeclareLaunchArgument("global_path_topic", default_value="/pct_path"),
            # Raw scan generated from Fast-LIO cloud, retained for RViz and A/B comparison.
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            # Filtered scan consumed by RL/PRIEST local path generation.
            DeclareLaunchArgument("rl_scan_topic", default_value="/traversability_filtered_scan"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("heading_change_guard_enabled", default_value="true"),
            DeclareLaunchArgument("heading_change_guard_stair_only", default_value="true"),
            DeclareLaunchArgument("max_heading_change_deg", default_value="35.0"),
            DeclareLaunchArgument("turn_guard_min_lookahead", default_value="0.6"),
            DeclareLaunchArgument("turn_guard_pre_distance", default_value="0.7"),
            DeclareLaunchArgument("adapter_path_timeout", default_value="1.2"),
            DeclareLaunchArgument("body_scan_min_height", default_value="-0.1"),
            DeclareLaunchArgument("body_scan_max_height", default_value="0.8"),
            DeclareLaunchArgument("start_initialpose_3d_marker", default_value="true"),
            DeclareLaunchArgument("pct_root", default_value=default_pct_root),
            DeclareLaunchArgument(
                "pct_venv_site",
                default_value="/home/orin/venv/m20_nav_cupy/lib/python3.10/site-packages",
            ),
            DeclareLaunchArgument("pct_tomogram_file", default_value="m20_3d_map"),
            DeclareLaunchArgument("pct_start_source", default_value="tf"),
            DeclareLaunchArgument("pct_replan_interval", default_value="1.0"),
            DeclareLaunchArgument("pct_always_replan", default_value="true"),
            DeclareLaunchArgument("pct_a_star_cost_threshold", default_value="45.0"),
            DeclareLaunchArgument("pct_safe_cost_margin", default_value="15.0"),
            DeclareLaunchArgument("pct_step_cost_weight", default_value="1.0"),
            DeclareLaunchArgument("pct_layer_match_height_tolerance", default_value="1.2"),
            DeclareLaunchArgument("pct_robot_ground_offset", default_value="0.45"),
            DeclareLaunchArgument("pct_use_interactive_markers", default_value="true"),
            DeclareLaunchArgument("pct_global_path_perception_enabled", default_value="true"),
            DeclareLaunchArgument("pct_global_path_perception_scan_topic", default_value="/scan"),
            DeclareLaunchArgument("pct_global_path_perception_min_range", default_value="0.15"),
            DeclareLaunchArgument("pct_global_path_perception_width", default_value="8.0"),
            DeclareLaunchArgument("pct_global_path_perception_height", default_value="8.0"),
            DeclareLaunchArgument("pct_global_path_perception_inflation_radius", default_value="1.0"),
            DeclareLaunchArgument("pct_global_path_perception_inscribed_radius", default_value="0.45"),
            DeclareLaunchArgument("pct_global_path_perception_cost_scaling_factor", default_value="5.0"),
            DeclareLaunchArgument("pct_global_path_perception_persistence", default_value="5.0"),
            DeclareLaunchArgument("pct_global_path_perception_raytrace_enabled", default_value="true"),
            DeclareLaunchArgument("pct_global_path_perception_raytrace_max_range", default_value="0.0"),
            DeclareLaunchArgument("pct_global_path_perception_raytrace_max_rays", default_value="360"),
            DeclareLaunchArgument("pct_local_replan_enabled", default_value="true"),
            DeclareLaunchArgument("pct_local_replan_forward_distance", default_value="10.0"),
            DeclareLaunchArgument("pct_local_replan_join_extension", default_value="2.0"),
            DeclareLaunchArgument("pct_stair_mode_enabled", default_value="true"),
            DeclareLaunchArgument("pct_stair_disable_global_path_perception", default_value="true"),
            DeclareLaunchArgument("pct_stair_lookahead", default_value="5.0"),
            DeclareLaunchArgument("pct_stair_dynamic_guard_lookahead", default_value="5.0"),
            DeclareLaunchArgument("pct_stair_enter_slope", default_value="0.18"),
            DeclareLaunchArgument("pct_stair_enter_dz", default_value="0.35"),
            DeclareLaunchArgument("pct_stair_up_enter_slope", default_value="0.14"),
            DeclareLaunchArgument("pct_stair_up_enter_dz", default_value="0.28"),
            DeclareLaunchArgument("pct_stair_down_enter_slope", default_value="0.18"),
            DeclareLaunchArgument("pct_stair_down_enter_dz", default_value="0.35"),
            DeclareLaunchArgument("pct_stair_enter_hold_time", default_value="0.5"),
            DeclareLaunchArgument("pct_stair_exit_hold_time", default_value="2.0"),
            DeclareLaunchArgument("pct_stair_min_state_duration", default_value="5.0"),
            DeclareLaunchArgument("pct_stair_gait_udp_enabled", default_value="true"),
            DeclareLaunchArgument("pct_stair_gait_udp_ip", default_value="10.21.31.103"),
            DeclareLaunchArgument("pct_stair_gait_udp_port", default_value="30000"),
            DeclareLaunchArgument("pct_stair_gait_require_stationary", default_value="true"),
            DeclareLaunchArgument("pct_stair_gait_stationary_linear_threshold", default_value="0.05"),
            DeclareLaunchArgument("pct_stair_gait_stationary_angular_threshold", default_value="0.10"),
            DeclareLaunchArgument("pct_stair_gait_stationary_hold_time", default_value="0.3"),
            DeclareLaunchArgument("pct_stair_gait_switch_cooldown", default_value="3.0"),
            DeclareLaunchArgument("pct_stair_gait_pause_nav_cmd_enabled", default_value="true"),
            DeclareLaunchArgument("pct_stair_gait_pause_nav_cmd_topic", default_value="/stair_gait_pause_nav_cmd"),
            DeclareLaunchArgument("pct_stair_gait_pause_before_switch_time", default_value="0.6"),
            DeclareLaunchArgument("pct_stair_gait_pause_after_switch_time", default_value="1.0"),
            DeclareLaunchArgument("pct_flat_gait_param", default_value="4097"),
            DeclareLaunchArgument("pct_stair_up_gait_param", default_value="4097"),
            DeclareLaunchArgument("pct_stair_down_gait_param", default_value="4099"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venv", "m20_nav", "bin", "python"]
                ),
            ),
            localization,
            nav2_group,
            pct_planner_group,
            external_nav_group,
            stair_gait_group,
        ]
    )
