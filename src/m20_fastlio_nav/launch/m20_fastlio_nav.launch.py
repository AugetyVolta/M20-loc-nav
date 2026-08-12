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
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


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
    start_pct_waypoints = LaunchConfiguration("start_pct_waypoints")
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
    pct_clicked_point_topic = LaunchConfiguration("pct_clicked_point_topic")
    pct_waypoint_replan_period = LaunchConfiguration("pct_waypoint_replan_period")
    pct_waypoint_goal_tolerance = LaunchConfiguration("pct_waypoint_goal_tolerance")
    pct_waypoint_edit_radius = LaunchConfiguration("pct_waypoint_edit_radius")
    pct_waypoint_marker_z_offset = LaunchConfiguration("pct_waypoint_marker_z_offset")
    pct_waypoint_marker_visual_z_offset = LaunchConfiguration(
        "pct_waypoint_marker_visual_z_offset"
    )
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
    terrain_min_path_length = LaunchConfiguration("terrain_min_path_length")
    terrain_slope_window_max = LaunchConfiguration("terrain_slope_window_max")
    terrain_stair_up_slope = LaunchConfiguration("terrain_stair_up_slope")
    terrain_stair_up_dz = LaunchConfiguration("terrain_stair_up_dz")
    terrain_stair_down_slope = LaunchConfiguration("terrain_stair_down_slope")
    terrain_stair_down_dz = LaunchConfiguration("terrain_stair_down_dz")
    terrain_platform_merge_distance = LaunchConfiguration("terrain_platform_merge_distance")
    terrain_current_exit_margin = LaunchConfiguration("terrain_current_exit_margin")
    terrain_projection_max_distance = LaunchConfiguration("terrain_projection_max_distance")
    mode_pct_dynamic_guard_distance = LaunchConfiguration(
        "mode_pct_dynamic_guard_distance"
    )
    mode_costmap_stair_pre_distance = LaunchConfiguration(
        "mode_costmap_stair_pre_distance"
    )
    mode_scan_stair_pre_distance = LaunchConfiguration("mode_scan_stair_pre_distance")
    mode_heading_guard_pre_distance = LaunchConfiguration(
        "mode_heading_guard_pre_distance"
    )
    mode_gait_switch_pre_distance = LaunchConfiguration(
        "mode_gait_switch_pre_distance"
    )
    stair_gait_udp_enabled = LaunchConfiguration("stair_gait_udp_enabled")
    stair_gait_udp_ip = LaunchConfiguration("stair_gait_udp_ip")
    stair_gait_udp_port = LaunchConfiguration("stair_gait_udp_port")
    stair_gait_require_stationary = LaunchConfiguration("stair_gait_require_stationary")
    stair_gait_stationary_linear_threshold = LaunchConfiguration(
        "stair_gait_stationary_linear_threshold"
    )
    stair_gait_stationary_angular_threshold = LaunchConfiguration(
        "stair_gait_stationary_angular_threshold"
    )
    stair_gait_stationary_hold_time = LaunchConfiguration(
        "stair_gait_stationary_hold_time"
    )
    stair_gait_switch_cooldown = LaunchConfiguration("stair_gait_switch_cooldown")
    stair_gait_pause_nav_cmd_enabled = LaunchConfiguration(
        "stair_gait_pause_nav_cmd_enabled"
    )
    stair_gait_pause_nav_cmd_topic = LaunchConfiguration(
        "stair_gait_pause_nav_cmd_topic"
    )
    stair_gait_pause_before_switch_time = LaunchConfiguration(
        "stair_gait_pause_before_switch_time"
    )
    stair_gait_pause_after_switch_time = LaunchConfiguration(
        "stair_gait_pause_after_switch_time"
    )
    flat_gait_param = LaunchConfiguration("flat_gait_param")
    stair_up_gait_param = LaunchConfiguration("stair_up_gait_param")
    stair_down_gait_param = LaunchConfiguration("stair_down_gait_param")

    pct_pkg_share = FindPackageShare("pct_planner_ros2")
    gtsam_vendor_prefix = FindPackagePrefix("gtsam_vendor")
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
        gtsam_vendor_prefix,
        "/lib:",
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
            "clicked_point_topic": "/clicked_point",
            "waypoint_goal_topic": "/goal_pose",
            "waypoint_robot_ground_offset": pct_robot_ground_offset,
            "waypoint_replan_period": pct_waypoint_replan_period,
            "waypoint_goal_tolerance": pct_waypoint_goal_tolerance,
            "waypoint_edit_radius": pct_waypoint_edit_radius,
            "waypoint_marker_z_offset": pct_waypoint_marker_z_offset,
            "odom_topic": output_odom_topic,
            "scan_topic": rl_scan_topic,
            "global_plan_use_3d": "true",
            "adapter_path_transform_use_3d": "true",
            "adapter_path_timeout": adapter_path_timeout,
            "adapter_pause_nav_cmd_topic": stair_gait_pause_nav_cmd_topic,
            "adapter_pause_nav_cmd_timeout": "0.5",
            "pure_pursuit_use_3d_path_distance": "true",
            "heading_change_guard_enabled": heading_change_guard_enabled,
            "heading_change_guard_stair_only": heading_change_guard_stair_only,
            "navigation_mode_topic": "/navigation_mode",
            "max_heading_change_deg": max_heading_change_deg,
            "turn_guard_min_lookahead": turn_guard_min_lookahead,
            "turn_guard_pre_distance": turn_guard_pre_distance,
        }.items(),
    )

    waypoint_editor = Node(
        condition=IfCondition(start_pct_waypoints),
        package="move",
        executable="global_path_seq_publisher",
        name="global_path_sequence_publisher",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "use_static_goals": False,
                "clicked_point_topic": "/clicked_point",
                "delete_clicked_point_topic": "/waypoint_sequence/delete_nearest",
                "replace_clicked_point_topic": "/waypoint_sequence/replace_nearest",
                "status_topic": "/waypoint_sequence/status",
                "waypoints_topic": "/waypoints",
                "waypoints_pose_topic": "/waypoints_pose_array",
                "global_frame": "map",
                "robot_frame": "base_link",
                "replan_period": ParameterValue(
                    pct_waypoint_replan_period,
                    value_type=float,
                ),
                "goal_tolerance": ParameterValue(
                    pct_waypoint_goal_tolerance,
                    value_type=float,
                ),
                "edit_radius": ParameterValue(
                    pct_waypoint_edit_radius,
                    value_type=float,
                ),
                "enable_interactive_markers": True,
                "interactive_marker_namespace": "waypoint_editor",
                "goal_topic": "/goal_pose",
                "robot_ground_offset": ParameterValue(
                    pct_robot_ground_offset,
                    value_type=float,
                ),
                "marker_z_offset": ParameterValue(
                    pct_waypoint_marker_z_offset,
                    value_type=float,
                ),
                "marker_visual_z_offset": ParameterValue(
                    pct_waypoint_marker_visual_z_offset,
                    value_type=float,
                ),
            }
        ],
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
                        respawn=True,
                        respawn_delay=2.0,
                        parameters=[
                            pct_config_file,
                            {
                                "pct_root": pct_root,
                                "tomogram_file": pct_tomogram_file,
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "frame_id": "map",
                                "path_topic": global_path_topic,
                                "reference_path_topic": "/pct/reference_path",
                                "navigation_mode_topic": "/navigation_mode",
                                "start_source": pct_start_source,
                                "odom_topic": output_odom_topic,
                                "global_frame": "map",
                                "robot_frame": "base_link",
                                "goal_pose_topic": "/goal_pose",
                                "clicked_point_topic": pct_clicked_point_topic,
                                "wait_for_goal": True,
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
                            },
                        ],
                    )
                ],
            ),
        ],
    )
    terrain_mode_group = GroupAction(
        actions=[
            TimerAction(
                period=9.0,
                actions=[
                    Node(
                        package="m20_fastlio_nav",
                        executable="terrain_state_estimator",
                        name="terrain_state_estimator",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "reference_path_topic": "/pct/reference_path",
                                "terrain_state_topic": "/terrain/state",
                                "global_frame": "map",
                                "robot_frame": "base_link",
                                "min_path_length": ParameterValue(
                                    terrain_min_path_length, value_type=float
                                ),
                                "slope_window_max": ParameterValue(
                                    terrain_slope_window_max, value_type=float
                                ),
                                "stair_up_slope": ParameterValue(
                                    terrain_stair_up_slope, value_type=float
                                ),
                                "stair_up_dz": ParameterValue(
                                    terrain_stair_up_dz, value_type=float
                                ),
                                "stair_down_slope": ParameterValue(
                                    terrain_stair_down_slope, value_type=float
                                ),
                                "stair_down_dz": ParameterValue(
                                    terrain_stair_down_dz, value_type=float
                                ),
                                "platform_merge_distance": ParameterValue(
                                    terrain_platform_merge_distance, value_type=float
                                ),
                                "current_exit_margin": ParameterValue(
                                    terrain_current_exit_margin, value_type=float
                                ),
                                "projection_max_distance": ParameterValue(
                                    terrain_projection_max_distance, value_type=float
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="m20_fastlio_nav",
                        executable="navigation_mode_manager",
                        name="navigation_mode_manager",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "terrain_state_topic": "/terrain/state",
                                "navigation_mode_topic": "/navigation_mode",
                                "pct_dynamic_guard_distance": ParameterValue(
                                    mode_pct_dynamic_guard_distance, value_type=float
                                ),
                                "costmap_stair_pre_distance": ParameterValue(
                                    mode_costmap_stair_pre_distance, value_type=float
                                ),
                                "scan_stair_pre_distance": ParameterValue(
                                    mode_scan_stair_pre_distance, value_type=float
                                ),
                                "heading_guard_pre_distance": ParameterValue(
                                    mode_heading_guard_pre_distance, value_type=float
                                ),
                                "gait_switch_pre_distance": ParameterValue(
                                    mode_gait_switch_pre_distance, value_type=float
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="m20_fastlio_nav",
                        executable="navigation_scan_mux",
                        name="navigation_scan_mux",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "raw_scan_topic": scan_topic,
                                "traversability_scan_topic": "/traversability_filtered_scan",
                                "output_scan_topic": "/navigation_scan",
                                "navigation_mode_topic": "/navigation_mode",
                            }
                        ],
                    ),
                ],
            )
        ],
    )
    external_nav_group = GroupAction(
        condition=IfCondition(start_external_nav),
        actions=[TimerAction(period=11.0, actions=[external_nav])],
    )
    stair_gait_group = GroupAction(
        condition=IfCondition(stair_gait_udp_enabled),
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
                                "navigation_mode_topic": "/navigation_mode",
                                "odom_topic": output_odom_topic,
                                "udp_ip": stair_gait_udp_ip,
                                "udp_port": ParameterValue(stair_gait_udp_port, value_type=int),
                                "require_stationary": ParameterValue(
                                    stair_gait_require_stationary,
                                    value_type=bool,
                                ),
                                "stationary_linear_threshold": ParameterValue(
                                    stair_gait_stationary_linear_threshold,
                                    value_type=float,
                                ),
                                "stationary_angular_threshold": ParameterValue(
                                    stair_gait_stationary_angular_threshold,
                                    value_type=float,
                                ),
                                "stationary_hold_time": ParameterValue(
                                    stair_gait_stationary_hold_time,
                                    value_type=float,
                                ),
                                "switch_cooldown": ParameterValue(
                                    stair_gait_switch_cooldown,
                                    value_type=float,
                                ),
                                "pause_nav_cmd_enabled": ParameterValue(
                                    stair_gait_pause_nav_cmd_enabled,
                                    value_type=bool,
                                ),
                                "pause_nav_cmd_topic": stair_gait_pause_nav_cmd_topic,
                                "pause_before_switch_time": ParameterValue(
                                    stair_gait_pause_before_switch_time,
                                    value_type=float,
                                ),
                                "pause_after_switch_time": ParameterValue(
                                    stair_gait_pause_after_switch_time,
                                    value_type=float,
                                ),
                                "initial_gait_param": ParameterValue(
                                    flat_gait_param,
                                    value_type=int,
                                ),
                                "flat_gait_param": ParameterValue(
                                    flat_gait_param,
                                    value_type=int,
                                ),
                                "stair_up_gait_param": ParameterValue(
                                    stair_up_gait_param,
                                    value_type=int,
                                ),
                                "stair_down_gait_param": ParameterValue(
                                    stair_down_gait_param,
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
            DeclareLaunchArgument("start_pct_waypoints", default_value="true"),
            DeclareLaunchArgument("global_path_topic", default_value="/pct_path"),
            # Raw scan generated from Fast-LIO cloud, retained for RViz and A/B comparison.
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            # Explicit scan mux output: raw on flat ground, traversability-filtered near stairs.
            DeclareLaunchArgument("rl_scan_topic", default_value="/navigation_scan"),
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
            DeclareLaunchArgument("pct_use_interactive_markers", default_value="false"),
            DeclareLaunchArgument(
                "pct_clicked_point_topic",
                default_value="/pct_direct_clicked_point",
                description="Direct PCT click input; kept separate from the waypoint queue.",
            ),
            DeclareLaunchArgument("pct_waypoint_replan_period", default_value="0.2"),
            DeclareLaunchArgument("pct_waypoint_goal_tolerance", default_value="1.0"),
            DeclareLaunchArgument("pct_waypoint_edit_radius", default_value="1.0"),
            DeclareLaunchArgument("pct_waypoint_marker_z_offset", default_value="0.2"),
            DeclareLaunchArgument(
                "pct_waypoint_marker_visual_z_offset", default_value="0.12"
            ),
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
            DeclareLaunchArgument("terrain_min_path_length", default_value="0.6"),
            DeclareLaunchArgument("terrain_slope_window_max", default_value="1.5"),
            DeclareLaunchArgument("terrain_stair_up_slope", default_value="0.14"),
            DeclareLaunchArgument("terrain_stair_up_dz", default_value="0.28"),
            DeclareLaunchArgument("terrain_stair_down_slope", default_value="0.18"),
            DeclareLaunchArgument("terrain_stair_down_dz", default_value="0.35"),
            DeclareLaunchArgument("terrain_platform_merge_distance", default_value="3.0"),
            DeclareLaunchArgument("terrain_current_exit_margin", default_value="0.6"),
            DeclareLaunchArgument("terrain_projection_max_distance", default_value="2.0"),
            DeclareLaunchArgument("mode_pct_dynamic_guard_distance", default_value="5.0"),
            DeclareLaunchArgument("mode_costmap_stair_pre_distance", default_value="4.0"),
            DeclareLaunchArgument("mode_scan_stair_pre_distance", default_value="4.0"),
            DeclareLaunchArgument("mode_heading_guard_pre_distance", default_value="3.0"),
            DeclareLaunchArgument("mode_gait_switch_pre_distance", default_value="3.0"),
            DeclareLaunchArgument("stair_gait_udp_enabled", default_value="true"),
            DeclareLaunchArgument("stair_gait_udp_ip", default_value="10.21.31.103"),
            DeclareLaunchArgument("stair_gait_udp_port", default_value="30000"),
            DeclareLaunchArgument("stair_gait_require_stationary", default_value="true"),
            DeclareLaunchArgument(
                "stair_gait_stationary_linear_threshold",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "stair_gait_stationary_angular_threshold",
                default_value="0.10",
            ),
            DeclareLaunchArgument("stair_gait_stationary_hold_time", default_value="0.3"),
            DeclareLaunchArgument("stair_gait_switch_cooldown", default_value="3.0"),
            DeclareLaunchArgument(
                "stair_gait_pause_nav_cmd_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "stair_gait_pause_nav_cmd_topic",
                default_value="/stair_gait_pause_nav_cmd",
            ),
            DeclareLaunchArgument(
                "stair_gait_pause_before_switch_time",
                default_value="0.6",
            ),
            DeclareLaunchArgument(
                "stair_gait_pause_after_switch_time",
                default_value="1.0",
            ),
            DeclareLaunchArgument("flat_gait_param", default_value="4097"),
            DeclareLaunchArgument("stair_up_gait_param", default_value="4097"),
            DeclareLaunchArgument("stair_down_gait_param", default_value="4099"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venv", "m20_nav", "bin", "python"]
                ),
            ),
            waypoint_editor,
            localization,
            nav2_group,
            terrain_mode_group,
            pct_planner_group,
            external_nav_group,
            stair_gait_group,
        ]
    )
