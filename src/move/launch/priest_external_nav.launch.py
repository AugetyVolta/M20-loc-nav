"""Launch the external M20 local navigation chain.

This launch consumes an external global path, normally /pct_path from the
separate PCT planner workspace, and starts:

  /pct_path -> pure_pursuit.py -> /subgoal, /final_goal
  /pct_path + /subgoal -> priest_rl_publisher_nav_cmd_fast.py -> /local_path
  /local_path -> priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py -> /NAV_CMD
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    python_executable = LaunchConfiguration("python_executable")
    rl_python_executable = LaunchConfiguration("rl_python_executable")

    start_pure_pursuit = LaunchConfiguration("start_pure_pursuit")
    start_rl_local_path = LaunchConfiguration("start_rl_local_path")
    start_adapter = LaunchConfiguration("start_adapter")
    start_rviz_waypoints = LaunchConfiguration("start_rviz_waypoints")

    global_path_topic = LaunchConfiguration("global_path_topic")
    pure_pursuit_plan_topic = LaunchConfiguration("pure_pursuit_plan_topic")
    subgoal_topic = LaunchConfiguration("subgoal_topic")
    final_goal_topic = LaunchConfiguration("final_goal_topic")
    local_path_topic = LaunchConfiguration("local_path_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic")
    clicked_point_topic = LaunchConfiguration("clicked_point_topic")
    waypoint_delete_topic = LaunchConfiguration("waypoint_delete_topic")
    waypoint_replace_topic = LaunchConfiguration("waypoint_replace_topic")
    waypoint_status_topic = LaunchConfiguration("waypoint_status_topic")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    waypoints_pose_topic = LaunchConfiguration("waypoints_pose_topic")
    waypoint_replan_period = LaunchConfiguration("waypoint_replan_period")
    waypoint_goal_tolerance = LaunchConfiguration("waypoint_goal_tolerance")
    waypoint_edit_radius = LaunchConfiguration("waypoint_edit_radius")
    waypoint_enable_interactive_markers = LaunchConfiguration("waypoint_enable_interactive_markers")
    waypoint_interactive_marker_ns = LaunchConfiguration("waypoint_interactive_marker_ns")

    global_frame = LaunchConfiguration("global_frame")
    robot_frame = LaunchConfiguration("robot_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    path_target_frame = LaunchConfiguration("path_target_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    global_plan_use_3d = LaunchConfiguration("global_plan_use_3d")
    pure_pursuit_use_3d_path_distance = LaunchConfiguration("pure_pursuit_use_3d_path_distance")
    adapter_path_transform_use_3d = LaunchConfiguration("adapter_path_transform_use_3d")

    lookahead = LaunchConfiguration("lookahead")
    pure_pursuit_rate = LaunchConfiguration("pure_pursuit_rate")
    rl_hz = LaunchConfiguration("rl_hz")
    rl_pp_lookahead = LaunchConfiguration("rl_pp_lookahead")
    rl_virt_goal_min = LaunchConfiguration("rl_virt_goal_min")
    rl_virt_goal_max = LaunchConfiguration("rl_virt_goal_max")
    rl_virt_goal_pref = LaunchConfiguration("rl_virt_goal_pref")
    rl_expand_external_subgoal = LaunchConfiguration("rl_expand_external_subgoal")
    use_arc_length_lookahead = LaunchConfiguration("use_arc_length_lookahead")
    heading_change_guard_enabled = LaunchConfiguration("heading_change_guard_enabled")
    max_heading_change_deg = LaunchConfiguration("max_heading_change_deg")
    turn_guard_min_lookahead = LaunchConfiguration("turn_guard_min_lookahead")
    turn_guard_pre_distance = LaunchConfiguration("turn_guard_pre_distance")
    device = LaunchConfiguration("device")
    require_localization_confidence = LaunchConfiguration("require_localization_confidence")
    adapter_goal_send_hz = LaunchConfiguration("adapter_goal_send_hz")
    adapter_min_goal_resend_interval = LaunchConfiguration("adapter_min_goal_resend_interval")
    adapter_path_timeout = LaunchConfiguration("adapter_path_timeout")

    common_env = {
        "PYTHONUNBUFFERED": "1",
        "XLA_PYTHON_CLIENT_PREALLOCATE": "false",
        "MPLCONFIGDIR": "/tmp/matplotlib",
    }

    rviz_waypoints = Node(
        condition=IfCondition(start_rviz_waypoints),
        package="move",
        executable="global_path_seq_publisher",
        name="global_path_sequence_publisher",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "use_static_goals": False,
                "clicked_point_topic": clicked_point_topic,
                "delete_clicked_point_topic": waypoint_delete_topic,
                "replace_clicked_point_topic": waypoint_replace_topic,
                "status_topic": waypoint_status_topic,
                "path_topic": global_path_topic,
                "publish_pure_pursuit_plan": True,
                "pure_pursuit_plan_topic": pure_pursuit_plan_topic,
                "waypoints_topic": waypoints_topic,
                "waypoints_pose_topic": waypoints_pose_topic,
                "global_frame": global_frame,
                "robot_frame": robot_frame,
                "replan_period": waypoint_replan_period,
                "goal_tolerance": waypoint_goal_tolerance,
                "edit_radius": waypoint_edit_radius,
                "enable_interactive_markers": waypoint_enable_interactive_markers,
                "interactive_marker_namespace": waypoint_interactive_marker_ns,
            }
        ],
    )

    pure_pursuit = ExecuteProcess(
        condition=IfCondition(start_pure_pursuit),
        cmd=[
            python_executable,
            "-m",
            "move.pure_pursuit",
            "--ros-args",
            "-r",
            ["plan:=", pure_pursuit_plan_topic],
            "-r",
            ["subgoal:=", subgoal_topic],
            "-r",
            ["final_goal:=", final_goal_topic],
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["lookahead:=", lookahead],
            "-p",
            ["rate:=", pure_pursuit_rate],
            "-p",
            ["world_frame:=", global_frame],
            "-p",
            ["robot_frame:=", robot_frame],
            "-p",
            ["use_3d_path_distance:=", pure_pursuit_use_3d_path_distance],
            "-p",
            ["use_arc_length_lookahead:=", use_arc_length_lookahead],
            "-p",
            ["heading_change_guard_enabled:=", heading_change_guard_enabled],
            "-p",
            ["max_heading_change_deg:=", max_heading_change_deg],
            "-p",
            ["turn_guard_min_lookahead:=", turn_guard_min_lookahead],
            "-p",
            ["turn_guard_pre_distance:=", turn_guard_pre_distance],
        ],
        output="screen",
        additional_env=common_env,
    )

    rl_local_path = ExecuteProcess(
        condition=IfCondition(start_rl_local_path),
        cmd=[
            rl_python_executable,
            "-m",
            "move.priest_rl_publisher_nav_cmd_fast",
            "--ros-args",
            "-r",
            ["local_path:=", local_path_topic],
            "-r",
            ["odom:=", odom_topic],
            "-r",
            ["scan:=", scan_topic],
            "-r",
            ["subgoal:=", subgoal_topic],
            "-r",
            ["final_goal:=", final_goal_topic],
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["global_plan_topic:=", global_path_topic],
            "-p",
            ["global_frame:=", global_frame],
            "-p",
            ["base_frame:=", robot_frame],
            "-p",
            ["frame_id:=", robot_frame],
            "-p",
            ["odom_frame:=", odom_frame],
            "-p",
            ["global_plan_use_3d:=", global_plan_use_3d],
            "-p",
            ["hz:=", rl_hz],
            "-p",
            ["pp_lookahead:=", rl_pp_lookahead],
            "-p",
            ["virt_goal_min:=", rl_virt_goal_min],
            "-p",
            ["virt_goal_max:=", rl_virt_goal_max],
            "-p",
            ["virt_goal_pref:=", rl_virt_goal_pref],
            "-p",
            ["expand_external_subgoal:=", rl_expand_external_subgoal],
            "-p",
            ["device:=", device],
        ],
        output="screen",
        additional_env=common_env,
    )

    adapter = ExecuteProcess(
        condition=IfCondition(start_adapter),
        cmd=[
            python_executable,
            "-m",
            "move.priest_mppi_adapter_nav_cmd_dwb_smooth_responsive",
            "--ros-args",
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["priest_path_topic:=", local_path_topic],
            "-p",
            ["path_target_frame:=", path_target_frame],
            "-p",
            ["base_frame:=", robot_frame],
            "-p",
            ["path_transform_use_3d:=", adapter_path_transform_use_3d],
            "-p",
            ["goal_send_hz:=", adapter_goal_send_hz],
            "-p",
            ["min_goal_resend_interval:=", adapter_min_goal_resend_interval],
            "-p",
            ["path_timeout:=", adapter_path_timeout],
            "-p",
            ["cmd_vel_topic:=", cmd_vel_topic],
            "-p",
            ["nav_cmd_topic:=", nav_cmd_topic],
            "-p",
            ["require_localization_confidence:=", require_localization_confidence],
        ],
        output="screen",
        additional_env=common_env,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("python_executable", default_value="python3"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venv", "m20_nav", "bin", "python"]
                ),
            ),
            DeclareLaunchArgument("start_pure_pursuit", default_value="true"),
            DeclareLaunchArgument("start_rl_local_path", default_value="true"),
            DeclareLaunchArgument("start_adapter", default_value="true"),
            DeclareLaunchArgument("start_rviz_waypoints", default_value="false"),
            DeclareLaunchArgument("global_path_topic", default_value="/pct_path"),
            DeclareLaunchArgument("pure_pursuit_plan_topic", default_value="/pct_path"),
            DeclareLaunchArgument("subgoal_topic", default_value="subgoal"),
            DeclareLaunchArgument("final_goal_topic", default_value="final_goal"),
            DeclareLaunchArgument("local_path_topic", default_value="local_path"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("nav_cmd_topic", default_value="/NAV_CMD"),
            DeclareLaunchArgument("clicked_point_topic", default_value="/clicked_point"),
            DeclareLaunchArgument("waypoint_delete_topic", default_value="/waypoint_sequence/delete_nearest"),
            DeclareLaunchArgument("waypoint_replace_topic", default_value="/waypoint_sequence/replace_nearest"),
            DeclareLaunchArgument("waypoint_status_topic", default_value="/waypoint_sequence/status"),
            DeclareLaunchArgument("waypoints_topic", default_value="waypoints"),
            DeclareLaunchArgument("waypoints_pose_topic", default_value="waypoints_pose_array"),
            DeclareLaunchArgument("waypoint_replan_period", default_value="2.0"),
            DeclareLaunchArgument("waypoint_goal_tolerance", default_value="1.5"),
            DeclareLaunchArgument("waypoint_edit_radius", default_value="1.5"),
            DeclareLaunchArgument("waypoint_enable_interactive_markers", default_value="true"),
            DeclareLaunchArgument("waypoint_interactive_marker_ns", default_value="waypoint_editor"),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_frame", default_value="odom_body"),
            DeclareLaunchArgument("path_target_frame", default_value="odom_body"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("global_plan_use_3d", default_value="true"),
            DeclareLaunchArgument("pure_pursuit_use_3d_path_distance", default_value="true"),
            DeclareLaunchArgument("adapter_path_transform_use_3d", default_value="true"),
            DeclareLaunchArgument("lookahead", default_value="1.8"),
            DeclareLaunchArgument("pure_pursuit_rate", default_value="10.0"),
            DeclareLaunchArgument("rl_hz", default_value="10.0"),
            DeclareLaunchArgument("rl_pp_lookahead", default_value="4.0"),
            DeclareLaunchArgument("rl_virt_goal_min", default_value="3.5"),
            DeclareLaunchArgument("rl_virt_goal_max", default_value="4.0"),
            DeclareLaunchArgument("rl_virt_goal_pref", default_value="4.0"),
            DeclareLaunchArgument("rl_expand_external_subgoal", default_value="false"),
            DeclareLaunchArgument("use_arc_length_lookahead", default_value="true"),
            DeclareLaunchArgument("heading_change_guard_enabled", default_value="true"),
            DeclareLaunchArgument("max_heading_change_deg", default_value="35.0"),
            DeclareLaunchArgument("turn_guard_min_lookahead", default_value="0.6"),
            DeclareLaunchArgument("turn_guard_pre_distance", default_value="0.7"),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("require_localization_confidence", default_value="false"),
            DeclareLaunchArgument("adapter_goal_send_hz", default_value="3.0"),
            DeclareLaunchArgument("adapter_min_goal_resend_interval", default_value="0.35"),
            DeclareLaunchArgument("adapter_path_timeout", default_value="1.0"),
            rviz_waypoints,
            pure_pursuit,
            rl_local_path,
            adapter,
        ]
    )
