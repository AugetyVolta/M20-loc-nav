from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    map_pcd = LaunchConfiguration("map_pcd")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")
    start_external_nav = LaunchConfiguration("start_external_nav")
    start_rviz_waypoints = LaunchConfiguration("start_rviz_waypoints")
    rl_python_executable = LaunchConfiguration("rl_python_executable")
    clicked_point_topic = LaunchConfiguration("clicked_point_topic")
    waypoint_delete_topic = LaunchConfiguration("waypoint_delete_topic")
    waypoint_replace_topic = LaunchConfiguration("waypoint_replace_topic")
    waypoint_status_topic = LaunchConfiguration("waypoint_status_topic")
    waypoint_replan_period = LaunchConfiguration("waypoint_replan_period")
    waypoint_goal_tolerance = LaunchConfiguration("waypoint_goal_tolerance")
    waypoint_edit_radius = LaunchConfiguration("waypoint_edit_radius")
    waypoint_enable_interactive_markers = LaunchConfiguration("waypoint_enable_interactive_markers")
    waypoint_interactive_marker_ns = LaunchConfiguration("waypoint_interactive_marker_ns")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("m20_fastlio_nav"), "launch", "m20_fastlio_localization.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_livox": start_livox,
            "map_pcd": map_pcd,
            "scan_topic": "/scan",
            "output_odom_topic": "/odom",
            "rviz": rviz,
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_server"]},
        ],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
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
            "start_rviz_waypoints": start_rviz_waypoints,
            "clicked_point_topic": clicked_point_topic,
            "waypoint_delete_topic": waypoint_delete_topic,
            "waypoint_replace_topic": waypoint_replace_topic,
            "waypoint_status_topic": waypoint_status_topic,
            "waypoint_replan_period": waypoint_replan_period,
            "waypoint_goal_tolerance": waypoint_goal_tolerance,
            "waypoint_edit_radius": waypoint_edit_radius,
            "waypoint_enable_interactive_markers": waypoint_enable_interactive_markers,
            "waypoint_interactive_marker_ns": waypoint_interactive_marker_ns,
            "global_path_topic": "global_path",
            "pure_pursuit_plan_topic": "global_path",
            "subgoal_topic": "subgoal",
            "final_goal_topic": "final_goal",
            "local_path_topic": "local_path",
            "cmd_vel_topic": "/cmd_vel",
            "nav_cmd_topic": "/NAV_CMD",
            "global_frame": "map",
            "robot_frame": "base_footprint",
            "odom_frame": "odom_nav",
            "path_target_frame": "odom_nav",
        }.items(),
    )

    external_nav_group = GroupAction(
        condition=IfCondition(start_external_nav),
        actions=[TimerAction(period=11.0, actions=[external_nav])],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_livox", default_value="false"),
            DeclareLaunchArgument(
                "map_pcd",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("m20_fastlio_nav"), "config", "nav2_dwb_fastlio.yaml"]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("start_external_nav", default_value="true"),
            DeclareLaunchArgument("start_rviz_waypoints", default_value="true"),
            DeclareLaunchArgument("clicked_point_topic", default_value="/clicked_point"),
            DeclareLaunchArgument("waypoint_delete_topic", default_value="/waypoint_sequence/delete_nearest"),
            DeclareLaunchArgument("waypoint_replace_topic", default_value="/waypoint_sequence/replace_nearest"),
            DeclareLaunchArgument("waypoint_status_topic", default_value="/waypoint_sequence/status"),
            DeclareLaunchArgument("waypoint_replan_period", default_value="2.0"),
            DeclareLaunchArgument("waypoint_goal_tolerance", default_value="1.5"),
            DeclareLaunchArgument("waypoint_edit_radius", default_value="1.5"),
            DeclareLaunchArgument("waypoint_enable_interactive_markers", default_value="true"),
            DeclareLaunchArgument("waypoint_interactive_marker_ns", default_value="waypoint_editor"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venv", "m20_nav", "bin", "python"]
                ),
            ),
            localization,
            TimerAction(period=3.0, actions=[map_server, map_lifecycle]),
            TimerAction(period=8.0, actions=[nav2_navigation]),
            external_nav_group,
        ]
    )
