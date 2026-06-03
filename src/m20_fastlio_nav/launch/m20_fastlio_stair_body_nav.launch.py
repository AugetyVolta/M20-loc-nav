from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    map_pcd = LaunchConfiguration("map_pcd")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_external_nav = LaunchConfiguration("start_external_nav")
    start_pct_adapter = LaunchConfiguration("start_pct_adapter")
    start_body_scan = LaunchConfiguration("start_body_scan")
    rl_python_executable = LaunchConfiguration("rl_python_executable")
    pct_path_topic = LaunchConfiguration("pct_path_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    body_scan_min_height = LaunchConfiguration("body_scan_min_height")
    body_scan_max_height = LaunchConfiguration("body_scan_max_height")
    body_scan_topic = LaunchConfiguration("body_scan_topic")
    body_odom_topic = LaunchConfiguration("body_odom_topic")
    rl_pp_lookahead = LaunchConfiguration("rl_pp_lookahead")
    rl_virt_goal_min = LaunchConfiguration("rl_virt_goal_min")
    rl_virt_goal_max = LaunchConfiguration("rl_virt_goal_max")
    rl_virt_goal_pref = LaunchConfiguration("rl_virt_goal_pref")

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
            "start_scan": "false",
            "rviz": rviz,
        }.items(),
    )

    body_odom_bridge = Node(
        package="m20_fastlio_nav",
        executable="body_plane_odom_bridge",
        name="body_plane_odom_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "source_odom_topic": "/Odometry_loc",
                "output_odom_topic": body_odom_topic,
                "odom_frame": "odom",
                "body_odom_frame": "odom_body",
                "base_frame": "base_link",
                "publish_tf": True,
                "reset_on_wall_time_gap": True,
                "bag_switch_wall_gap_sec": 1.5,
                "base_to_body_translation": [0.32713234, 0.01413551, 0.31238696],
                "base_to_body_quaternion": [
                    -0.00394028,
                    0.24367785,
                    0.00970223,
                    0.96979969,
                ],
            }
        ],
    )

    body_scan = Node(
        condition=IfCondition(start_body_scan),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="fastlio_pointcloud_to_laserscan_body",
        output="screen",
        remappings=[
            ("cloud_in", "/cloud_registered_body_1"),
            ("scan", body_scan_topic),
        ],
        parameters=[
            {
                "target_frame": "base_link",
                "transform_tolerance": 0.05,
                "min_height": ParameterValue(body_scan_min_height, value_type=float),
                "max_height": ParameterValue(body_scan_max_height, value_type=float),
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.01745,
                "scan_time": 0.1,
                "range_min": 0.35,
                "range_max": 10.0,
                "use_inf": True,
                "use_sim_time": use_sim_time,
            }
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

    pct_adapter = Node(
        condition=IfCondition(start_pct_adapter),
        package="move",
        executable="pct_path_adapter",
        name="pct_path_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pct_path_topic": pct_path_topic,
                "global_path_topic": global_path_topic,
                "output_frame": "map",
                "force_output_frame": False,
                "restamp": True,
                "min_path_points": 2,
                "republish_hz": 2.0,
            }
        ],
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
            "start_pure_pursuit": "false",
            "start_rl_local_path": "true",
            "start_adapter": "true",
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
            "odom_topic": body_odom_topic,
            "scan_topic": body_scan_topic,
            "global_plan_use_3d": "true",
            "adapter_path_transform_use_3d": "true",
            "rl_hz": "10.0",
            "rl_pp_lookahead": rl_pp_lookahead,
            "rl_virt_goal_min": rl_virt_goal_min,
            "rl_virt_goal_max": rl_virt_goal_max,
            "rl_virt_goal_pref": rl_virt_goal_pref,
            "adapter_goal_send_hz": "10.0",
            "adapter_min_goal_resend_interval": "0.10",
            "adapter_path_timeout": "0.60",
        }.items(),
    )

    nav2_group = GroupAction(
        condition=IfCondition(start_nav2),
        actions=[TimerAction(period=8.0, actions=[nav2_navigation])],
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
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("m20_fastlio_nav"), "config", "nav2_dwb_body_plane.yaml"]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_external_nav", default_value="true"),
            DeclareLaunchArgument("start_pct_adapter", default_value="true"),
            DeclareLaunchArgument("start_body_scan", default_value="true"),
            DeclareLaunchArgument("pct_path_topic", default_value="/pct_path"),
            DeclareLaunchArgument("global_path_topic", default_value="global_path"),
            DeclareLaunchArgument("body_scan_topic", default_value="/scan_body"),
            DeclareLaunchArgument("body_odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("body_scan_min_height", default_value="-0.10"),
            DeclareLaunchArgument("body_scan_max_height", default_value="0.65"),
            DeclareLaunchArgument("rl_pp_lookahead", default_value="4.0"),
            DeclareLaunchArgument("rl_virt_goal_min", default_value="3.5"),
            DeclareLaunchArgument("rl_virt_goal_max", default_value="4.0"),
            DeclareLaunchArgument("rl_virt_goal_pref", default_value="4.0"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venv", "m20_nav", "bin", "python"]
                ),
            ),
            localization,
            body_odom_bridge,
            body_scan,
            nav2_group,
            pct_adapter,
            external_nav_group,
        ]
    )
