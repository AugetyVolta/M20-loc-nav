from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pct_planner_ros2.pct_launch_env import (
    DEFAULT_VENV_SITE,
    cuda_library_path_substitutions,
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_livox = LaunchConfiguration("start_livox")
    fastlio_frontend = LaunchConfiguration("fastlio_frontend")
    map_pcd = LaunchConfiguration("map_pcd")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    local_planner_type = LaunchConfiguration("local_planner_type")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_pct_planner = LaunchConfiguration("start_pct_planner")
    start_external_nav = LaunchConfiguration("start_external_nav")
    start_pure_pursuit = LaunchConfiguration("start_pure_pursuit")
    start_rl_local_path = LaunchConfiguration("start_rl_local_path")
    start_adapter = LaunchConfiguration("start_adapter")
    scan_topic = LaunchConfiguration("scan_topic")
    body_scan_min_height = LaunchConfiguration("body_scan_min_height")
    body_scan_max_height = LaunchConfiguration("body_scan_max_height")
    start_initialpose_3d_marker = LaunchConfiguration("start_initialpose_3d_marker")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    ego_local_path_topic = LaunchConfiguration("ego_local_path_topic")
    ego_goal_topic = LaunchConfiguration("ego_goal_topic")
    ego_goal_frame = LaunchConfiguration("ego_goal_frame")
    ego_goal_z_mode = LaunchConfiguration("ego_goal_z_mode")
    ego_goal_z = LaunchConfiguration("ego_goal_z")
    rl_python_executable = LaunchConfiguration("rl_python_executable")
    pct_root = LaunchConfiguration("pct_root")
    pct_venv_site = LaunchConfiguration("pct_venv_site")
    pct_tomogram_file = LaunchConfiguration("pct_tomogram_file")
    pct_start_source = LaunchConfiguration("pct_start_source")
    pct_replan_interval = LaunchConfiguration("pct_replan_interval")
    pct_position_epsilon = LaunchConfiguration("pct_position_epsilon")
    pct_always_replan = LaunchConfiguration("pct_always_replan")
    pct_a_star_cost_threshold = LaunchConfiguration("pct_a_star_cost_threshold")
    pct_safe_cost_margin = LaunchConfiguration("pct_safe_cost_margin")
    pct_step_cost_weight = LaunchConfiguration("pct_step_cost_weight")
    pct_layer_match_height_tolerance = LaunchConfiguration("pct_layer_match_height_tolerance")
    pct_use_interactive_markers = LaunchConfiguration("pct_use_interactive_markers")
    pct_global_path_perception_enabled = LaunchConfiguration("pct_global_path_perception_enabled")
    pct_global_path_perception_scan_topic = LaunchConfiguration("pct_global_path_perception_scan_topic")
    pct_global_path_perception_width = LaunchConfiguration("pct_global_path_perception_width")
    pct_global_path_perception_height = LaunchConfiguration("pct_global_path_perception_height")
    pct_global_path_perception_inflation_radius = LaunchConfiguration("pct_global_path_perception_inflation_radius")
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

    pct_pkg_share = FindPackageShare("pct_planner_ros2")
    default_pct_root = PathJoinSubstitution([pct_pkg_share, "PCT_planner"])
    pct_config_file = PathJoinSubstitution([pct_pkg_share, "config", "pct_planner.yaml"])

    pct_python_paths = [
        pct_venv_site,
        ":",
        pct_root,
        "/planner/scripts:",
        pct_root,
        "/planner:",
        pct_root,
        "/planner/lib:",
        EnvironmentVariable("PYTHONPATH", default_value=""),
    ]
    pct_library_paths = cuda_library_path_substitutions(pct_venv_site) + [
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
    use_ego_local_planner = PythonExpression(["'", local_planner_type, "' == 'ego'"])

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
            "start_rl_local_path": PythonExpression([
                "'false' if '",
                local_planner_type,
                "' == 'ego' else '",
                start_rl_local_path,
                "'",
            ]),
            "start_adapter": start_adapter,
            "global_path_topic": global_path_topic,
            "pure_pursuit_plan_topic": global_path_topic,
            "subgoal_topic": "subgoal",
            "final_goal_topic": "final_goal",
            "local_path_topic": PythonExpression([
                "'",
                ego_local_path_topic,
                "' if '",
                local_planner_type,
                "' == 'ego' else 'local_path'",
            ]),
            "cmd_vel_topic": "/cmd_vel",
            "nav_cmd_topic": "/NAV_CMD",
            "global_frame": "map",
            "robot_frame": "base_link",
            "odom_frame": "odom_body",
            "path_target_frame": "odom_body",
            "odom_topic": output_odom_topic,
            "scan_topic": scan_topic,
            "global_plan_use_3d": "true",
            "adapter_path_transform_use_3d": "true",
            "pure_pursuit_use_3d_path_distance": "true",
            "pure_pursuit_publish_ego_goal": use_ego_local_planner,
            "ego_goal_topic": ego_goal_topic,
            "ego_goal_frame": ego_goal_frame,
            "ego_goal_z_mode": ego_goal_z_mode,
            "ego_goal_z": ego_goal_z,
        }.items(),
    )

    ego_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("m20_fastlio_nav"), "launch", "m20_ego_local_planner.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "source_odom_topic": output_odom_topic,
            "ego_odom_topic": "/ego_odom",
            "input_cloud_topic": "/cloud_registered_body_1",
            "ego_goal_topic": ego_goal_topic,
            "output_path_topic": ego_local_path_topic,
            "output_path_frame": "odom_body",
        }.items(),
    )

    nav2_group = GroupAction(
        condition=IfCondition(start_nav2),
        actions=[TimerAction(period=8.0, actions=[nav2_navigation])],
    )
    pct_planner_group = GroupAction(
        condition=IfCondition(start_pct_planner),
        actions=[
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="pct_planner_ros2",
                        executable="pct_planner_node",
                        name="pct_planner_node",
                        output="screen",
                        additional_env={
                            "PCT_PLANNER_ROOT": pct_root,
                            "PYTHONPATH": pct_python_paths,
                            "LD_LIBRARY_PATH": pct_library_paths,
                        },
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
                                "position_epsilon": ParameterValue(pct_position_epsilon, value_type=float),
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
                                "use_interactive_markers": ParameterValue(
                                    pct_use_interactive_markers,
                                    value_type=bool,
                                ),
                                "global_path_perception_enabled": ParameterValue(
                                    pct_global_path_perception_enabled,
                                    value_type=bool,
                                ),
                                "global_path_perception_scan_topic": pct_global_path_perception_scan_topic,
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
    ego_local_planner_group = GroupAction(
        condition=IfCondition(use_ego_local_planner),
        actions=[TimerAction(period=12.0, actions=[ego_local_planner])],
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
                default_value=EnvironmentVariable(
                    "M20_MAP_PCD",
                    default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd",
                ),
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
            DeclareLaunchArgument(
                "local_planner_type",
                default_value="priest_rl",
                description="Local planner chain: priest_rl keeps the original chain; ego uses EGO-Planner + DWB.",
            ),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_pct_planner", default_value="true"),
            DeclareLaunchArgument("start_external_nav", default_value="true"),
            DeclareLaunchArgument("start_pure_pursuit", default_value="true"),
            DeclareLaunchArgument("start_rl_local_path", default_value="true"),
            DeclareLaunchArgument("start_adapter", default_value="true"),
            DeclareLaunchArgument("global_path_topic", default_value="/pct_path"),
            DeclareLaunchArgument("ego_local_path_topic", default_value="/ego_local_path"),
            DeclareLaunchArgument("ego_goal_topic", default_value="/ego_goal_pose"),
            DeclareLaunchArgument("ego_goal_frame", default_value="map"),
            DeclareLaunchArgument("ego_goal_z_mode", default_value="path"),
            DeclareLaunchArgument("ego_goal_z", default_value="0.5"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("body_scan_min_height", default_value="-0.1"),
            DeclareLaunchArgument("body_scan_max_height", default_value="0.55"),
            DeclareLaunchArgument("start_initialpose_3d_marker", default_value="true"),
            DeclareLaunchArgument("pct_root", default_value=default_pct_root),
            DeclareLaunchArgument(
                "pct_venv_site",
                default_value=DEFAULT_VENV_SITE,
            ),
            DeclareLaunchArgument("pct_tomogram_file", default_value="m20_3d_map"),
            DeclareLaunchArgument("pct_start_source", default_value="tf"),
            DeclareLaunchArgument("pct_replan_interval", default_value="1.0"),
            DeclareLaunchArgument("pct_position_epsilon", default_value="0.2"),
            DeclareLaunchArgument("pct_always_replan", default_value="true"),
            DeclareLaunchArgument("pct_a_star_cost_threshold", default_value="45.0"),
            DeclareLaunchArgument("pct_safe_cost_margin", default_value="15.0"),
            DeclareLaunchArgument("pct_step_cost_weight", default_value="1.0"),
            DeclareLaunchArgument("pct_layer_match_height_tolerance", default_value="1.2"),
            DeclareLaunchArgument("pct_use_interactive_markers", default_value="true"),
            DeclareLaunchArgument("pct_global_path_perception_enabled", default_value="true"),
            DeclareLaunchArgument("pct_global_path_perception_scan_topic", default_value="/traversability_filtered_scan"),
            DeclareLaunchArgument("pct_global_path_perception_width", default_value="6.0"),
            DeclareLaunchArgument("pct_global_path_perception_height", default_value="6.0"),
            DeclareLaunchArgument("pct_global_path_perception_inflation_radius", default_value="0.60"),
            DeclareLaunchArgument("pct_global_path_perception_cost_scaling_factor", default_value="5.0"),
            DeclareLaunchArgument("pct_global_path_perception_persistence", default_value="5.0"),
            DeclareLaunchArgument("pct_global_path_perception_raytrace_enabled", default_value="true"),
            DeclareLaunchArgument("pct_global_path_perception_raytrace_max_range", default_value="0.0"),
            DeclareLaunchArgument("pct_global_path_perception_raytrace_max_rays", default_value="360"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=EnvironmentVariable(
                    "M20_NAV_PYTHON",
                    default_value="/home/orin/venv/m20_nav/bin/python",
                ),
            ),
            localization,
            nav2_group,
            pct_planner_group,
            ego_local_planner_group,
            external_nav_group,
        ]
    )
