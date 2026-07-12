from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    source_odom_topic = LaunchConfiguration("source_odom_topic")
    input_cloud_topic = LaunchConfiguration("input_cloud_topic")
    ego_odom_topic = LaunchConfiguration("ego_odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    grid_map_frame = LaunchConfiguration("grid_map_frame")
    ego_goal_topic = LaunchConfiguration("ego_goal_topic")
    bspline_topic = LaunchConfiguration("bspline_topic")
    output_path_topic = LaunchConfiguration("output_path_topic")
    output_path_frame = LaunchConfiguration("output_path_frame")
    bspline_source_frame = LaunchConfiguration("bspline_source_frame")

    ego_odom_bridge = Node(
        package="m20_fastlio_nav",
        executable="ego_odom_bridge",
        name="ego_odom_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "source_odom_topic": source_odom_topic,
                "output_odom_topic": ego_odom_topic,
                "output_frame": "map",
                "base_frame": "base_link",
            }
        ],
    )

    ego_cloud_bridge = Node(
        package="m20_fastlio_nav",
        executable="ego_cloud_frame_bridge",
        name="ego_cloud_frame_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_cloud_topic": input_cloud_topic,
                "output_cloud_topic": cloud_topic,
                "output_frame": "map",
                "base_frame": "base_link",
                "body_frame": "body",
                "input_frame_override": "body",
                "max_points": LaunchConfiguration("cloud_max_points"),
                "self_filter_radius": LaunchConfiguration("cloud_self_filter_radius"),
                "self_filter_z_min": LaunchConfiguration("cloud_self_filter_z_min"),
                "self_filter_z_max": LaunchConfiguration("cloud_self_filter_z_max"),
            }
        ],
    )

    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ego_planner"), "launch", "advanced_param.launch.py"])
        ),
        launch_arguments={
            "drone_id": "0",
            "use_sim_time": use_sim_time,
            "odometry_topic": ego_odom_topic,
            "cloud_topic": cloud_topic,
            "camera_pose_topic": "/unused_ego_camera_pose",
            "grid_map_frame": grid_map_frame,
            "grid_map_pose_type": "2",
            "ground_height": LaunchConfiguration("ground_height"),
            "goal_topic": ego_goal_topic,
            "min_goal_z": LaunchConfiguration("min_goal_z"),
            "map_size_x_": LaunchConfiguration("map_size_x"),
            "map_size_y_": LaunchConfiguration("map_size_y"),
            "map_size_z_": LaunchConfiguration("map_size_z"),
            "max_vel": LaunchConfiguration("max_vel"),
            "max_acc": LaunchConfiguration("max_acc"),
            "planning_horizon": LaunchConfiguration("planning_horizon"),
            "flight_type": "1",
            "point_num": "1",
            "enable_ground_mode": "true",
            "xy_extend": LaunchConfiguration("xy_extend"),
            "z_extend": LaunchConfiguration("z_extend"),
            "z_penalty_weight": LaunchConfiguration("z_penalty_weight"),
            "xy_gradient_weight": LaunchConfiguration("xy_gradient_weight"),
        }.items(),
    )

    bspline_to_path = Node(
        package="move",
        executable="ego_bspline_to_path",
        name="ego_bspline_to_path",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "bspline_topic": bspline_topic,
                "output_path_topic": output_path_topic,
                "output_frame": output_path_frame,
                "source_frame": bspline_source_frame,
                "sample_dt": LaunchConfiguration("sample_dt"),
                "max_horizon_sec": LaunchConfiguration("path_horizon_sec"),
                "path_timeout": LaunchConfiguration("path_timeout"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("source_odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("input_cloud_topic", default_value="/cloud_registered_body_1"),
            DeclareLaunchArgument("ego_odom_topic", default_value="/ego_odom"),
            DeclareLaunchArgument("cloud_topic", default_value="/ego_cloud"),
            DeclareLaunchArgument("grid_map_frame", default_value="map"),
            DeclareLaunchArgument("ground_height", default_value="-1.5"),
            DeclareLaunchArgument("ego_goal_topic", default_value="/ego_goal_pose"),
            DeclareLaunchArgument("min_goal_z", default_value="-100.0"),
            DeclareLaunchArgument("bspline_topic", default_value="/drone_0_planning/bspline"),
            DeclareLaunchArgument("output_path_topic", default_value="/ego_local_path"),
            DeclareLaunchArgument("output_path_frame", default_value="odom_body"),
            DeclareLaunchArgument("bspline_source_frame", default_value="map"),
            DeclareLaunchArgument("cloud_max_points", default_value="25000"),
            DeclareLaunchArgument("cloud_self_filter_radius", default_value="0.45"),
            DeclareLaunchArgument("cloud_self_filter_z_min", default_value="-0.70"),
            DeclareLaunchArgument("cloud_self_filter_z_max", default_value="0.90"),
            DeclareLaunchArgument("map_size_x", default_value="16.0"),
            DeclareLaunchArgument("map_size_y", default_value="16.0"),
            DeclareLaunchArgument("map_size_z", default_value="3.0"),
            DeclareLaunchArgument("max_vel", default_value="0.6"),
            DeclareLaunchArgument("max_acc", default_value="0.9"),
            DeclareLaunchArgument("planning_horizon", default_value="5.5"),
            DeclareLaunchArgument("xy_extend", default_value="3"),
            DeclareLaunchArgument("z_extend", default_value="1"),
            DeclareLaunchArgument("z_penalty_weight", default_value="1.5"),
            DeclareLaunchArgument("xy_gradient_weight", default_value="0.8"),
            DeclareLaunchArgument("sample_dt", default_value="0.10"),
            DeclareLaunchArgument("path_horizon_sec", default_value="3.0"),
            DeclareLaunchArgument("path_timeout", default_value="1.0"),
            ego_odom_bridge,
            ego_cloud_bridge,
            TimerAction(period=1.0, actions=[ego_planner]),
            TimerAction(period=2.0, actions=[bspline_to_path]),
        ]
    )
