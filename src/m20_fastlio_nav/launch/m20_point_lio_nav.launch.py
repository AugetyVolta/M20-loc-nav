from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    raw_cloud_topic = LaunchConfiguration("raw_cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("m20_fastlio_nav"), "launch", "m20_point_lio_localization.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_livox": start_livox,
            "map_pcd": map_pcd,
            "raw_cloud_topic": raw_cloud_topic,
            "imu_topic": imu_topic,
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
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
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
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
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
            DeclareLaunchArgument("raw_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            localization,
            TimerAction(period=3.0, actions=[map_server, map_lifecycle]),
            TimerAction(period=8.0, actions=[nav2_navigation]),
        ]
    )
