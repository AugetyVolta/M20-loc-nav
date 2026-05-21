from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BASE_TO_LIVOX = [
    "0.32713234",
    "0.01413551",
    "0.31238696",
    "-0.00394028",
    "0.24367785",
    "0.00970223",
    "0.96979969",
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    rviz = LaunchConfiguration("rviz")
    map_pcd = LaunchConfiguration("map_pcd")

    config_file = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "fastlio_mapping_mid360.yaml"]
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch_ROS2", "msg_MID360_launch.py"]
            )
        ),
        condition=IfCondition(start_livox),
    )

    base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_base_to_livox_frame",
        arguments=BASE_TO_LIVOX + ["base_link", "livox_frame"],
    )

    fast_lio_mapping = Node(
        package="fast_lio_map",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            config_file,
            {
                "map_file_path": map_pcd,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="fastlio_mapping_rviz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("fast_lio_map"), "rviz", "fastlio.rviz"]),
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_livox",
                default_value="false",
                description="Start the Livox MID360 driver from this launch file.",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "map_pcd",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd",
            ),
            livox_driver,
            base_to_livox,
            fast_lio_mapping,
            rviz_node,
        ]
    )
