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

BODY_TO_BASE_LINK = [
    "-0.140800",
    "-0.006440",
    "-0.430040",
    "-0.02353",
    "-0.49227",
    "0.01403",
]

# The Orin deployment used LD_PRELOAD to work around an MVS/libusb conflict.
# Keep it disabled by default on the local x86_64 workstation.
LIBUSB_PRELOAD = {}


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    rviz = LaunchConfiguration("rviz")
    map_pcd = LaunchConfiguration("map_pcd")
    map_save_dir = LaunchConfiguration("map_save_dir")

    frontend_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "fastlio_mapping_mid360.yaml"]
    )
    backend_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "slam_mapping_mid360.yaml"]
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

    body_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_body_to_base_link",
        arguments=BODY_TO_BASE_LINK + ["body", "base_link"],
    )

    fast_lio_mapping = Node(
        package="fast_lio_map",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        additional_env=LIBUSB_PRELOAD,
        parameters=[
            frontend_config,
            {
                "map_file_path": map_pcd,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    slam_mapping = Node(
        package="slam_mapping",
        executable="alaserPGO",
        name="alaserPGO",
        output="screen",
        additional_env=LIBUSB_PRELOAD,
        parameters=[
            backend_config,
            {
                "save_directory": map_save_dir,
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("/aft_mapped_to_init", "/Odometry"),
            ("/cloud_for_scancontext", "/cloud_registered_body"),
            ("/velodyne_cloud_registered_local", "/cloud_registered_body"),
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
            PathJoinSubstitution([FindPackageShare("slam_mapping"), "rviz_cfg", "mapping.rviz"]),
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
                default_value="/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/m20_map.pcd",
            ),
            DeclareLaunchArgument(
                "map_save_dir",
                default_value="/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/",
                description="Directory where slam_mapping writes global_map.pcd and sc_database.txt.",
            ),
            livox_driver,
            base_to_livox,
            body_to_base_link,
            fast_lio_mapping,
            slam_mapping,
            rviz_node,
        ]
    )
