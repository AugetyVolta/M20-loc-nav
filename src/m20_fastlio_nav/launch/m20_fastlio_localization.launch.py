from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


BASE_TO_SENSOR = [
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
    fastlio_frontend = LaunchConfiguration("fastlio_frontend")
    map_pcd = LaunchConfiguration("map_pcd")
    scan_topic = LaunchConfiguration("scan_topic")
    start_scan = LaunchConfiguration("start_scan")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    start_initialpose_3d_marker = LaunchConfiguration("start_initialpose_3d_marker")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    fastlio_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "fastlio_localization_mid360.yaml"]
    )
    open3d_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "open3d_localization_m20.yaml"]
    )

    open3d_lib_path = "/home/ubuntu/xlab/third_party/Open3D/install/lib"

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch_ROS2", "msg_MID360_launch.py"]
            )
        ),
        condition=IfCondition(start_livox),
    )

    odom_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_odom_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "camera_init"],
    )
    base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_base_to_livox_frame",
        arguments=BASE_TO_SENSOR + ["base_link", "livox_frame"],
    )
    base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_base_to_imu_link",
        arguments=BASE_TO_SENSOR + ["base_link", "imu_link"],
    )
    base_to_motion = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_base_to_motion_link",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "motion_link"],
    )

    fast_lio_mapping_frontend = Node(
        package=fastlio_frontend,
        executable="fastlio_mapping",
        name="fastlio_localization_odom",
        output="screen",
        parameters=[
            fastlio_config,
            {
                "map_file_path": map_pcd,
                "use_sim_time": use_sim_time,
                "publish.path_en": False,
                "publish.effect_map_en": False,
                "publish.map_en": False,
                "publish.scan_publish_en": True,
                "publish.dense_publish_en": False,
                "publish.scan_bodyframe_pub_en": True,
                "pcd_save.pcd_save_en": False,
            },
        ],
        remappings=[
            ("/Odometry", "/Odometry_loc"),
            ("/cloud_registered", "/cloud_registered_1"),
            ("/cloud_registered_body", "/cloud_registered_body_1"),
            ("/cloud_effected", "/cloud_effected_1"),
            ("/Laser_map", "/Laser_map_1"),
            ("/path", "/path_1"),
        ],
    )

    odom_bridge = Node(
        package="m20_fastlio_nav",
        executable="fastlio_odom_bridge",
        name="fastlio_odom_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "source_odom_topic": "/Odometry_loc",
                "output_odom_topic": output_odom_topic,
                "map_frame": "map",
                "odom_frame": "odom",
                "nav_odom_frame": "odom_body",
                "base_frame": "base_link",
                "nav_base_frame": "base_link",
                "publish_tf": True,
                "publish_nav_base_tf": False,
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

    open3d_loc = Node(
        package="open3d_loc",
        executable="global_localization_node",
        name="global_localization_node",
        output="screen",
        additional_env={
            "LD_LIBRARY_PATH": [
                open3d_lib_path,
                ":",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ]
        },
        remappings=[
            ("/map", "/map_3d"),
            ("/scan", "/scan_3d"),
        ],
        parameters=[
            open3d_config,
            {
                "path_map": map_pcd,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    initialpose_3d_marker = Node(
        condition=IfCondition(start_initialpose_3d_marker),
        package="m20_fastlio_nav",
        executable="initialpose_3d_marker",
        name="initialpose_3d_marker",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "frame_id": "map",
                "base_frame": "base_link",
                "marker_namespace": "initialpose_3d_marker",
                "publish_topic": "/initialpose",
                "marker_scale": 0.8,
                "sync_from_tf_on_start": False,
            }
        ],
    )

    pointcloud_to_scan = Node(
        condition=IfCondition(start_scan),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="fastlio_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/cloud_registered_body_1"),
            ("scan", scan_topic),
        ],
        parameters=[
            {
                "target_frame": "base_link",
                "transform_tolerance": 0.05,
                "min_height": ParameterValue(scan_min_height, value_type=float),
                "max_height": ParameterValue(scan_max_height, value_type=float),
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="fastlio_localization_rviz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            rviz_config,
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
            DeclareLaunchArgument(
                "fastlio_frontend",
                default_value="fast_lio",
                description="Fast-LIO frontend package for A/B testing: fast_lio_map or fast_lio.",
            ),
            DeclareLaunchArgument(
                "map_pcd",
                default_value="/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/m20_map_leveled.pcd",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("start_scan", default_value="true"),
            DeclareLaunchArgument("scan_min_height", default_value="0.1"),
            DeclareLaunchArgument("scan_max_height", default_value="0.55"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom_body"),
            DeclareLaunchArgument("start_initialpose_3d_marker", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("m20_fastlio_nav"), "config", "m20_nav3d.rviz"]
                ),
            ),
            livox_driver,
            odom_to_camera_init,
            base_to_livox,
            base_to_imu,
            base_to_motion,
            fast_lio_mapping_frontend,
            odom_bridge,
            open3d_loc,
            initialpose_3d_marker,
            pointcloud_to_scan,
            rviz_node,
        ]
    )
