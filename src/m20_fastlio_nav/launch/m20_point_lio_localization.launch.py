from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    map_pcd = LaunchConfiguration("map_pcd")
    raw_cloud_topic = LaunchConfiguration("raw_cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    localization_cloud_topic = LaunchConfiguration("localization_cloud_topic")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    enable_lidar_localizer = LaunchConfiguration("enable_lidar_localizer")
    use_imu_as_input = LaunchConfiguration("use_imu_as_input")
    time_lag_imu_to_lidar = LaunchConfiguration("time_lag_imu_to_lidar")
    rviz = LaunchConfiguration("rviz")

    point_lio_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "point_lio_mid360_m20.yaml"]
    )
    open3d_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "open3d_localization_m20.yaml"]
    )
    open3d_lib_path = "/home/orin/drivers/Open3D/install/lib"

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
        name="m20_pointlio_base_to_livox_frame",
        arguments=BASE_TO_SENSOR + ["base_link", "livox_frame"],
    )
    base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_pointlio_base_to_imu_link",
        arguments=BASE_TO_SENSOR + ["base_link", "imu_link"],
    )
    base_to_motion = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_pointlio_base_to_motion_link",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "motion_link"],
    )
    identity_map_to_odom_nav = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="m20_pointlio_identity_map_to_odom_nav",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom_nav"],
        condition=UnlessCondition(enable_lidar_localizer),
    )

    point_lio_odom = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="pointlio_odom",
        output="screen",
        parameters=[
            point_lio_config,
            {
                "use_sim_time": use_sim_time,
                "common.lid_topic": raw_cloud_topic,
                "common.imu_topic": imu_topic,
                "common.time_lag_imu_to_lidar": time_lag_imu_to_lidar,
                "use_imu_as_input": use_imu_as_input,
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                # Turning used to drift before Open3D could pull the pose back.
                # Keep slightly denser Point-LIO matching than the upstream MID360
                # default, but do not make it as heavy as the old Fast-LIO stack.
                "point_filter_num": 2,
                "space_down_sample": True,
                "filter_size_surf": 0.4,
                "filter_size_map": 0.4,
                "cube_side_length": 1000.0,
                "runtime_pos_log_enable": False,
                "odom_only": False,
                "publish.path_en": False,
                "publish.scan_publish_en": True,
                "publish.scan_bodyframe_pub_en": True,
                "odom_header_frame_id": "odom",
                "odom_child_frame_id": "body",
            },
        ],
    )

    odom_bridge = Node(
        package="m20_fastlio_nav",
        executable="fastlio_odom_bridge",
        name="pointlio_odom_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "source_odom_topic": "/aft_mapped_to_init",
                "output_odom_topic": output_odom_topic,
                "map_frame": "map",
                "odom_frame": "odom",
                "nav_odom_frame": "odom_nav",
                "base_frame": "base_link",
                "nav_base_frame": "base_footprint",
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
                "smooth_map_to_odom_tf": True,
                "map_to_odom_smoothing_alpha": 0.12,
                "map_to_odom_max_translation_step": 0.025,
                "map_to_odom_max_yaw_step_deg": 0.25,
                "map_to_odom_snap_translation_threshold": 1.0,
                "map_to_odom_snap_yaw_threshold_deg": 12.0,
                # Point-LIO local odom is 20 Hz, but can have high-frequency yaw
                # jitter in turns. Smooth only the 2D Nav2/RViz pose output; keep
                # the raw 3D odom->base_link TF untouched for Open3D and debugging.
                "smooth_nav_pose": True,
                "nav_pose_smoothing_alpha": 0.45,
                "nav_pose_max_translation_step": 0.10,
                "nav_pose_max_yaw_step_deg": 3.0,
                "nav_pose_snap_translation_threshold": 0.7,
                "nav_pose_snap_yaw_threshold_deg": 18.0,
            }
        ],
    )

    open3d_loc = Node(
        package="open3d_loc",
        executable="global_localization_node",
        name="global_localization_node",
        output="screen",
        remappings=[
            ("/Odometry_loc", "/aft_mapped_to_init"),
            ("/cloud_registered_1", localization_cloud_topic),
            ("/map", "/map_3d"),
            ("/scan", "/scan_3d"),
        ],
        parameters=[
            open3d_config,
            {
                "use_sim_time": use_sim_time,
                "path_map": map_pcd,
                "fastlio_reset_service": "/pointlio_odom/reset_localization",
            },
        ],
        condition=IfCondition(enable_lidar_localizer),
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointlio_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", scan_cloud_topic),
            ("scan", scan_topic),
        ],
        parameters=[
            {
                "target_frame": "base_footprint",
                "transform_tolerance": 0.05,
                "min_height": -0.3,
                "max_height": 0.35,
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
        name="pointlio_rviz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("m20_fastlio_nav"), "config", "m20_nav3d.rviz"]),
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_livox", default_value="false"),
            DeclareLaunchArgument(
                "map_pcd",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd",
            ),
            DeclareLaunchArgument("raw_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("localization_cloud_topic", default_value="/cloud_registered"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("enable_lidar_localizer", default_value="true"),
            DeclareLaunchArgument("use_imu_as_input", default_value="false"),
            DeclareLaunchArgument("time_lag_imu_to_lidar", default_value="0.0"),
            DeclareLaunchArgument("rviz", default_value="false"),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [open3d_lib_path, ":", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
            ),
            livox_driver,
            base_to_livox,
            base_to_imu,
            base_to_motion,
            identity_map_to_odom_nav,
            point_lio_odom,
            odom_bridge,
            open3d_loc,
            pointcloud_to_scan,
            rviz_node,
        ]
    )
