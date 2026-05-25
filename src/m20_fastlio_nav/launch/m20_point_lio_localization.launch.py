from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


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
    normalized_cloud_topic = LaunchConfiguration("normalized_cloud_topic")
    normalized_imu_topic = LaunchConfiguration("normalized_imu_topic")
    twist_topic = LaunchConfiguration("twist_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    enable_lidar_localizer = LaunchConfiguration("enable_lidar_localizer")
    use_imu_as_input = LaunchConfiguration("use_imu_as_input")
    rviz = LaunchConfiguration("rviz")

    point_lio_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "point_lio_mid360_m20.yaml"]
    )
    localization_config = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "lidar_localization_m20.yaml"]
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

    stamp_republisher = Node(
        package="m20_fastlio_nav",
        executable="stamp_republisher",
        name="pointlio_stamp_republisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cloud_in_topic": raw_cloud_topic,
                "cloud_out_topic": normalized_cloud_topic,
                "imu_in_topic": imu_topic,
                "imu_out_topic": normalized_imu_topic,
            }
        ],
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
                "common.lid_topic": normalized_cloud_topic,
                "common.imu_topic": normalized_imu_topic,
                "use_imu_as_input": use_imu_as_input,
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                "point_filter_num": 3,
                "space_down_sample": True,
                "filter_size_surf": 0.5,
                "filter_size_map": 0.5,
                "cube_side_length": 1000.0,
                "runtime_pos_log_enable": False,
                "odom_only": True,
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
                "source_odom_topic": "/odom_corrected",
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
            }
        ],
    )

    lidar_localization = LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        namespace="",
        output="screen",
        parameters=[
            localization_config,
            {
                "use_sim_time": use_sim_time,
                "map_path": map_pcd,
                "enable_map_odom_tf": True,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
            },
        ],
        remappings=[
            ("/cloud", normalized_cloud_topic),
            ("/imu", normalized_imu_topic),
            ("/twist", twist_topic),
            ("/pcl_pose", "/localization/pose_with_covariance"),
        ],
        condition=IfCondition(enable_lidar_localizer),
    )

    activate_localization = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_localization),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(enable_lidar_localizer),
    )

    configure_localization = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_localization),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
        condition=IfCondition(enable_lidar_localizer),
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointlio_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", normalized_cloud_topic),
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
            DeclareLaunchArgument("normalized_cloud_topic", default_value="/livox/lidar_stamped"),
            DeclareLaunchArgument("normalized_imu_topic", default_value="/livox/imu_stamped"),
            DeclareLaunchArgument("twist_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("enable_lidar_localizer", default_value="true"),
            DeclareLaunchArgument("use_imu_as_input", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            livox_driver,
            base_to_livox,
            base_to_imu,
            base_to_motion,
            stamp_republisher,
            point_lio_odom,
            odom_bridge,
            lidar_localization,
            activate_localization,
            configure_localization,
            pointcloud_to_scan,
            rviz_node,
        ]
    )
