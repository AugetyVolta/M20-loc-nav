from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([

        # MID360: base_link -> livox_frame
        # 注意：这里的 livox_frame 必须和 /livox/imu 的 header.frame_id 一致
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_tf',
            arguments=[
                '0.32713234', '0.01413551', '0.31238696',
                '-0.00394028', '0.24367785', '0.00970223', '0.96979969',
                'base_link',
                'livox_frame'
            ]
        ),
        
        # 前雷达: base_link -> rslidar_front/pear
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='front_lidar_tf',
        #     arguments=[
        #         '0.32028', '0', '-0.013',
        #         '0', '-1.57079', '-3.14159',
        #         'base_link',
        #         'rslidar_front'
        #     ]
        # ),

        # # 后雷达: base_link -> rslidar_rear/pear
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='rear_lidar_tf',
        #     arguments=[
        #         '-0.32028', '0', '-0.013',
        #         '0', '-1.57079', '0',
        #         'base_link',
        #         'rslidar_rear'
        #     ]
        # ),

        # 使用 MID360 IMU + 机器狗 MotionInfo 的 odom
        # Node(
        #     package='m20',
        #     executable='simple_odom',
        #     name='simple_odom_node',
        #     output='screen',
        #     parameters=[{
        #         # 机器狗本体速度
        #         'motion_topic': '/MOTION_INFO',

        #         # MID360 IMU
        #         'imu_topic': '/livox/imu',

        #         # odom 输出
        #         'odom_topic': '/odom',
        #         'odom_frame': 'odom',
        #         'base_frame': 'base_link',

        #         # MID360 IMU 的 frame
        #         'imu_frame': 'livox_frame',

        #         # 使用 MID360 角速度，不用 orientation
        #         'use_imu_angular_velocity': True,
        #         'use_imu_orientation': False,

        #         # 把 livox_frame 下的角速度转到 base_link 下
        #         'transform_imu_to_base': True,

        #         # 启动后保持机器狗不动 3 秒，用于标定 gyro 零偏
        #         'gyro_bias_calib_time': 3.0,

        #         # MID360 gyro 标定后的角速度死区
        #         'gyro_deadband': 0.015,

        #         # 如果现实左转，RViz 里右转，就改成 -1.0
        #         'gyro_z_sign': 1.0,

        #         # 新增：压掉 /MOTION_INFO 静止速度偏置
        #         # 你静止时 vel_x≈0.005，vel_y≈-0.0037，所以 0.015 比较合适
        #         'linear_deadband': 0.015,

        #         # 新增：压掉 /MOTION_INFO 静止 yaw 速度偏置
        #         'angular_deadband': 0.01,

        #         # 新增：判定静止时不积分 x/y，避免 odom 原地漂
        #         'stationary_linear_threshold': 0.02,
        #         'stationary_angular_threshold': 0.015,

        #         'publish_rate': 50.0,
        #         'publish_tf': True,
        #         'imu_timeout': 0.2,
        #         'tf_warn_interval': 2.0,
        #     }]
        # ),
        
        Node(
            package='m20',
            executable='simple_odom_old',
            name='simple_odom_node',
            output='screen',
        ),
        # MID360 点云转 LaserScan
        ComposableNodeContainer(
            name='pointcloud_to_laserscan_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='pointcloud_to_laserscan',
                    plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                    name='pointcloud_to_laserscan',
                    remappings=[
                        ('cloud_in', '/livox/lidar'),
                        ('scan', '/scan')
                    ],
                    parameters=[{
                        'target_frame': 'base_link',

                        # 收窄高度，只取接近障碍物高度的点
                        'min_height': -0.3,
                        'max_height': 0.35,

                        'angle_min': -3.14,
                        'angle_max': 3.14,

                        # 原来 0.0087 约 720 beams
                        # 先改成 1 度，约 360 beams
                        'angle_increment': 0.01745,

                        # 过滤掉狗身附近点
                        'range_min': 0.35,

                        # 不要先看 30m，局部避障 8~10m 足够
                        'range_max': 10.0,

                        'transform_tolerance': 0.05,
                        'use_inf': True,
                    }]
                    # parameters=[{
                    #     'target_frame': 'base_link',

                    #     # 收窄高度，只取接近障碍物高度的点
                    #     'min_height': -0.2,
                    #     'max_height': 0.35,

                    #     'angle_min': -3.14,
                    #     'angle_max': 3.14,

                    #     # 原来 0.0087 约 720 beams
                    #     # 先改成 1 度，约 360 beams
                    #     'angle_increment': 0.01745,

                    #     # 过滤掉狗身附近点
                    #     'range_min': 0.6,

                    #     # 不要先看 30m，局部避障 8~10m 足够
                    #     'range_max': 10.0,

                    #     'transform_tolerance': 0.05,
                    #     'use_inf': True,
                    # }]
                    # parameters=[{
                    #     'target_frame': 'base_link',

                    #     # 如果 /scan 里有狗身/地面杂点，后续可以收窄到 -0.2 ~ 0.3
                    #     'min_height': -0.5,
                    #     'max_height': 0.5,

                    #     'angle_min': -3.14,
                    #     'angle_max': 3.14,
                    #     'angle_increment': 0.0087,

                    #     # 如果狗身附近有杂点，可以调到 0.5 或 0.6
                    #     'range_min': 0.35,
                    #     'range_max': 20.0,

                    #     'transform_tolerance': 0.02,
                    #     'use_inf': True,
                    # }]
                )
            ]
        )
    ])