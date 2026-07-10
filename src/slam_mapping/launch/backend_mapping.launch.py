import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_mapping')
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for bag playback.'
    )
    
    map_save_dir_arg = DeclareLaunchArgument(
        'map_save_dir',
        default_value=os.path.expanduser('~/Desktop/catkin_fastlio_slam/data/'),
        description='Directory to save map and SC database'
    )
    
    parameters = [
        {'scan_line': 16},
        {'mapping_skip_frame': 1},
        {'minimum_range': 0.1},
        {'mapping_line_resolution': 0.2},
        {'mapping_plane_resolution': 0.4},
        {'mapviz_filter_size': 0.1},
        
        # SC-A-LOAM 关键帧参数
        {'keyframe_meter_gap': 1.0},
        {'keyframe_deg_gap': 10.0},
        
        # Scan Context 回环粗匹配参数 (为演示放宽)
        {'sc_dist_thres': 0.4},       # 默认 0.4，放大阈值，让它更容易认出老地方
        {'sc_max_radius': 40.0},       # 稍微扩大 SC 描述子的感知范围
        
        # ICP 回环精匹配与图优化参数 (为演示放宽)
        {'historyKeyframeSearchRadius': 20.0},
        {'historyKeyframeSearchTimeDiff': 60.0},
        {'historyKeyframeSearchNum': 40},
        {'speedFactor': 1.0},
        {'loopClosureFrequency': 2.0},
        {'graphUpdateFrequency': 2.0},
        {'graphUpdateTimes': 5},
        {'loopNoiseScore': 0.1},
        {'vizmapFrequency': 0.5},  
        {'loopFitnessScoreThreshold': 0.6},     # 默认 0.3 极其严格，改为 0.6 确保回环必被接受
        {'lidar_type': 'VLP16'},  
        # save directory
        {'save_directory': LaunchConfiguration('map_save_dir')},
    ]
    
    alaserPGO_node = Node(
        package='slam_mapping',
        executable='alaserPGO',
        name='alaserPGO',
        output='screen',
        additional_env={},
        parameters=parameters + [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/velodyne_points', '/velodyne_points'),
            ('/aft_mapped_to_init', '/Odometry'),
            # 【关键修改】让 ScanContext 正确接收 FAST-LIO 发出的雷达自身坐标系下的点云
            ('/cloud_for_scancontext', '/cloud_registered_body'),
            ('/velodyne_cloud_registered_local', '/cloud_registered_body'),
        ]
    )
    
    rviz_config_file = os.path.join(pkg_share, 'rviz_cfg', 'mapping.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_sc_pgo',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        map_save_dir_arg,
        alaserPGO_node,
        rviz_node,
    ])
