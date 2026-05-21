from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pcd_file = LaunchConfiguration("pcd_file")
    output_map = LaunchConfiguration("output_map")
    params_file = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "pcd2pgm_m20.yaml"]
    )

    pcd2pgm = Node(
        package="pcd2pgm",
        executable="pcd2pgm_node",
        name="pcd2pgm",
        output="screen",
        parameters=[
            params_file,
            {
                "pcd_file": pcd_file,
            },
        ],
    )

    save_map = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            output_map,
            "--ros-args",
            "-p",
            "map_subscribe_transient_local:=true",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcd_file",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd",
            ),
            DeclareLaunchArgument(
                "output_map",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map",
            ),
            pcd2pgm,
            TimerAction(period=3.0, actions=[save_map]),
        ]
    )
