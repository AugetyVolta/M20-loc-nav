from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pcd_file = LaunchConfiguration("pcd_file")
    output_map = LaunchConfiguration("output_map")
    output_pcd = LaunchConfiguration("output_pcd")
    level_pcd_python = LaunchConfiguration("level_pcd_python")
    params_file = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "config", "pcd2pgm_m20.yaml"]
    )
    level_pcd_script = PathJoinSubstitution(
        [FindPackageShare("m20_fastlio_nav"), "scripts", "level_pcd.py"]
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

    save_leveled_pcd = ExecuteProcess(
        cmd=[
            level_pcd_python,
            level_pcd_script,
            "--input",
            pcd_file,
            "--output",
            output_pcd,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcd_file",
                default_value="/mnt/nvme/workspace/fast_lio_ws_stable_localization/maps/fastlio/global_map.pcd",
            ),
            DeclareLaunchArgument(
                "output_map",
                default_value="/mnt/nvme/workspace/fast_lio_ws_stable_localization/maps/fastlio/m20_2d_map",
                description="Output prefix for Nav2 2D map .pgm/.yaml files.",
            ),
            DeclareLaunchArgument(
                "output_pcd",
                default_value="/mnt/nvme/workspace/fast_lio_ws_stable_localization/maps/fastlio/m20_map_leveled.pcd",
                description="Output path for the leveled 3D PCD used by Open3D localization.",
            ),
            DeclareLaunchArgument(
                "level_pcd_python",
                default_value="/home/orin/venv/m20_nav/bin/python",
                description="Python executable with open3d/scipy for level_pcd.py.",
            ),
            pcd2pgm,
            save_leveled_pcd,
            TimerAction(period=8.0, actions=[save_map]),
        ]
    )
