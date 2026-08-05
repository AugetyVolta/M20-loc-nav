from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("pct_planner_ros2")
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "m20_tomogram_editor.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pct_root",
                default_value=EnvironmentVariable(
                    "PCT_PLANNER_ROOT",
                    default_value=PathJoinSubstitution([package_share, "PCT_planner"]),
                ),
            ),
            DeclareLaunchArgument("tomogram_file", default_value="m20_3d_map"),
            DeclareLaunchArgument("output_tomogram_name", default_value=""),
            DeclareLaunchArgument(
                "pcd_file",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd",
            ),
            DeclareLaunchArgument("wall_width", default_value="0.30"),
            DeclareLaunchArgument("inflation_radius", default_value="0.50"),
            DeclareLaunchArgument("cost_scaling_factor", default_value="5.0"),
            DeclareLaunchArgument("barrier_cost", default_value="50.0"),
            DeclareLaunchArgument("layer_height_tolerance", default_value="0.75"),
            DeclareLaunchArgument("surface_snap_radius", default_value="0.50"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            Node(
                package="pct_planner_ros2",
                executable="pct_map_viz_node",
                name="pct_tomogram_editor_pcd",
                output="screen",
                parameters=[
                    {
                        "pct_root": LaunchConfiguration("pct_root"),
                        "pcd_file": LaunchConfiguration("pcd_file"),
                        "publish_pcd": True,
                        "publish_tomogram": False,
                    }
                ],
            ),
            Node(
                package="pct_planner_ros2",
                executable="pct_tomogram_editor",
                name="pct_tomogram_editor",
                output="screen",
                parameters=[
                    {
                        "pct_root": LaunchConfiguration("pct_root"),
                        "tomogram_file": LaunchConfiguration("tomogram_file"),
                        "output_tomogram_name": LaunchConfiguration("output_tomogram_name"),
                        "wall_width": ParameterValue(
                            LaunchConfiguration("wall_width"), value_type=float
                        ),
                        "inflation_radius": ParameterValue(
                            LaunchConfiguration("inflation_radius"), value_type=float
                        ),
                        "cost_scaling_factor": ParameterValue(
                            LaunchConfiguration("cost_scaling_factor"), value_type=float
                        ),
                        "barrier_cost": ParameterValue(
                            LaunchConfiguration("barrier_cost"), value_type=float
                        ),
                        "layer_height_tolerance": ParameterValue(
                            LaunchConfiguration("layer_height_tolerance"), value_type=float
                        ),
                        "surface_snap_radius": ParameterValue(
                            LaunchConfiguration("surface_snap_radius"), value_type=float
                        ),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
        ]
    )
