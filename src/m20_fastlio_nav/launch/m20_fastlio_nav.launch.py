from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_livox = LaunchConfiguration("start_livox")
    map_pcd = LaunchConfiguration("map_pcd")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_external_nav = LaunchConfiguration("start_external_nav")
    start_pct_planner = LaunchConfiguration("start_pct_planner")
    start_body_scan = LaunchConfiguration("start_body_scan")
    pct_planner_root = LaunchConfiguration("pct_planner_root")
    pct_tomogram_file = LaunchConfiguration("pct_tomogram_file")
    pct_tomogram_dir = LaunchConfiguration("pct_tomogram_dir")
    pct_start_source = LaunchConfiguration("pct_start_source")
    pct_start_z_offset = LaunchConfiguration("pct_start_z_offset")
    pct_goal_z_offset = LaunchConfiguration("pct_goal_z_offset")
    pct_goal_topic = LaunchConfiguration("pct_goal_topic")
    body_scan_min_height = LaunchConfiguration("body_scan_min_height")
    body_scan_max_height = LaunchConfiguration("body_scan_max_height")
    rl_python_executable = LaunchConfiguration("rl_python_executable")

    stair_body_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("m20_fastlio_nav"), "launch", "m20_fastlio_stair_body_nav.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "start_livox": start_livox,
            "map_pcd": map_pcd,
            "params_file": params_file,
            "rviz": rviz,
            "start_nav2": start_nav2,
            "start_external_nav": start_external_nav,
            "start_pct_planner": start_pct_planner,
            "start_pct_adapter": "false",
            "start_body_scan": start_body_scan,
            "pct_planner_root": pct_planner_root,
            "pct_tomogram_file": pct_tomogram_file,
            "pct_tomogram_dir": pct_tomogram_dir,
            "pct_start_source": pct_start_source,
            "pct_start_z_offset": pct_start_z_offset,
            "pct_goal_z_offset": pct_goal_z_offset,
            "pct_goal_topic": pct_goal_topic,
            "body_scan_min_height": body_scan_min_height,
            "body_scan_max_height": body_scan_max_height,
            "rl_python_executable": rl_python_executable,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_livox", default_value="false"),
            DeclareLaunchArgument(
                "map_pcd",
                default_value="/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("m20_fastlio_nav"), "config", "nav2_dwb_body_plane.yaml"]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_external_nav", default_value="true"),
            DeclareLaunchArgument("start_pct_planner", default_value="true"),
            DeclareLaunchArgument("start_body_scan", default_value="true"),
            DeclareLaunchArgument(
                "pct_planner_root",
                default_value="/mnt/nvme/workspace/fast_lio_ws/third_party/global_path_planning",
            ),
            DeclareLaunchArgument("pct_tomogram_file", default_value="output"),
            DeclareLaunchArgument("pct_tomogram_dir", default_value="/rsc/tomogram/"),
            DeclareLaunchArgument("pct_start_source", default_value="tf"),
            DeclareLaunchArgument("pct_start_z_offset", default_value="0.0"),
            DeclareLaunchArgument("pct_goal_z_offset", default_value="0.0"),
            DeclareLaunchArgument("pct_goal_topic", default_value="/goal_3d"),
            DeclareLaunchArgument("body_scan_min_height", default_value="-0.10"),
            DeclareLaunchArgument("body_scan_max_height", default_value="0.65"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venv", "m20_nav", "bin", "python"]
                ),
            ),
            stair_body_nav,
        ]
    )
