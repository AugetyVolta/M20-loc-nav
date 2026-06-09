# M20 MID360 3D 导航

当前主线分支：`feature/3d-body-plane-nav`

当前方案使用仓库内置的 PCT planner 输出 3D 全局路径，本仓库负责 Fast-LIO/Open3D 定位、PCT 全局规划、机体平面局部控制和底盘输出。Jie/OctoMap 全局规划方案已单独归档到分支 `archive/jie-octomap-global-planner`，归档提交为 `2907d0b`。

## 总体链路

```text
MID360 点云/IMU
  -> Fast-LIO 建图或定位
  -> Open3D 全局定位提供 map -> odom
  -> fastlio_odom_bridge 输出 /odom_body，并发布 map -> odom_body 与 odom -> base_link
  -> pointcloud_to_laserscan 输出 /scan
  -> pct_planner_ros2 输出 /pct_path
  -> pure_pursuit 在 3D 路径上按 1.8m 前视距离发布 /subgoal
  -> priest_rl_publisher_nav_cmd_fast 根据 /subgoal 和 /scan 发布 /local_path
  -> DWB 跟踪 /local_path 输出 /cmd_vel
  -> adapter 输出 /NAV_CMD
```

关键话题：

| 话题 | 说明 |
|---|---|
| `/Odometry_loc` | Fast-LIO 定位原始输出 |
| `/odom_body` | 局部控制使用的机体平面 odom |
| `/scan` | local/global costmap 和 RL local path 共用的 LaserScan |
| `/pct_path` | PCT planner 发布的 3D 全局路径 |
| `/subgoal` | pure pursuit 从 `/pct_path` 选出的 `base_link` 局部目标 |
| `/local_path` | RL local path 输出给 DWB 的局部路径 |
| `/cmd_vel` | DWB 输出 |
| `/NAV_CMD` | 底盘控制输出 |

关键 frame：

| Frame | 说明 |
|---|---|
| `map` | 3D 地图、Open3D 全局定位、PCT 全局路径 |
| `odom` | Fast-LIO 局部里程计 |
| `odom_body` | 局部控制平面，跟随机器狗当前机体高度和平面姿态 |
| `base_link` | 机器狗机体 |
| `livox_frame` | MID360 雷达 |

## 编译本仓库

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

colcon build --symlink-install --packages-select pct_planner_ros2 move m20_fastlio_nav
source ./source_m20_nav.sh
```

PCT planner 代码在本仓库内：

```text
src/pct_planner_ros2
src/pct_planner_ros2/PCT_planner
```

Jie/OctoMap planner 不在当前主线链路里，只保留在归档分支。

## 统一加载环境

每个新终端先执行一次：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
```

这个脚本会加载：

```text
/opt/ros/humble/setup.bash
~/liv_ws/install/setup.bash
当前 workspace 的 install/setup.bash
仓库内置 PCT planner 的 Python/LD_LIBRARY_PATH
MID360 需要的 libusb LD_PRELOAD
```

常用变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `M20_FASTLIO_WS` | `/mnt/nvme/workspace/fast_lio_ws` | 当前工作区 |
| `M20_MAP_PCD` | `maps/fastlio/m20_3d_map.pcd` | 默认导航 PCD |
| `PCT_PLANNER_ROOT` | `src/pct_planner_ros2/PCT_planner` | 仓库内置 PCT core |
| `M20_NAV_CUPY_PYTHON` | `/home/orin/venv/m20_nav_cupy/bin/python` | PCD/PCT 处理 Python |
| `M20_NAV_PYTHON` | `/home/orin/venv/m20_nav/bin/python` | RL 本地路径 Python |

## 建 3D PCD 地图

导航默认使用：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd
```

实车建图：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
mkdir -p maps/fastlio

ros2 launch m20_fastlio_nav m20_fastlio_mapping.launch.py \
  start_livox:=true \
  rviz:=true
```

Bag 建图：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
mkdir -p maps/fastlio

ros2 launch m20_fastlio_nav m20_fastlio_mapping.launch.py \
  use_sim_time:=true \
  start_livox:=false \
  rviz:=true
```

另一个终端播放 bag：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

ros2 bag play bags/stair1 --clock
```

保存地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
ros2 service call /save_pgo_map std_srvs/srv/Trigger {}
```

## 准备导航 PCD

默认脚本优先读取 `maps/fastlio/global_map.pcd`，没有它时读取 `maps/fastlio/m20_map_raw.pcd`，输出 `maps/fastlio/m20_3d_map.pcd`。

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

"${M20_NAV_CUPY_PYTHON}" \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py
```

如果确认地面识别正确，再使用 `--ground-zero` 把地图高度归零：

```bash
"${M20_NAV_CUPY_PYTHON}" \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py --ground-zero
```

先试运行到临时文件：

```bash
"${M20_NAV_CUPY_PYTHON}" \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --ground-zero \
  --output /tmp/m20_ground_zero_check.pcd
```

如果脚本提示 ground plane near vertical，说明当前点云姿态或地面选择不可靠，不要强行覆盖正式地图。

## 启动 PCT Planner

PCT planner 已放进本仓库，先 source 本仓库环境：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
```

生成或查看 tomogram：

```bash
ros2 launch pct_planner_ros2 m20_tomography_rviz.launch.py
```

启动 PCT 全局规划，带 PCT 自己的 RViz：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py
```

只启动 planner，不启动 PCT RViz：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py launch_rviz:=false
```

如果要让 PCT 起点来自当前定位：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  start_source:=odom \
  odom_topic:=/odom_body
```

PCT 规划成功后应发布：

```bash
ros2 topic echo /pct_path --once
```

## 启动 M20 3D 导航

Bag 或仿真时间：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

实车：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  start_livox:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

默认参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `global_path_topic` | `/pct_path` | pure pursuit 和 RL local path 使用的全局路径 |
| `start_pct_planner` | `true` | 默认随主导航启动 PCT planner |
| `pct_start_source` | `tf` | PCT 起点默认来自 TF `map -> base_link`，会随机器人位置更新 |
| `pct_replan_interval` | `1.0` | 每 1 秒检查是否需要重规划 |
| `pct_position_epsilon` | `0.2` | 起点或终点变化超过 0.2m 才重规划 |
| `scan_topic` | `/scan` | costmap 与 RL 使用同一个 LaserScan |
| `output_odom_topic` | `/odom_body` | DWB、RL 和 adapter 使用的 body-plane odom |
| `start_pure_pursuit` | `true` | 默认启动 pure pursuit |
| `start_rl_local_path` | `true` | 默认启动 RL local path |
| `start_adapter` | `true` | 默认启动 `/local_path -> /NAV_CMD` 适配 |
| `body_scan_min_height` | `0.1` | 从 body 点云生成 scan 的最低高度 |
| `body_scan_max_height` | `0.55` | 从 body 点云生成 scan 的最高高度 |

pure pursuit 的前视距离在 `src/move/move/pure_pursuit.py` 中默认是 `1.8`，启动文件不覆盖这个值。`rl_pp_lookahead=4.0` 只作为 RL 节点在没有 `/subgoal` 时的备用全局路径取点距离。

## 发布终点

主导航默认已经启动 PCT planner，不需要再单独启动 `m20_pct_rviz.launch.py`。PCT 起点默认来自 TF `map -> base_link`，所以全局路径会从机器人真实 `base_link` 位置开始；终点改变或机器人移动超过 `pct_position_epsilon` 后，会重新规划并更新 `/pct_path`。

RViz 里推荐两种方式：

```text
Publish Point 工具：在地图上点一下，发布到 /clicked_point，作为 PCT 终点。
Interact 工具：拖动 PCT Goal Editor 里的 end_pos marker。
```

`Waypoint Editors` 是旧的手工 waypoint 序列工具，对当前 PCT 全局规划链路没有作用，主 RViz 默认已经关闭。

命令行发布终点：

```bash
ros2 topic pub --once /pct_goal_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 0.5}}"
```

如果用 RViz 的 `Nav2 Goal` 工具，也可以发到 `/goal_pose`；PCT planner 会读取 pose 的位置作为终点：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"
```

检查是否出全局路径：

```bash
ros2 topic echo /pct_path --once
```

## 常用检查

定位：

```bash
ros2 topic echo /Odometry_loc --once
ros2 topic echo /odom_body --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom_body base_link
```

点云转 scan：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

全局路径到局部路径：

```bash
ros2 topic echo /pct_path --once
ros2 topic echo /subgoal --once
ros2 topic echo /local_path --once
```

控制输出：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /NAV_CMD --once
```

如果 `/pct_path` 有输出但 `/subgoal` 没有，优先检查 `map -> base_link` TF。

如果 `/subgoal` 有输出但 `/local_path` 没有，优先检查 `/scan`、RL checkpoint 和 Python 环境。
如果 `/local_path` 有输出但机器人不动，检查 DWB lifecycle、`/cmd_vel` 和 `/NAV_CMD`。

## Jie/OctoMap 归档

Jie/OctoMap 方案没有放在当前主线分支里。需要查看当时的 OctoMap 编辑 GUI、Jie planner launch 和相关命令时：

```bash
git switch archive/jie-octomap-global-planner
```

回到当前 PCT 主线：

```bash
git switch feature/3d-body-plane-nav
```
