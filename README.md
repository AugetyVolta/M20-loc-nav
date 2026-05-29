# M20 Orin 2D 定位导航工作空间

`stable/slam-toolbox-2d-localization` 分支是 M20 在 Orin 上使用的 2D 定位导航优化版本。它保留原来的 2D 导航链路：MID360 点云转 `/scan`，建图使用
`slam_toolbox`，定位/导航使用 Nav2；和 `stable/fastlio-localization` 分支的 Fast-LIO + Open3D 3D 定位方案不是同一套链路。

常用命令原始记录在：

```text
src/m20/cmd.md
```

本文只是把当前常用启动顺序整理清楚。`src/m20_local_path_bt` 行为树相关包暂不纳入本文档。

本分支只跟踪：

- `README.md`
- `src/`

## 工作区定位

- 当前 Orin 工作区路径：`/mnt/nvme/workspace/m20_ws`
- 兼容路径：`~/workspace/m20_ws` 指向同一个目录
- 底层驱动：`~/liv_ws`
- 基础包：`src/m20`
- 局部路径和 adapter：`src/move`
- RL/PRIEST 依赖：`src/RL2Path`
- 默认 2D 地图：`src/move/map/lab.yaml`

核心链路：

```text
MID360 /livox/lidar
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox 建图 或 Nav2 localization/navigation
  -> /cmd_vel
  -> move adapter
  -> /NAV_CMD
```

`m20_launch.launch.py` 会启动：

- `base_link -> livox_frame` 静态 TF
- `simple_odom_old`，发布旧版 `/odom`
- `pointcloud_to_laserscan`，把 `/livox/lidar` 转成 `/scan`

## 编译

```bash
export M20_WS=/mnt/nvme/workspace/m20_ws
cd $M20_WS
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

每个新终端都先执行：

```bash
export M20_WS=/mnt/nvme/workspace/m20_ws
cd $M20_WS
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
```

如果要运行 RL local path：

```bash
source ~/venv/m20_nav/bin/activate
```

## 启动前准备

如果m20 pro里还在跑旧服务，先停掉：

```bash
sudo systemctl stop planner.service
sudo systemctl stop localization.service
```

同步时间：

```bash
sudo systemctl restart chrony
sudo chronyc makestep
```

## Step 1: 启动 MID360

```bash
cd ~/liv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## Step 2: 启动 M20 基础链路

```bash
cd $M20_WS
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash

ros2 launch m20 m20_launch.launch.py
```

检查：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## Step 3: 用 slam_toolbox 建图

建图时打开 Nav2 RViz：

```bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=False
```

启动 slam_toolbox：

```bash
ros2 launch nav2_bringup slam_launch.py \
  params_file:=$M20_WS/src/m20/config/m20_slam.yaml \
  use_sim_time:=False
```

`m20_slam.yaml` 中关键设置：

- `base_frame: base_link`
- `odom_frame: odom`
- `map_frame: map`
- `scan_topic: /scan`
- `mode: mapping`

## Step 4: 用已有 2D 地图导航

常规启动方式：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=$M20_WS/src/move/map/lab.yaml \
  params_file:=$M20_WS/src/m20/config/m20_nav2_real.yaml \
  use_sim_time:=False
```

当前更常用的 DWB smooth responsive 配置：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=$M20_WS/src/move/map/lab.yaml \
  params_file:=$M20_WS/src/m20/config/m20_nav2_real_dwb_smooth_responsive.yaml \
  use_sim_time:=False \
  rviz:=true
```

Nav2 配置里的 `map_server.yaml_filename` 保持为空，地图文件统一由 launch 命令的 `map:=...` 参数指定，避免写死某台机器上的绝对路径。

如果只想分开启动 localization 和 navigation：

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=$M20_WS/src/move/map/lab.yaml \
  params_file:=$M20_WS/src/m20/config/m20_nav2_real.yaml \
  use_sim_time:=False
```

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$M20_WS/src/m20/config/m20_nav2_real.yaml \
  use_sim_time:=False
```

## Step 5: 启动 global path、RL local path 和 adapter

Global path：

```bash
cd $M20_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/move/move/global_path_publisher.py
```

Pure pursuit 需要订阅 global path。推荐显式 remap：

```bash
python3 src/move/move/pure_pursuit.py --ros-args -r plan:=global_path
```

RL local path：

```bash
cd $M20_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venv/m20_nav/bin/activate

python src/move/move/priest_rl_publisher_nav_cmd_fast.py
```

Adapter：

```bash
cd $M20_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

python src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py
```

如果要退回老版本：

```bash
python src/move/move/priest_rl_publisher_nav_cmd.py
python src/move/move/priest_mppi_adapter_nav_cmd.py
```

## 常用检查

基础定位和传感器：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

Nav2 和控制：

```bash
ros2 lifecycle nodes
ros2 topic hz /global_path
ros2 topic hz /subgoal
ros2 topic hz /local_path
ros2 topic hz /cmd_vel
ros2 topic echo /NAV_CMD
```

## 注意事项

- 这个工作区的定位/建图是 2D `/scan` 链路，主要依赖 `slam_toolbox` 和 Nav2，不使用 Fast-LIO/Open3D。
- `src/move/move/priest_rl_publisher_nav_cmd.py` 已经用 `__file__` 自动推导 `src/RL2Path` 和 `src/move/ckpts`，不再依赖写死的工作区绝对路径。
- `m20_launch.launch.py` 会发布旧版 `/odom`，不要和 `stable/fastlio-localization` 的 Fast-LIO 定位 launch 同时运行。
- `m20_local_path_bt` 行为树包本次不整理，后续如果继续用再单独补文档。
- `src/m20/cmd.md` 保留为原始命令备忘；README 里的命令是按当前 Orin 使用方式整理后的推荐顺序。
