# M20 MID360 Point-LIO + NDT 定位导航系统

这个分支是 `point-lio-lidar-localization`，用于把 MID360 的 Point-LIO 局部里程计和
`lidar_localization_ros2` 的 NDT 点云地图匹配接入 Nav2，并继续运行当前工作区里的
RL local path + Nav2 DWB adapter。

当前工作空间是独立工作空间，不依赖 `m20_ws/src` 里的软链接。

## 分支定位

当前仓库有两条定位线：

| 分支 | 定位前端 | 全局点云定位 | 目标 |
|---|---|---|---|
| `main` | `fast_lio` | `open3d_loc` | 保留上一版定位链路和导航 glue |
| `point-lio-lidar-localization` | `point_lio_ros2` | `lidar_localization_ros2` + `ndt_omp_ros2` | 新方案实验，重点看实时性和稳定性 |

本分支保留完整第三方源码：

- `src/point_lio_ros2`
- `src/lidar_localization_ros2`
- `src/ndt_omp_ros2`

这样后续不需要重新 clone 或再手工打补丁。第三方包已经包含当前 Jetson/CMake/OpenMPI 环境需要的编译兼容修改，以及 Point-LIO 对 MID360 `PointCloud2` 的处理改动。

## 系统目标

原定位链路的问题是 Open3D 全局定位和 Fast-LIO 组合在当前机器上实时性差，Nav2 的 `/local_path` 已经掉到很低频率。这个分支尝试替换为：

- MID360 `/livox/lidar` + `/livox/imu` 进入 `point_lio_ros2`。
- Point-LIO 发布局部 LIO odom：`/odom_corrected`。
- `lidar_localization_ros2` 加载摆正后的 3D PCD 地图，用 NDT_OMP 做 `map -> odom` 校正。
- `fastlio_odom_bridge` 复用原 bridge，把 `/odom_corrected` 转成 Nav2 标准 `/odom`，并发布 `map -> odom_nav -> base_footprint`。
- Nav2 不启动 AMCL，直接使用 3D 定位链路提供的 `map -> odom_nav -> base_footprint`。

不要同时运行旧的：

```bash
ros2 launch m20 m20_launch.launch.py
```

旧 launch 会发布 `simple_odom_old` 和旧的 `odom -> base_link`，会和本分支定位链路冲突。

## 架构

```text
MID360
  ├── /livox/lidar  ─┐
  └── /livox/imu    ─┴──> point_lio_ros2 / pointlio_mapping
                              │
                              ├── /odom_corrected
                              └── TF: odom -> body
                                      │
                                      ▼
                         m20_fastlio_nav / fastlio_odom_bridge
                              │
                              ├── /odom
                              ├── TF: map -> odom_nav
                              ├── TF: odom_nav -> base_footprint
                              └── TF: odom -> base_link

MID360 /livox/lidar ───────> lidar_localization_ros2
3D PCD map ─────────────────> lidar_localization_ros2
                              │
                              ├── /localization/pose_with_covariance
                              ├── /alignment_status
                              └── TF: map -> odom

MID360 /livox/lidar ───────> pointcloud_to_laserscan
                              └── /scan

Nav2 DWB:
  map_server -> /map
  pointcloud_to_laserscan -> /scan
  controller_server -> /cmd_vel
  adapter -> /NAV_CMD
```

核心 TF 链：

```text
map
 ├── odom                 lidar_localization_ros2 动态发布（3D 原始定位帧）
 │    └── base_link       fastlio_odom_bridge 动态发布（3D 调试帧）
 │         ├── livox_frame
 │         └── imu_link
 └── odom_nav             fastlio_odom_bridge 动态发布（Nav2 平面帧）
      └── base_footprint  fastlio_odom_bridge 动态发布
```

`body` 是 Point-LIO 内部机体帧，`base_link` 是机器狗 3D 机体帧，`base_footprint` 是 Nav2 使用的平面帧。桥接节点会把 NDT 的 `map -> odom` 压成 `map -> odom_nav`，同时保留 `odom -> base_link` 方便 3D 调试。

## 工作空间结构

```text
/mnt/nvme/workspace/fast_lio_ws/
├── README.md
├── level_pcd.py                  # PCD 摆正脚本（补偿 MID360 安装倾角）
├── maps/                         # 本地生成地图，已加入 .gitignore，不再纳入 Git
│   └── fastlio/
│       ├── m20_map.pcd
│       ├── m20_map_leveled.pcd
│       ├── global_map.pcd
│       ├── sc_database.txt
│       ├── m20_2d_map.pgm
│       └── m20_2d_map.yaml
└── src/
    ├── fast_lio/                 # main 分支原定位前端，当前分支保留备用
    ├── fast_lio_map/             # 建图前端
    ├── slam_mapping/             # 建图后端 PGO
    ├── open3d_loc/               # main 分支原全局定位，当前分支保留备用
    ├── pcd2pgm/                  # PCD 转 Nav2 2D OccupancyGrid
    ├── m20_fastlio_nav/          # M20 专用 launch、参数、odom bridge
    ├── ndt_omp_ros2/             # NDT_OMP/GICP 加速库，已 vendored
    ├── lidar_localization_ros2/  # NDT 点云地图定位，已 vendored
    └── point_lio_ros2/           # Point-LIO 局部 LIO odom，已 vendored
```

## 地图文件和 Git

`maps/*` 已加入 `.gitignore`，并且本分支已经把历史中跟踪过的地图文件从 Git 索引移除。也就是说：

- 本地 `maps/fastlio/*.pcd`、`*.pgm`、`*.yaml` 仍然保留，可直接运行。
- 新建图或重新转图后，Git 不会再显示地图文件 dirty。
- 如果确实要把某个地图版本强制提交，需要显式 `git add -f maps/fastlio/xxx`。

这样避免每次实车建图或 rosbag 调试后，仓库状态被大体积地图文件污染。

## 包说明

| 包 | 来源/用途 | 运行阶段 |
|---|---|---|
| `fast_lio_map` | Fast-LIO2 建图前端，保存原始 PCD | 建图 |
| `slam_mapping` | 建图后端 PGO，保存 `global_map.pcd` 和 `sc_database.txt` | 建图 |
| `point_lio` | Point-LIO ROS2 包，发布 `/odom_corrected` | 替代定位/导航 |
| `ndt_omp_ros2` | NDT_OMP/GICP 注册库，供 `lidar_localization_ros2` 链接 | 替代定位/导航 |
| `lidar_localization_ros2` | NDT/GICP 点云地图定位，发布 `map -> odom` | 替代定位/导航 |
| `pcd2pgm` | 3D PCD 转 2D 占据栅格 | 地图转换 |
| `m20_fastlio_nav` | M20 glue 包：TF、参数、launch、`/odom_corrected` -> `/odom` | 全流程 |
| `move` | 当前拷贝进工作区的 global path、RL local path 和 adapter | 导航执行 |

## 关键文件

```text
src/m20_fastlio_nav/launch/m20_fastlio_mapping.launch.py
src/m20_fastlio_nav/launch/m20_pcd2pgm_save.launch.py
src/m20_fastlio_nav/launch/m20_point_lio_localization.launch.py
src/m20_fastlio_nav/launch/m20_point_lio_nav.launch.py

src/m20_fastlio_nav/config/fastlio_mapping_mid360.yaml
src/m20_fastlio_nav/config/slam_mapping_mid360.yaml
src/m20_fastlio_nav/config/point_lio_mid360_m20.yaml
src/m20_fastlio_nav/config/lidar_localization_m20.yaml
src/m20_fastlio_nav/config/nav2_dwb_fastlio.yaml
src/m20_fastlio_nav/config/m20_nav3d.rviz

src/m20_fastlio_nav/m20_fastlio_nav/fastlio_odom_bridge.py

src/point_lio_ros2/CMakeLists.txt
src/point_lio_ros2/src/preprocess.cpp
src/point_lio_ros2/src/preprocess.h
src/lidar_localization_ros2/CMakeLists.txt
src/ndt_omp_ros2/CMakeLists.txt

level_pcd.py
```

## MID360 外参

`base_link` 是水平机体坐标系，但 MID360 前倾约 28 度安装：

```text
translation: [0.32713234, 0.01413551, 0.31238696]
quaternion:  [-0.00394028, 0.24367785, 0.00970223, 0.96979969]
Euler:       roll -0.2 deg, pitch 28.2 deg, yaw 1.1 deg
```

这个倾角导致建图坐标系的 Z 轴不是真正垂直。因此 PCD 必须摆正后再用于：

- `lidar_localization_ros2` 的 3D 地图匹配
- `pcd2pgm` 的 2D 栅格地图生成

外参当前同步出现在：

```text
src/m20_fastlio_nav/launch/m20_fastlio_mapping.launch.py
src/m20_fastlio_nav/launch/m20_point_lio_localization.launch.py
src/m20_fastlio_nav/m20_fastlio_nav/fastlio_odom_bridge.py
level_pcd.py
```

后续重新标定 MID360 时，这些位置必须同步修改。

## Point-LIO 配置要点

本分支使用 `point_lio_ros2` 的 `pointlio_mapping` 可执行文件，但入口和参数由 `m20_fastlio_nav` 管理。

配置文件：

```text
src/m20_fastlio_nav/config/point_lio_mid360_m20.yaml
```

关键参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `common.lid_topic` | `/livox/lidar` | MID360 点云输入 |
| `common.imu_topic` | `/livox/imu` | MID360 IMU 输入 |
| `preprocess.lidar_type` | `1` | Point-LIO 内部用 AVIA/Livox 分支处理 Livox 点云 |
| `preprocess.scan_line` | `4` | MID360 扫描线数 |
| `preprocess.timestamp_unit` | `3` | 按当前 Livox `PointCloud2` 时间字段适配 |
| `mapping.extrinsic_est_en` | `false` | 不在线估计外参 |
| `mapping.gravity_align` | `true` | 用 IMU 重力方向辅助初始化 |
| `publish.scan_publish_en` | `false` | 不发布 Point-LIO 世界点云，降低负载 |
| `publish.scan_bodyframe_pub_en` | `false` | 不发布 body 点云，降低负载 |
| `pcd_save.pcd_save_en` | `false` | 定位模式不保存 PCD |

`point_lio_ros2` 原始代码对 ROS2 `PointCloud2` 的 MID360 点类型支持不完整。本分支已经在 `src/point_lio_ros2/src/preprocess.cpp` 和 `src/point_lio_ros2/src/preprocess.h` 里增加了 `x/y/z/intensity/tag/line` 点类型处理，避免必须依赖 Livox `CustomMsg`。

## lidar_localization_ros2 配置要点

配置文件：

```text
src/m20_fastlio_nav/config/lidar_localization_m20.yaml
```

关键参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `registration_method` | `NDT_OMP` | 使用 NDT_OMP |
| `ndt_resolution` | `1.0` | NDT 栅格分辨率 |
| `ndt_num_threads` | `4` | 线程数，Jetson 上不要盲目拉高 |
| `ndt_max_iterations` | `30` | 单帧最大迭代 |
| `voxel_leaf_size` | `0.25` | 实时 scan 下采样 |
| `scan_max_range` | `80.0` | 远点过滤 |
| `scan_min_range` | `0.5` | 近点过滤 |
| `use_pcd_map` | `true` | 从 PCD 文件加载地图 |
| `map_path` | `m20_map_leveled.pcd` | 默认地图，可由 launch 参数覆盖 |
| `set_initial_pose` | `false` | 默认等 RViz 初始位姿 |
| `use_odom` | `false` | 当前不使用 odom 预测，先降低耦合 |
| `use_imu` | `false` | 当前不使用 IMU 预测，先降低耦合 |
| `enable_local_map_crop` | `true` | 使用局部地图裁剪降低计算量 |
| `local_map_radius` | `60.0` | 局部地图半径 |
| `reject_above_score_threshold` | `true` | 分数过差时拒绝更新 |
| `enable_map_odom_tf` | `true` | 发布 `map -> odom` |

启动后建议用 RViz 的 `2D Pose Estimate` 给一次初始位姿。`lidar_localization_ros2` 默认需要收到初始位姿后才开始稳定处理点云。

## 编译

### 依赖

```text
ROS 2:      /opt/ros/humble
Livox:      ~/liv_ws
工作空间:   /mnt/nvme/workspace/fast_lio_ws
```

`point_lio_ros2`、`ndt_omp_ros2`、`lidar_localization_ros2` 已经直接放进本分支 `src/` 下。不要再去
`third_party/` 里按补丁流程 clone，也不要再次 clone 同名目录。拿到这个分支后，源码已经齐全，直接按下面命令编译。

### 构建建图和主包

如果只需要建图、转图、原 Fast-LIO/Open3D 入口和 M20 glue：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash

colcon build --packages-select fast_lio fast_lio_map slam_mapping open3d_loc pcd2pgm m20_fastlio_nav \
  --symlink-install \
  --executor sequential \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 构建 Point-LIO 替代定位

Jetson 上建议限制并行度，避免 `lidar_localization_ros2` 大模板文件编译时拖慢或吃满内存：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1

colcon build \
  --packages-select ndt_omp_ros2 lidar_localization_ros2 point_lio m20_fastlio_nav \
  --symlink-install \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果你只改了 M20 launch、bridge 或时间戳 republisher，不需要重新编三个 C++ 第三方包，只编 glue 包即可：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash

colcon build --packages-select m20_fastlio_nav --symlink-install
```

这台机器上的 `CMake 4.3 + OpenMPI + PCL/VTK` 兼容性处理已经写进三个第三方包的 `CMakeLists.txt`。如果换机器或系统版本，先保留这些修改，除非确认系统 CMake/PCL/MPI 不再有同样问题。

### 验证构建

```bash
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source /mnt/nvme/workspace/fast_lio_ws/install/setup.bash

ros2 pkg prefix point_lio
ros2 pkg prefix ndt_omp_ros2
ros2 pkg prefix lidar_localization_ros2
ros2 pkg prefix m20_fastlio_nav

ros2 launch m20_fastlio_nav m20_point_lio_localization.launch.py --show-args
ros2 launch m20_fastlio_nav m20_point_lio_nav.launch.py --show-args
```

## 环境准备

### LD_PRELOAD

当前系统上 `/opt/MVS` 相机 SDK 自带的旧版 `libusb-1.0.so` 可能抢在系统新版前面加载，而系统 PCL 需要新版 libusb 的 `libusb_set_option` 符号。所有 link 了 PCL 的节点都建议带上：

```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0
```

如果经常忘，可以写进 `.bashrc`：

```bash
echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0' >> ~/.bashrc
source ~/.bashrc
```

### source 顺序

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source /mnt/nvme/workspace/fast_lio_ws/install/setup.bash
```

如果要运行 RL local path 相关 Python，继续进入：

```bash
source ~/venv/m20_nav/bin/activate
```

## Step 1: 启动 MID360 驱动

建议单独开一个终端：

```bash
cd ~/liv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

驱动应该发布：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | MID360 点云，frame 通常为 `livox_frame` |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 内置 IMU |

检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/imu --once
```

`m20_point_lio_*` launch 也提供 `start_livox:=true`，但实车调试时建议分终端启动，日志更清楚。

## Step 2: 建 3D PCD 地图

建图入口仍使用原 Fast-LIO + PGO 链路，不使用 Point-LIO 建图。

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_fastlio_mapping.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd \
  rviz:=true
```

这个 launch 内部会同时起：

- `fast_lio_map/fastlio_mapping`
- `slam_mapping/alaserPGO`

建图阶段关注：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_body
ros2 topic hz /aft_pgo_odom
ros2 topic hz /aft_pgo_map
ros2 service list | grep -E 'map_save|save_pgo_map'
```

保存前端原始图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

保存后端 PGO 图：

```bash
ros2 service call /save_pgo_map std_srvs/srv/Trigger {}
```

正常 `Ctrl+C` 结束建图时，`slam_mapping` 也会自动保存一次：

```text
maps/fastlio/global_map.pcd
maps/fastlio/sc_database.txt
```

如果只看最终建图效果，优先用 `global_map.pcd`。

## Step 3: 摆正 PCD

MID360 前倾约 28 度，原始 PCD 坐标系通常是歪的。直接转 2D 地图或给 NDT 定位会让地图和 Nav2 坐标系不一致。

处理前端原始图：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
python3 level_pcd.py \
  --input /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd \
  --output /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

处理后端 PGO 图：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
python3 level_pcd.py \
  --input /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/global_map.pcd \
  --output /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

实际建议：优先把 `global_map.pcd` 摆正后作为最终定位地图。

## Step 4: 生成 Nav2 2D 地图

用摆正后的 PCD 生成 2D occupancy grid：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_pcd2pgm_save.launch.py \
  pcd_file:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  output_map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map
```

输出：

```text
maps/fastlio/m20_2d_map.pgm
maps/fastlio/m20_2d_map.yaml
```

`pcd2pgm` 参数在：

```text
src/m20_fastlio_nav/config/pcd2pgm_m20.yaml
```

常调参数：

| 参数 | 当前用途 |
|---|---|
| `map_resolution` | 2D 地图分辨率 |
| `thre_z_min` / `thre_z_max` | 参与投影的高度范围 |
| `thre_radius` | 点云膨胀/聚合半径 |
| `thres_point_count` | 判定占据的点数阈值 |

如果 2D 地图墙太粗或障碍太多，优先调高度切片和点数阈值。

## Step 5: 启动 Point-LIO 纯定位

用于先确认 Point-LIO 和 NDT 定位本身是否能工作：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_point_lio_localization.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  rviz:=true
```

如果只看 Point-LIO odom，不跑 NDT 地图匹配：

```bash
ros2 launch m20_fastlio_nav m20_point_lio_localization.launch.py \
  enable_lidar_localizer:=false \
  rviz:=true
```

启动后用 RViz 的 `2D Pose Estimate` 给一次大概初始位姿。NDT 成功后应看到：

- `/odom_corrected` 稳定输出
- `/odom` 稳定输出
- `map -> odom` 存在
- `map -> odom_nav -> base_footprint` 连通
- `/localization/pose_with_covariance` 有输出
- `/alignment_status` 的状态逐步变好

## Step 6: 启动 Point-LIO 定位 + Nav2

确保已经有：

```text
maps/fastlio/m20_map_leveled.pcd
maps/fastlio/m20_2d_map.yaml
```

启动：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_point_lio_nav.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml \
  rviz:=true
```

这条命令是实车模式，和旧 Fast-LIO 链路保持一致：不改写 `/livox/lidar`、`/livox/imu` 的 header stamp。
如果你是播放 rosbag，见后面的“rosbag 调试”章节，核心区别只是加 `use_sim_time:=true` 并用
`ros2 bag play --clock`。

这个 launch 会启动：

1. `m20_point_lio_localization.launch.py`
2. `point_lio/pointlio_mapping`
3. `m20_fastlio_nav/fastlio_odom_bridge`
4. `lidar_localization_ros2/lidar_localization_node`
5. `pointcloud_to_laserscan_node`
6. Nav2 `map_server`
7. Nav2 `navigation_launch.py`

这里使用的是 `nav2_bringup/launch/navigation_launch.py`，不是 `bringup_launch.py`，因此不会启动 AMCL。

## Step 7: 启动 RL local path 和 DWB adapter

Nav2 启动后，再运行当前工作区里的 global path、RL local path 和 adapter。

Terminal: global path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/move/move/global_path_publisher.py
```

Terminal: RL local path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venv/m20_nav/bin/activate

python src/move/move/priest_rl_publisher_nav_cmd_fast.py
```

Terminal: DWB adapter

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py
```

这个 adapter 当前支持定位置信度门控。默认参数里 `require_localization_confidence` 是 `False`，如果后续给 `lidar_localization_ros2` 接一个稳定的置信度 Float32 话题，可以打开：

```bash
python src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py --ros-args \
  -p require_localization_confidence:=true \
  -p localization_confidence_topic:=/localization_3d_confidence \
  -p localization_confidence_threshold:=0.65
```

注意：当前 `lidar_localization_ros2` 原生主要输出 `/alignment_status`，不等价于旧 `open3d_loc` 的 `/localization_3d_confidence`。没有单独桥接前，不要盲目打开该门控。

## 远程 RViz

Orin 上没有显示器时，在笔记本上跑 RViz。前提是笔记本和 Orin 同一局域网，且 ROS_DOMAIN_ID 一致。

拷贝配置：

```bash
scp orin@10.196.232.109:/mnt/nvme/workspace/fast_lio_ws/src/m20_fastlio_nav/config/m20_nav3d.rviz ~/
```

启动：

```bash
export ROS_DOMAIN_ID=0
rviz2 -d ~/m20_nav3d.rviz
```

rosbag 重播时：

```bash
rviz2 -d ~/m20_nav3d.rviz --ros-args -p use_sim_time:=true
```

如果看不到数据：

1. `ping 10.196.232.109`
2. 确认两边 `ROS_DOMAIN_ID` 一致
3. 确认 Orin 上节点和话题已经启动
4. 如果网络阻挡 DDS 多播，再考虑 discovery server

## 运行后检查

Point-LIO：

```bash
ros2 topic hz /odom_corrected
ros2 topic echo /odom_corrected --once
ros2 run tf2_ros tf2_echo odom body
```

NDT 定位：

```bash
ros2 topic hz /localization/pose_with_covariance
ros2 topic echo /alignment_status --once
ros2 run tf2_ros tf2_echo map odom
```

Bridge 和 Nav2 TF：

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map odom_nav
ros2 run tf2_ros tf2_echo odom_nav base_footprint
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Nav2 输入输出：

```bash
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel --once
ros2 node list
ros2 lifecycle nodes
```

期望现象：

- `/odom_corrected` 有稳定输出。
- `/odom` 有稳定输出，`header.frame_id` 是 `odom_nav`，`child_frame_id` 是 `base_footprint`。
- `tf2_echo map odom_nav` 能持续输出，而且基本只有 yaw。
- `/scan` 有 LaserScan 数据。
- Nav2 lifecycle 节点处于 `active`。

## 主要话题

| 话题 | 类型 | 发布者 | 用途 |
|---|---|---|---|
| `/livox/lidar` | `PointCloud2` | Livox driver | Point-LIO、NDT、LaserScan 输入 |
| `/livox/imu` | `Imu` | Livox driver | Point-LIO 输入 |
| `/odom_corrected` | `Odometry` | `point_lio` | 局部 LIO odom |
| `/odom` | `Odometry` | `fastlio_odom_bridge` | Nav2 odom |
| `/localization/pose_with_covariance` | `PoseWithCovarianceStamped` | `lidar_localization_ros2` | NDT 定位结果 |
| `/alignment_status` | `DiagnosticArray` | `lidar_localization_ros2` | NDT 状态和诊断 |
| `/scan` | `LaserScan` | `pointcloud_to_laserscan` | Nav2 costmap 障碍层 |
| `/map` | `OccupancyGrid` | `map_server` | Nav2 2D 地图 |
| `/cmd_vel` | `Twist` | Nav2 controller | adapter 输入 |
| `/NAV_CMD` | `drdds/NavCmd` | adapter | 底盘控制输出 |

## Nav2 参数

当前 Nav2 配置：

```text
src/m20_fastlio_nav/config/nav2_dwb_fastlio.yaml
```

关键点：

- `controller_frequency: 10.0`
- DWB 控制器，不使用 MPPI。
- `/odom` 来自 `fastlio_odom_bridge`，`header.frame_id` 是 `odom_nav`。
- `/scan` 来自 MID360 原始点云转换到 `base_footprint`。
- AMCL 段虽然可能保留在 YAML 中，但正常启动路径不会启动 AMCL。

## 常见问题

### 1. 启动即崩溃: `undefined symbol: libusb_set_option`

**原因**：`/opt/MVS` 相机 SDK 的旧版 libusb 抢在系统新版前面加载。

**修复**：

```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0
```

建议写进 `.bashrc`。

### 2. `point_lio` 没有 `/odom_corrected`

先看输入：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once
```

重点检查：

- `/livox/lidar` 是否是 `sensor_msgs/msg/PointCloud2`
- 点云是否有 `x/y/z/intensity/tag/line` 字段
- `/livox/imu` 是否连续
- `point_lio_mid360_m20.yaml` 的 `lid_topic` 和 `imu_topic` 是否一致

如果点云字段不含 `line`，当前 `livox_handler` 会跳过点，Point-LIO 很可能起不来。

### 3. NDT 定位不动或一直等待初始位姿

`lidar_localization_ros2` 默认需要初始位姿。用 RViz 的 `2D Pose Estimate` 给一次大概位置和朝向。

检查：

```bash
ros2 topic echo /initialpose --once
ros2 topic echo /alignment_status --once
ros2 run tf2_ros tf2_echo map odom
```

也确认 PCD 路径存在：

```bash
ls -lh /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

### 4. `map -> odom_nav -> base_footprint` 不连通

先确认三段：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom_nav
ros2 run tf2_ros tf2_echo odom_nav base_footprint
```

- `map -> odom` 来自 `lidar_localization_ros2`
- `odom -> base_link` 来自 `fastlio_odom_bridge`，依赖 `/odom_corrected`
- `map -> odom_nav` 和 `odom_nav -> base_footprint` 来自 `fastlio_odom_bridge`

如果 `map -> odom` 没有，先排查 NDT。  
如果 `/odom_corrected` 没有，先排查 Point-LIO。

### 5. `/scan` 没有数据

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_footprint livox_frame
```

`/scan` 来自 `/livox/lidar` 通过 `pointcloud_to_laserscan` 转换到 `base_footprint`。如果 TF 不连通或点云 frame 不对，LaserScan 会为空。

### 6. Nav2 报 TF extrapolation

实车时所有节点应使用系统时间：

```bash
ros2 param get /controller_server use_sim_time
ros2 param get /bt_navigator use_sim_time
ros2 param get /local_costmap/local_costmap use_sim_time
ros2 param get /pointlio_odom_bridge use_sim_time
```

rosbag 重播时所有定位/导航/RViz 节点都要 `use_sim_time:=true`，并且播放 bag：

```bash
ros2 bag play /path/to/bag --clock
```

### 7. RViz / costmap 报 `timestamp ... earlier than all the data in the transform cache`

如果看到类似：

```text
Message Filter dropping message: frame 'livox_frame' ... earlier than all the data in the transform cache
Message Filter dropping message: frame 'base_footprint' ... earlier than all the data in the transform cache
```

优先判断时间线是否混了。你贴出来的日志里：

- TF / Point-LIO / RViz 在 `17797248xx`
- Livox 点云和 costmap 输入在 `17793464xx`

两者差了几天，所以 TF cache 里不可能找到对应时间的变换。

当前分支已经不再改写传感器时间戳，策略与旧 Fast-LIO 定位保持一致。出现这类 warning 时，先检查是否混用了实车和 bag，或者某些节点没有统一 `use_sim_time`。

检查时间戳和话题：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once | grep -A3 stamp
ros2 topic echo /odom_corrected --once | grep -A3 stamp
ros2 topic echo /scan --once | grep -A3 stamp
```

rosbag 重播时不要同时跑实车 Livox 驱动；如果要用 bag 的时间线，启动 launch 时传：

```bash
ros2 launch m20_fastlio_nav m20_point_lio_nav.launch.py \
  use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

### 8. Point-LIO 占用高或频率低

优先降低输出和地图维护开销：

- 确认 `publish.scan_publish_en: false`
- 确认 `publish.scan_bodyframe_pub_en: false`
- 确认 `pcd_save.pcd_save_en: false`
- 适当增大 `point_filter_num`
- 适当增大 `filter_size_surf` 和 `filter_size_map`

当前 launch 里已经设置：

```text
point_filter_num: 3
filter_size_surf: 0.5
filter_size_map: 0.5
odom_only: true
```

### 9. NDT 占用高或定位抖

优先调：

- `voxel_leaf_size`
- `ndt_resolution`
- `ndt_num_threads`
- `local_map_radius`
- `score_threshold`
- `reject_above_score_threshold`

建议先从 `voxel_leaf_size: 0.25 -> 0.4` 和 `local_map_radius: 60 -> 40` 测起，看 CPU 和定位稳定性变化。

### 10. 2D 地图歪或导航坐标不匹配

通常是 PCD 没摆正，或者 3D 定位和 2D map 使用了不同版本地图。

推荐固定流程：

1. 用建图得到 `global_map.pcd`
2. 用 `level_pcd.py` 输出 `m20_map_leveled.pcd`
3. `lidar_localization_ros2` 使用 `m20_map_leveled.pcd`
4. `pcd2pgm` 也使用同一个 `m20_map_leveled.pcd`

### 11. Git 仍显示 maps 改动

本分支已经取消跟踪历史地图文件。如果仍显示：

```bash
git ls-files maps
```

如果有输出，说明当前分支还跟踪 maps，需要执行：

```bash
git rm --cached maps/fastlio/global_map.pcd maps/fastlio/m20_2d_map.pgm maps/fastlio/m20_2d_map.yaml maps/fastlio/m20_map.pcd maps/fastlio/m20_map_leveled.pcd maps/fastlio/sc_database.txt
git commit -m "Stop tracking generated maps"
```

注意不要用 `rm` 删除地图文件，只从 Git 索引移除即可。

## rosbag 调试

### 回放前先停实车 Livox

不要让实车驱动和 bag 同时发布 `/livox/lidar`、`/livox/imu`。否则同一个话题有两个时间线来源，Point-LIO
会先按其中一个初始化，另一个就可能被当成旧数据丢掉。

检查发布者：

```bash
ros2 topic info /livox/lidar -v
ros2 topic info /livox/imu -v
```

回放 bag 时，发布者应该只来自 `rosbag2_player`，不要同时有 `livox_lidar_publisher`。

录必要话题到 NVMe：

```bash
mkdir -p /mnt/nvme/bags
ros2 bag record -o /mnt/nvme/bags/m20_point_lio_run \
  /livox/lidar /livox/imu /tf /tf_static /odom_corrected /odom /scan /alignment_status
```

重播定位：

```bash
ros2 launch m20_fastlio_nav m20_point_lio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml
```

另开终端：

```bash
ros2 bag play /path/to/bag --clock
```

如果重复播放或切 bag，建议完整重启 launch，避免 TF buffer 残留上一轮时间。

### 实车模式和 bag 模式的区别

| 场景 | `use_sim_time` | `/livox/lidar` 来源 | 说明 |
|---|---:|---|---|
| 实车 MID360 | `false` | Livox driver | 使用实车实时话题，不改写 stamp |
| rosbag 回放 | `true` | rosbag2_player | 使用 bag 自带时间线和 `/clock`，不改写 stamp |

原则和旧 Fast-LIO 定位一样：定位链路不改传感器时间戳；bag 调试靠 `use_sim_time:=true` 和 `--clock`；实车调试靠系统时间。

## 推荐启动终端布局

Terminal 1: Livox

```bash
cd ~/liv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Terminal 2: Point-LIO + NDT + Nav2

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_point_lio_nav.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml
```

Terminal 3: global path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/move/move/global_path_publisher.py
```

Terminal 4: RL local path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venv/m20_nav/bin/activate
python src/move/move/priest_rl_publisher_nav_cmd_fast.py
```

Terminal 5: adapter

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py
```

调试终端：

```bash
ros2 topic hz /odom_corrected
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /alignment_status --once
ros2 run tf2_ros tf2_echo map base_link
```

## 清理和重编译

只清当前工作空间生成物：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
rm -rf build install log
```

然后重新按本文档的构建命令执行。

不要在 `m20_ws` 里重新加入本工作区包的软链接。当前约定：

- `fast_lio_ws`: 建图、定位、Nav2 位姿链路、当前验证用 `move` / `RL2Path`。
- `m20_ws`: 原始工程备份和未迁移代码，不作为当前启动入口。
