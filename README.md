# M20 MID360 Fast-LIO2 建图定位导航系统

这个工作空间用于把 MID360 的 Fast-LIO2 建图/定位结果接入 Nav2，并运行当前拷贝到本工作区的
RL local path + Nav2 DWB adapter。

当前工作空间是独立复制出来的，不依赖 `m20_ws/src` 里的软链接。

## 分支定位

当前仓库有两条定位线：

| 分支 | 定位前端 | 全局点云定位 | 状态 |
|---|---|---|---|
| `main` | `fast_lio` | `open3d_loc` | 当前可用主线：MID360 PointCloud2 时间戳已修正，按 Jetson/室内场景收紧 Fast-LIO 和 Open3D 参数 |
| `point-lio-lidar-localization` | `point_lio_ros2` | `lidar_localization_ros2` + `ndt_omp_ros2` | 新方案实验分支，第三方源码已经直接放进 `src/` |

本 README 描述的是 `main`。如果要试 Point-LIO + NDT，请切到：

```bash
git switch point-lio-lidar-localization
```

## 系统目标

原系统依赖简陋轮速/运动估计 `/odom` 加 AMCL 做定位。这个方案替换为：

- MID360 `/livox/lidar` + `/livox/imu` 进入 Fast-LIO2。
- 建图阶段保存 3D PCD 地图。
- 定位阶段 Fast-LIO2 输出局部里程计，Open3D 用当前点云和 PCD 地图配准，发布 `map -> odom`。
- `m20_fastlio_nav` 把 Fast-LIO 的 `/Odometry_loc` 转成 Nav2 标准 `/odom`，并发布 `map -> odom_nav -> base_footprint`。
- Nav2 不再启动 AMCL，直接使用 `map -> odom_nav -> base_footprint`。

不要同时运行旧的：

```bash
ros2 launch m20 m20_launch.launch.py
```

这个旧 launch 会发布 `simple_odom_old` 和旧的 `odom -> base_link`，会和 Fast-LIO 定位链路冲突。

## 架构

```text
MID360
  ├── /livox/lidar  ─┐
  └── /livox/imu    ─┴──> fast_lio_map / fast_lio
                              │
                              ├── 建图前端: /Odometry, /cloud_registered_body
                              │             /map_save 保存 m20_map.pcd
                              ├── 建图后端: slam_mapping / alaserPGO
                              │             /save_pgo_map 保存 global_map.pcd
                              │
                              └── 定位: /Odometry_loc, /cloud_registered_1, /cloud_registered_body_1
                                                    │
                                                    ├── fastlio_odom_bridge
                                                    │       ├── /odom
                                                    │       ├── TF: map -> odom_nav
                                                    │       ├── TF: odom_nav -> base_footprint
                                                    │       └── TF: odom -> base_link
                                                    │
                                                    └── open3d_loc
                                                            └── TF: map -> odom

Nav2 DWB:
  map_server -> /map
  pointcloud_to_laserscan -> /scan
  controller_server -> /cmd_vel
  move adapter -> /NAV_CMD
```

核心 TF 链：

```text
map
 ├── odom                 open3d_loc 动态发布（3D 原始定位帧）
 │    └── camera_init     静态 identity bridge
 │         └── body       Fast-LIO 动态发布
 └── odom_nav             fastlio_odom_bridge 动态发布（Nav2 平面帧）
      └── base_footprint  fastlio_odom_bridge 动态发布
```

`body` 是 Fast-LIO 内部机体帧，`base_link` 是机器狗 3D 机体帧，`base_footprint` 是给 Nav2 用的平面帧。桥接节点会把 Open3D 的 `map -> odom` 再压成 `map -> odom_nav`，同时保留 `odom -> base_link` 给 3D 调试。

## 工作空间结构

```text
/mnt/nvme/workspace/fast_lio_ws/
├── README.md
├── level_pcd.py                  # 可选离线摆正工具；2D 转图已内置同等逻辑
├── maps/                         # 本地生成数据，默认不再进入 Git
│   └── fastlio/
│       ├── m20_map.pcd           # fast_lio_map 直接保存的原始 3D 地图
│       ├── global_map.pcd        # slam_mapping PGO 后端保存的优化 3D 地图
│       ├── sc_database.txt       # 建图后端生成的 ScanContext 数据库，仅用于建图后端输出
│       ├── m20_map_leveled.pcd   # 摆正后的 3D 点云地图（Open3D 定位使用）
│       ├── m20_2d_map.pgm        # pcd2pgm + map_saver 生成
│       └── m20_2d_map.yaml       # Nav2 map_server 使用
└── src/
    ├── fast_lio/                 # 定位用 Fast-LIO2，输出 /Odometry_loc
    ├── fast_lio_map/             # 建图用 Fast-LIO2 前端，输出里程计和局部点云
    ├── slam_mapping/             # 建图 PGO 后端，输出 global_map.pcd / sc_database.txt
    ├── open3d_loc/               # Open3D 点云地图定位，发布 map -> odom
    ├── pcd2pgm/                  # PCD 转 Nav2 2D OccupancyGrid
    └── m20_fastlio_nav/          # M20 专用 launch、参数、odom bridge
```

## 地图文件和 Git 规则

`maps/` 下的 PCD、PGM、YAML、ScanContext 数据库、轨迹文本都是实车运行生成数据，默认不应该进 Git。
`.gitignore` 已经包含：

```text
maps/*
```

但 `.gitignore` 只对**尚未被 Git 跟踪的新文件**生效。历史上已经提交过的地图文件，即使后来写进
`.gitignore`，`git status` 仍会继续显示它们的修改。正确处理方式是：

```bash
git rm --cached maps/fastlio/global_map.pcd \
  maps/fastlio/m20_2d_map.pgm \
  maps/fastlio/m20_2d_map.yaml \
  maps/fastlio/m20_map.pcd \
  maps/fastlio/m20_map_leveled.pcd \
  maps/fastlio/sc_database.txt
```

这个命令只从 Git 索引移除，不删除你本地磁盘上的地图文件。当前 `main` 和
`point-lio-lidar-localization` 都按这个规则处理。

## 包说明

| 包 | 来源/用途 | 运行阶段 |
|---|---|---|
| `fast_lio_map` | luckrobot 的 Fast-LIO2 建图前端，用 `/map_save` 保存原始 PCD | 建图 |
| `slam_mapping` | PGO 建图后端，订阅 Fast-LIO 建图输出，保存 `global_map.pcd` / `sc_database.txt` | 建图 |
| `fast_lio` | luckrobot 的 Fast-LIO2 定位版本，发布 `/Odometry_loc` | 定位/导航 |
| `open3d_loc` | luckrobot 的 Open3D ICP/NDT 类定位节点，发布 `map -> odom` | 定位/导航 |
| `pcd2pgm` | 把 3D PCD 转成 2D 占据栅格，再用 map_saver 保存 | 地图转换 |
| `m20_fastlio_nav` | 本系统新增 glue 包：TF、参数、launch、`/Odometry_loc` -> `/odom` | 全流程 |

## 关键文件

```text
src/m20_fastlio_nav/launch/m20_fastlio_mapping.launch.py
src/m20_fastlio_nav/launch/m20_fastlio_localization.launch.py
src/m20_fastlio_nav/launch/m20_fastlio_nav.launch.py
src/m20_fastlio_nav/launch/m20_pcd2pgm_save.launch.py

src/m20_fastlio_nav/config/fastlio_mapping_mid360.yaml
src/m20_fastlio_nav/config/fastlio_localization_mid360.yaml
src/m20_fastlio_nav/config/slam_mapping_mid360.yaml
src/m20_fastlio_nav/config/open3d_localization_m20.yaml
src/m20_fastlio_nav/config/pcd2pgm_m20.yaml
src/m20_fastlio_nav/config/nav2_dwb_fastlio.yaml
src/m20_fastlio_nav/config/m20_nav3d.rviz      # 定位+导航全景 RViz 配置

src/m20_fastlio_nav/m20_fastlio_nav/fastlio_odom_bridge.py

level_pcd.py                     # 可选离线 PCD 坐标系摆正脚本
```

## Fast-LIO 配置要点

当前实现按 `/home/orin/workspace/luckbot/FAST_LIO_LOCALIZATION_HUMANOID` 的思路重新对齐：
Fast-LIO 定位包和建图包都走 MID360 的 `PointCloud2` 路径，Open3D 只负责低频全局拉回。

注意：当前 `fast_lio` / `fast_lio_map` 代码里的枚举不是上游原版。`MID360=4` 才是 Livox MID360
的 `PointCloud2` 路径。历史配置里的 `lidar_type: 5` 是错误设置：

- 在 `fast_lio_map` 里，`5` 对应 Robosense RSM1。
- 在 `fast_lio` 里，`5` 不匹配任何有效 Livox 分支，会退到普通 `PointXYZI` 处理路径。
- 这会丢掉 MID360 的 `line` / `tag` / `timestamp` 信息，导致运动补偿和前端稳定性变差，还会增加无效计算。

现在源码已经补齐 MID360 `PointCloud2` 字段解析：Livox driver 的 `intensity` 字段会映射到
Fast-LIO 内部的 `reflectivity`，并读取 `tag` / `line` / `timestamp`。注意本机 rosbag 里的
`timestamp` 字段是绝对 ns 时间，不是点内 offset；代码会先取一帧点云内的最小 timestamp，再用
`timestamp - min_timestamp` 转成 Fast-LIO 需要的 scan 内相对时间。这样同时兼容实车驱动和 rosbag
重播，避免把绝对时间直接当点内 offset 导致 `/Odometry` / `/Odometry_loc` 不输出。

| 参数 | 正确值 | 说明 |
|---|---:|---|
| `preprocess.lidar_type` | `4` | MID360 `PointCloud2` 路径 |
| `preprocess.scan_line` | `4` | MID360 实际 line 数，参考 luckbot 配置；不要再用 `96` |
| `preprocess.timestamp_unit` | `3` | MID360 每点 timestamp/offset 以 ns 计，代码会转成 scan 内相对 ms |
| `preprocess.blind` | `0.5` | 近距离盲区过滤 |
| `point_filter_num` | `3` | 前端降采样，降低 Jetson 压力 |
| `filter_size_surf` / `filter_size_map` | `0.5` | 与 luckbot Jetson 配置一致 |
| `mapping.det_range` | 定位 `60` / 建图 `80` | 室内雷达有效距离约 30m，不再使用 `200` |
| `common.time_sync_en` | `false` | 关闭 IMU-LiDAR 时间同步，MID360 时间戳无对齐 |
| `common.lid_topic` | `/livox/lidar` | Livox 驱动话题 |
| `common.imu_topic` | `/livox/imu` | Livox IMU 话题 |

发布项也做了边缘设备裁剪：

- 定位配置关闭 `path_en`，避免持续发布无用轨迹。
- 建图和定位都关闭 `dense_publish_en`，避免额外大点云输出占 CPU/带宽。
- 保留 `scan_publish_en` 和 `scan_bodyframe_pub_en`，因为 Open3D 和 `pointcloud_to_laserscan` 仍需要实时点云。

当前 **建图和定位两个配置文件已经改为正确值**，不要再改回 `lidar_type: 5` / `scan_line: 96` / `det_range: 200`。

## 外参

### MID360 安装姿态

`base_link` 是水平的，但 MID360 **前倾约 28°** 安装在机器狗上：

```text
translation: [0.32713234, 0.01413551, 0.31238696]
quaternion:  [-0.00394028, 0.24367785, 0.00970223, 0.96979969]   ->  Euler: roll -0.2°, pitch 28.2°, yaw 1.1°
```

这个倾角导致建图坐标系（`camera_init`）的 Z 轴不是真正垂直的。因此 **PCD 转 2D 地图前必须先做摆平处理**。现在 `pcd2pgm` 会在转图流程里按 `pcd2pgm_m20.yaml` 自动完成摆平和地面基准对齐，不再需要为 2D 地图单独运行 `level_pcd.py`。

定位阶段的实时 `/scan` 使用 `pointcloud_to_laserscan(target_frame=base_footprint)`，会通过 TF 自动转平，不受影响。

### 外参修改范围

这个外参同时用于：

- `base_link -> livox_frame` 静态 TF
- `base_link -> imu_link` 静态 TF
- `fastlio_odom_bridge` 内部同时计算 `map -> odom_nav`、`odom_nav -> base_footprint` 和 `odom -> base_link`
- `pcd2pgm_m20.yaml` 中的 `leveling_quaternion_xyzw`
- `level_pcd.py` 可选离线摆正脚本

如果后面重新标定 MID360，必须同步修改：

```text
src/m20_fastlio_nav/launch/m20_fastlio_localization.launch.py
src/m20_fastlio_nav/launch/m20_fastlio_mapping.launch.py
src/m20_fastlio_nav/m20_fastlio_nav/fastlio_odom_bridge.py
src/m20_fastlio_nav/config/pcd2pgm_m20.yaml
level_pcd.py
```

## 编译

### 依赖

```text
ROS 2:      /opt/ros/humble
Livox:      ~/liv_ws
Open3D:     /home/orin/drivers/Open3D/install
工作空间:   /mnt/nvme/workspace/fast_lio_ws
```

Open3D 是 C++ 链接使用，不是只靠 Python venv。`open3d_loc` 的 CMake 指向：

```text
/home/orin/drivers/Open3D/install/lib/cmake/Open3D
```

### 完整构建

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1

colcon build --symlink-install --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DMPI_SKIP_COMPILER_WRAPPER=TRUE \
  -DMPI_C_LIB_NAMES=mpi \
  -DMPI_CXX_LIB_NAMES=mpi_cxx\;mpi \
  -DMPI_mpi_LIBRARY=/usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi.so \
  -DMPI_mpi_cxx_LIBRARY=/usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi_cxx.so \
  -DMPI_C_HEADER_DIR=/usr/lib/aarch64-linux-gnu/openmpi/include \
  -DMPI_CXX_HEADER_DIR=/usr/lib/aarch64-linux-gnu/openmpi/include \
  -DMPI_C_COMPILER_INCLUDE_DIRS=/usr/lib/aarch64-linux-gnu/openmpi/include\;/usr/lib/aarch64-linux-gnu/openmpi/include/openmpi \
  -DMPI_CXX_COMPILER_INCLUDE_DIRS=/usr/lib/aarch64-linux-gnu/openmpi/include\;/usr/lib/aarch64-linux-gnu/openmpi/include/openmpi
```

这些 MPI 参数是为了绕过当前系统上 PCL/CMake/OpenMPI 导出的错误 include 路径问题。

### 验证构建

```bash
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source /mnt/nvme/workspace/fast_lio_ws/install/setup.bash

ros2 pkg prefix fast_lio
ros2 pkg prefix fast_lio_map
ros2 pkg prefix open3d_loc
ros2 pkg prefix pcd2pgm
ros2 pkg prefix m20_fastlio_nav

ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py --show-args
```

## 环境准备

### LD_PRELOAD（必须）

当前系统上 `/opt/MVS` 相机 SDK 自带的旧版 `libusb-1.0.so` 被加到了 `LD_LIBRARY_PATH` 最前面，而系统 PCL（`libpcl_io.so`）需要新版 libusb 的 `libusb_set_option` 符号。旧版没有这个函数，导致所有 link 了 PCL 的节点启动即崩溃。

建议直接加到 `.bashrc`，一劳永逸：

```bash
echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0' >> ~/.bashrc
source ~/.bashrc
```

如果不想改 `.bashrc`，下面每条启动命令前都需要手动加。

### source 环境

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source /mnt/nvme/workspace/fast_lio_ws/install/setup.bash
```

如果还要运行 RL local path 相关 Python，继续留在 `fast_lio_ws`，只需要额外进入 `~/venv/m20_nav`。

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

`m20_fastlio_*` launch 里也提供 `start_livox:=true`，但实车调试时建议分终端启动，日志更清楚。

## Step 2: 建 3D PCD 地图

另开终端：

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

如果需要 RViz 监控（见下方"远程 RViz"章节）：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_mapping.launch.py rviz:=true
```

建图时遥控机器狗慢速走完整个环境，尽量让回环区域有足够重叠。开始时确保机器狗放在**水平地面**上，按步骤启动后等几秒 IMU 初始化后再开始走。

当前 mapping launch 会同时启动：

1. `fast_lio_map/fastlio_mapping`：前端 LIO，输出 `/Odometry`、`/cloud_registered_body`，可通过 `/map_save` 保存原始 `m20_map.pcd`。
2. `slam_mapping/alaserPGO`：后端 pose graph，订阅前端里程计和局部点云，做关键帧、ScanContext 和回环优化，可通过 `/save_pgo_map` 保存 `global_map.pcd` 和 `sc_database.txt`。

所以现在不是单纯 Fast-LIO 增量建图。实测时建议先保存后端优化图；如果 PGO 没有稳定闭环或地图明显异常，再退回使用前端原始 `m20_map.pcd`。

建图阶段关注：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_registered_body
ros2 service list | grep map_save
ros2 service list | grep save_pgo_map
```

### 用 rosbag 回放建图

如果用 rosbag 验证建图，建图 launch 必须使用仿真时间，bag 播放必须发布 `/clock`：

Terminal 1：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
ros2 launch m20_fastlio_nav m20_fastlio_mapping.launch.py \
  use_sim_time:=true \
  rviz:=true
```

Terminal 2：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash

ros2 bag play ~/workspace/fast_lio_ws/bags/lab1 --clock
```

判断标准：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_body
ros2 topic hz /cloud_registered
```

`/Odometry` 有输出才说明 `fast_lio_map` 前端正常吃到了 `/livox/lidar` 和 `/livox/imu`。如果实车能建图但 bag 没反应，优先检查是否漏了 `use_sim_time:=true` 或 `--clock`，以及是否运行的是重编后的 `fast_lio_map`。

保存前端原始 PCD：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

默认保存到：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd
```

保存后端 PGO 地图：

```bash
ros2 service call /save_pgo_map std_srvs/srv/Trigger {}
```

默认保存到：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/global_map.pcd
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/sc_database.txt
```

确认文件存在：

```bash
ls -lh /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd
ls -lh /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/global_map.pcd
ls -lh /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/sc_database.txt
```

## Step 3: 生成 Nav2 2D 地图

`pcd2pgm` 现在会在转图流程里完成三件事：

1. 用 `leveling_quaternion_xyzw` 补偿 MID360 安装倾角，把原始 PCD 摆到接近水平。
2. 可选拟合地面平面做小角度细修正。
3. 自动检测地面高度并平移到 `z=0`，然后用 `thre_z_min/thre_z_max` 按相对高度切片。

因此生成 2D 图时默认直接输入原始/优化后的 3D PCD，不需要先生成 `m20_map_leveled.pcd`：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
ros2 launch m20_fastlio_nav m20_pcd2pgm_save.launch.py \
  pcd_file:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/global_map.pcd \
  output_map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map
```

输出：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.pgm
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml
```

`pcd2pgm` 参数在：

```text
src/m20_fastlio_nav/config/pcd2pgm_m20.yaml
```

重要参数都在 `src/m20_fastlio_nav/config/pcd2pgm_m20.yaml`：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `pcd_file` | `global_map.pcd` | 默认输入原始/优化 3D PCD，`m20_pcd2pgm_save.launch.py` 的 `pcd_file:=...` 会覆盖它 |
| `odom_to_lidar_odom` | `[0,0,0,0,0,0]` | 转图前额外施加的已知位姿修正，通常保持全 0 |
| `enable_leveling` | `true` | 启用内置 PCD 摆平，替代单独运行 `level_pcd.py` |
| `leveling_quaternion_xyzw` | MID360 外参四元数 | 用 `base_link -> livox_frame` 外参补偿雷达安装倾角 |
| `refine_ground_plane` | `true` | 摆平后拟合近水平面并做小角度细修正 |
| `ground_plane_voxel_size` | `0.08` | 拟合地面平面前的降采样体素大小，单位 m |
| `ground_plane_distance_threshold` | `0.03` | RANSAC 平面内点距离阈值，单位 m |
| `ground_plane_min_normal_z` | `0.95` | 只有拟合平面接近水平时才用于细修正 |
| `auto_align_ground_to_map` | `true` | 自动把检测到的地面高度平移到 `z=0`，作为最终 2D map 基准面 |
| `ground_histogram_bin_size` | `0.10` | 自动找地面高度时的 Z 方向直方图间隔，单位 m |
| `thre_z_min` | `0.1` | 相对最终 2D map 基准面 `z=0` 的最低切片高度 |
| `thre_z_max` | `0.5` | 相对最终 2D map 基准面 `z=0` 的最高切片高度 |
| `thre_radius` | `0.8` | 半径滤波搜索半径，单位 m |
| `thres_point_count` | `10` | 半径内最少邻居数，低于该值的离群点会被滤掉 |
| `map_resolution` | `0.05` | 2D 地图分辨率，单位 m/pixel |

常规调参只需要动 `thre_z_min/thre_z_max`。例如 `0.1 / 0.5` 表示取 2D map 基准面上方 10cm 到 50cm 的点云投影成 2D 占据图；如果需要包含地面下方点，再把 `thre_z_min` 设成负数。不要再手算 `0.84` 这类绝对高度。

`level_pcd.py` 仍保留为离线检查工具；只有需要生成或复核 3D 定位用的 `m20_map_leveled.pcd` 时才需要运行。

## Step 4: 启动 Fast-LIO 定位 + Nav2

确保已经有：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd  # 摆正后的 3D PCD（给 Open3D）
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml      # 2D 地图（从摆正 PCD 生成）
```

启动：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml \
  rviz:=true
```

实车导航默认建议 `rviz:=false`。Open3D 初始化、3D 点云显示和远程 RViz 都会增加 Orin 负载；只有部署、看 TF/点云对齐或排查定位时再改成 `rviz:=true`。

这个 launch 会启动:

1. `m20_fastlio_localization.launch.py`
2. Fast-LIO 定位节点 `fast_lio/fastlio_mapping`
3. `fastlio_odom_bridge`
4. `open3d_loc/global_localization_node`
5. `pointcloud_to_laserscan_node`
6. Nav2 `map_server`
7. Nav2 `navigation_launch.py`

注意：这里使用的是 `nav2_bringup/launch/navigation_launch.py`，不是 `bringup_launch.py`，因此不会启动 AMCL。

如果临时还没有新 2D 地图，也可以使用旧的 lab 2D 地图（坐标系不匹配，仅调试用）：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/m20_ws/src/move/map/lab.yaml
```

## Step 5: 启动 RL local path 和 DWB adapter

Nav2 启动后，再按当前验证过的方式运行 global path、pure pursuit、RL 路径发布和 adapter。当前拷贝到本工作区的 `move` / `RL2Path` 默认按 2D 链路运行：`global_path(map)` -> `subgoal(map)` -> `local_path(base_footprint)` -> adapter 输出到 `odom_nav`。

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/move/move/global_path_publisher.py
```

`pure_pursuit.py` 必须和 global Path 发布配合使用：global publisher 发布 `/global_path`，pure pursuit 通过 remap 订阅这条全局路径，并发布 `/subgoal` / `/final_goal` 给 RL local path 使用。

另开终端：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/move/move/pure_pursuit.py --ros-args -r plan:=global_path
```

另开终端：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venv/m20_nav/bin/activate

python src/move/move/priest_rl_publisher_nav_cmd_fast.py
```

adapter 用当前实车验证效果最好的文件：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py
```

当前推荐使用 `priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py`。它默认不强依赖
`/localization_3d_confidence`，因此不会因为 Open3D 没有发布置信度而把底盘一直冻结。如果你想启用定位置信度门控，可以通过参数把
`require_localization_confidence` 设为 `true`。注意这里有两个阈值：Open3D 的 `confidence_loc_th=0.70`
用于判断 3D 配准是否接受；adapter 的 `localization_confidence_threshold=0.65` 是底盘命令门控阈值。

启用置信度门控后，当 Open3D 还没定位成功、正在重初始化，或者置信度低于默认阈值 `0.65` 时，adapter 会：

- 暂停给 Nav2 发送新的 `FollowPath` goal
- 取消已经激活的 `FollowPath` goal
- 持续向 `/NAV_CMD` 发布零速度，避免定位没锁住时底盘继续执行旧速度

恢复定位后，如果 `/localization_3d_confidence >= adapter.localization_confidence_threshold`（默认 `0.65`），adapter 会自动恢复转发 `/cmd_vel` 到 `/NAV_CMD`。如果启用了门控且 1.5 秒内收不到新的 `/localization_3d_confidence`，adapter 也会按定位失效处理并冻结输出。

## 远程 RViz（笔记本外接显示器）

Orin 上没有显示器，需要在笔记本上跑 RViz。前提：笔记本和 Orin 同一局域网（能 ping 通 `10.196.232.109`），笔记本装了 `ros-humble-rviz2`。

### 1) 从 Orin 拷 RViz 配置文件

```bash
# 在笔记本上执行
# 定位+导航全景模式（推荐，含3D点云+2D地图+costmap+plan）：
scp orin@10.196.232.109:/mnt/nvme/workspace/fast_lio_ws/src/m20_fastlio_nav/config/m20_nav3d.rviz ~/

# 纯 3D 定位模式（仅点云和 TF）：
scp orin@10.196.232.109:/mnt/nvme/workspace/fast_lio_ws/src/open3d_loc/rviz_cfg/fastlio.rviz ~/

# 建图模式：
scp orin@10.196.232.109:/mnt/nvme/workspace/fast_lio_ws/src/fast_lio_map/rviz/fastlio.rviz ~/
```

### 2) 启动 RViz

```bash
# 笔记本终端，ROS_DOMAIN_ID 和 Orin 保持一致（默认 0 就不用设）
export ROS_DOMAIN_ID=0
rviz2 -d ~/m20_nav3d.rviz
```

RViz 会自动通过 DDS 发现 Orin 上所有话题和 TF。
如果是在看 rosbag 重播，RViz 也必须跟随 `/clock`：

```bash
rviz2 -d ~/m20_nav3d.rviz --ros-args -p use_sim_time:=true
```

| 模式 | RViz 配置文件 | Fixed Frame | 内容 |
|---|---|---|---|
| 建图 | `src/fast_lio_map/rviz/fastlio.rviz` | `camera_init` | TF、里程计、点云 |
| 纯 3D 定位 | `src/open3d_loc/rviz_cfg/fastlio.rviz` | `map` | TF、3D 地图点云、实时扫描 |
| 定位+导航全景 | `src/m20_fastlio_nav/config/m20_nav3d.rviz` | `map` | 上述 + 2D 地图 + costmap + plan + /scan |

推荐笔记本上用**定位+导航全景**配置，可以看到完整链路。

### 3) 如果看不到数据

1. `ping 10.196.232.109` 确认网络通
2. 两边确认 `echo $ROS_DOMAIN_ID` 一致（默认都是 0 就不用设任何东西）
3. 如果公司网络有 VLAN/防火墙阻挡 DDS 多播，改用 ROS discovery server 方式排查

## 运行后检查

定位链路：

```bash
ros2 topic hz /Odometry_loc
ros2 topic hz /odom
ros2 topic hz /cloud_registered_1
ros2 topic hz /cloud_registered_body_1
ros2 topic echo /localization_3d_confidence
```

TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map odom_nav
ros2 run tf2_ros tf2_echo odom_nav base_footprint
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo map base_link
```

Nav2 输入输出：

```bash
ros2 topic hz /scan
ros2 topic hz /cmd_vel_nav
ros2 topic echo /cmd_vel_nav
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
ros2 node list
ros2 lifecycle nodes
```

期望现象：

- `/Odometry_loc` 有稳定输出（几秒内就应该有数据）。
- `/odom` 有稳定输出，`header.frame_id` 是 `odom_nav`，`child_frame_id` 是 `base_footprint`。
- `tf2_echo map odom_nav` 能持续输出，而且基本只有 yaw。
- `/scan` 有 LaserScan 数据。
- Nav2 lifecycle 节点处于 `active`。
- `/cmd_vel_nav` 是 controller 原始输出，`/cmd_vel` 是 velocity_smoother 平滑后的输出。adapter 默认订阅 `/cmd_vel` 并转换到 `/NAV_CMD`。

## 主要话题

| 话题 | 类型 | 发布者 | 用途 |
|---|---|---|---|
| `/livox/lidar` | `PointCloud2` | Livox driver | Fast-LIO 输入 |
| `/livox/imu` | `Imu` | Livox driver | Fast-LIO 输入 |
| `/Odometry` | `Odometry` | `fast_lio_map` 建图模式 | 建图里程计 |
| `/Odometry_loc` | `Odometry` | `fast_lio` 定位模式 | 定位里程计 |
| `/odom` | `Odometry` | `fastlio_odom_bridge` | Nav2 odom |
| `/cloud_registered_1` | `PointCloud2` | `fast_lio` | Open3D 地图配准 |
| `/cloud_registered_body_1` | `PointCloud2` | `fast_lio` | 转 `/scan` |
| `/scan` | `LaserScan` | `pointcloud_to_laserscan` | Nav2 costmap 障碍层 |
| `/map` | `OccupancyGrid` | `map_server` | Nav2 2D 地图 |
| `/cmd_vel_nav` | `Twist` | Nav2 controller | controller 原始速度输出 |
| `/cmd_vel` | `Twist` | Nav2 velocity_smoother | adapter 默认订阅的平滑速度 |
| `/NAV_CMD` | `drdds/NavCmd` | move adapter | 发送给底盘的速度命令 |
| `/localization_3d_confidence` | `Float32` | `open3d_loc` | 3D 定位置信度 |

## Nav2 参数

当前 Nav2 配置：

```text
src/m20_fastlio_nav/config/nav2_dwb_fastlio.yaml
```

它基于之前实车较平滑的 DWB 参数，关键点：

- `controller_frequency: 10.0`
- DWB 控制器，不使用 MPPI。
- 保留底盘死区：
  - `min_speed_xy: 0.20`
  - `min_speed_theta: 0.40`
- `max_vel_x: 0.80` / `max_speed_xy: 0.80`，当前实测比 `0.60` 更能跟上局部路径频率。
- `velocity_smoother.max_velocity: [0.8, 0.0, 0.6]`，和 DWB 最大速度保持一致。
- `/odom` 来自 Fast-LIO bridge，`header.frame_id` 是 `odom_nav`.
- `/scan` 来自 Fast-LIO body 点云，转换到 `base_footprint` 坐标系（自动水平）。
- AMCL 段虽然保留在 YAML 中，但 `tf_broadcast: false`，且正常启动路径不会启动 AMCL。

## Open3D 定位参数

配置文件：

```text
src/m20_fastlio_nav/config/open3d_localization_m20.yaml
```

关键参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `path_map` | `m20_map_leveled.pcd` | 3D PCD 地图（摆正后，与2D地图坐标系一致） |
| `initialpose` | `[0,0,0,0,0,0]` | 初始位姿，单位 m/deg |
| `pcd_queue_maxsize` | `10` | Open3D 等待 Fast-LIO 点云队列长度 |
| `voxelsize_coarse` | `0.15` | 粗配准体素，参考 luckbot Jetson 配置 |
| `voxelsize_fine` | `0.10` | 精配准体素，兼顾贴墙稳定性和 CPU |
| `threshold_fitness_init` | `0.50` | 初始化接受阈值 |
| `threshold_fitness` | `0.50` | 定位接受阈值 |
| `loc_frequence` | `2.5` | Open3D 全局拉回频率；不要当成前端里程计频率 |
| `maxpoints_source` | `80000` | 当前帧最大参与配准点数 |
| `maxpoints_target` | `400000` | 局部地图最大参与配准点数 |
| `confidence_loc_th` | `0.70` | 置信度阈值 |
| `dis_updatemap` | `3.5` | 移动超过该距离后更新局部地图 |
| `reset_fastlio_on_initialpose` | `true` | RViz `2D Pose Estimate` 后同时请求重置 Fast-LIO |
| `fastlio_reset_service` | `/fastlio_localization_odom/reset_localization` | Fast-LIO 安全复位服务 |

`open3d_loc` 初始化和重定位会消耗明显 CPU。实车导航时默认 `rviz:=false`，只在部署、看 TF/点云对齐、
排查定位问题时打开 RViz；远程 RViz 也会通过 DDS 拉点云，仍然会增加 Orin 侧负载。

如果机器人初始位置和 PCD 地图坐标差很多，需要在 RViz 用 `2D Pose Estimate` 给大概初值，或者修改 `initialpose`。

这里的 `initialpose` / RViz `2D Pose Estimate` 表达的是 `map -> base_link` 机器人位姿，不要求机器人从地图原点启动。定位节点会先结合当前 Fast-LIO 的 `odom -> base_link` 自动换算出正确的 `map -> odom`，然后调用 Fast-LIO 复位服务，清掉已经漂坏的 EKF/局部地图，再用新的 `/Odometry_loc` 和点云重新初始化 Open3D 定位。

## 常见问题

### 1. 启动即崩溃: `undefined symbol: libusb_set_option`

所有 link 了 PCL 的节点（fastlio_mapping、pcd2pgm_node 等）都会报这个错。

**原因**：`/opt/MVS` 相机 SDK 的旧版 libusb 抢在系统新版前面加载了。

**修复**：

```bash
echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0' >> ~/.bashrc
source ~/.bashrc
```

此后所有终端自动生效。必须执行。

### 2. Fast-LIO 始终不输出 `/Odometry_loc`，Open3D 一直 "Waiting for Odometry_loc..."

**原因 A**：`lidar_type` 配成了 AVIA (1)，Fast-LIO 订阅了 `CustomMsg`，但 Livox 驱动发布的是 `PointCloud2`，类型不匹配。

**修复**：确认 `fastlio_localization_mid360.yaml` 中 `preprocess.lidar_type: 4`，并且 `scan_line: 4`。

**原因 B**：`time_sync_en: true`，MID360 的时间戳无对齐，Fast-LIO 在等同步。

**修复**：确认 `fastlio_localization_mid360.yaml` 中 `common.time_sync_en: false`。

### 3. Nav2 报 "Could not find a connection between 'odom_nav' and 'base_footprint'"

TF 链没连通。先确认 Fast-LIO 有输出（问题 2），再检查：

```bash
ros2 topic hz /Odometry_loc
ros2 run tf2_ros tf2_echo map odom_nav
ros2 run tf2_ros tf2_echo odom_nav base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

- `map -> odom_nav` 由 `fastlio_odom_bridge` 从 Open3D 的 `map -> odom` 压平得到。
- `odom_nav -> base_footprint` 由 `fastlio_odom_bridge` 发布，依赖 `/Odometry_loc`，这是 Nav2 用的平面位姿。
- `odom -> base_link` 也由 `fastlio_odom_bridge` 发布，用于 3D 可视化和点云转换。
- `map -> odom` 由 `open3d_loc` 发布，依赖 `/Odometry_loc` 和 `/cloud_registered_1`。
- Open3D 初始化需要几十秒（加载 PCD 地图），耐心等待 "localization initialize success"。

### 4. `/scan` 没有数据

```bash
ros2 topic hz /cloud_registered_body_1
ros2 topic hz /scan
```

`/scan` 来自 `/cloud_registered_body_1` 通过 `pointcloud_to_laserscan` 转到 `base_footprint`。如果 Fast-LIO 没有 body 点云，`/scan` 也不会出数据。

### 5. Nav2 报 TF extrapolation

检查所有节点都用真机时间：

```bash
ros2 param get /controller_server use_sim_time
ros2 param get /bt_navigator use_sim_time
ros2 param get /local_costmap/local_costmap use_sim_time
ros2 param get /fastlio_odom_bridge use_sim_time
ros2 param get /fastlio_pointcloud_to_laserscan use_sim_time
```

实车应全部为 `False`。
rosbag 重播应全部为 `True`，并且用 `ros2 bag play ... --clock`。如果混用，一轮 bag 播完后下一轮开头很容易刷 `TF_OLD_DATA`。

也检查延迟：

```bash
ros2 topic delay /livox/lidar
ros2 topic delay /livox/imu
ros2 topic delay /odom
```

### 6. PCD 转 2D 地图效果差或不水平

**原因 A**（不水平）：MID360 安装有倾角，PCD 坐标系本身是歪的。现在 `pcd2pgm_m20.yaml` 里 `enable_leveling: true` 会在转图时自动摆平；如果仍然不水平，优先检查 `leveling_quaternion_xyzw` 是否和 launch 里的 `base_link -> livox_frame` 外参一致。

**原因 B**（障碍过多/过少）：调整 `pcd2pgm_m20.yaml` 中的参数：

- `thre_z_min` / `thre_z_max`：控制相对自动地面基准 `z=0` 的切片高度范围
- `thres_point_count`：判定占据的点数阈值
- `map_resolution`：2D 地图分辨率

### 7. Open3D 构建失败

确认 Open3D C++ 安装存在：

```bash
ls /home/orin/drivers/Open3D/install/lib/cmake/Open3D
ls /home/orin/drivers/Open3D/install/lib/libOpen3D.a
```

如果出现 MPI include 路径错误，必须使用本文档的完整 `colcon build` 命令，不要省略 MPI 参数。

### 8. 速度控制一卡一卡

先确认问题不在定位链路：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

如果 `/cmd_vel_nav` 已经不稳，优先看 Nav2 controller/costmap 负载。如果 `/cmd_vel_nav` 稳定但 `/cmd_vel` 不稳，看 `velocity_smoother` 参数。如果 `/cmd_vel` 稳定但底盘不稳，再看 adapter 或底盘控制接口。

### 9. `ros2 bag record` 录一段就停了

这通常不是“内存不够”，而是录包写盘太慢、输出目录在系统盘，或者一次录了太多高频话题。`rosbag2` 默认只用较小缓存，但最终还是要持续写到磁盘。

建议直接录到 NVMe/SSD，并只录必要话题：

```bash
mkdir -p /mnt/nvme/bags
ros2 bag record -o /mnt/nvme/bags/m20_run \
  /livox/lidar /livox/imu /tf /tf_static /Odometry_loc /odom /scan
```

如果只是排查 Fast-LIO，优先只录 `/livox/lidar` 和 `/livox/imu`。不要把 bag 直接落在系统盘或 `/home` 下面。

如果 bag 里是“中间空一段”，先判断是不是源话题本身就断流：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

如果实时话题是连续的，但 bag 里有空洞，通常就是 recorder 写盘或缓存跟不上。优先换到 NVMe/SSD，并避免压缩录包。

### 10. 激光迟启动或运行中重启后定位恢复

当前代码支持 MID360 驱动晚于定位导航启动、运行中短暂异常后恢复，或者同一个定位进程内 rosbag 播完后重新播放/换包。恢复链路分三层：

- Fast-LIO 检测到 `/livox/lidar` 或 IMU 时间戳回退后，会先请求复位，再在处理线程安全点重置内部定位状态、缓存和局部地图，避免在回调里直接清 KD-tree 导致段错误。
- Open3D 检测到 `/Odometry_loc` 时间戳回退后，会清空当前 scan 队列，把 `/localization_3d_confidence` 置 0，然后等待新的 `/cloud_registered_1` 并重新初始化定位。
- RViz `2D Pose Estimate` 会先更新目标 `map -> base_link`，再请求 `/fastlio_localization_odom/reset_localization`，让漂掉的 Fast-LIO EKF/局部地图一起复位。
- 如果启用了 adapter 置信度门控，置信度低于 `localization_confidence_threshold`（默认 `0.65`）时会冻结 `/NAV_CMD`，避免定位没定上时继续执行旧速度。

重启或恢复时，日志里预期能看到：

```text
Reset localization state: ...
Odometry_loc timestamp moved backwards, scheduling relocalization reset
Reinitializing Open3D localization after lidar/odom restart
Waiting for cloud_registered_1 after relocalization reset...
Fast-LIO localization reset requested by service
```

验证命令：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry_loc
ros2 topic hz /cloud_registered_1
ros2 topic echo /localization_3d_confidence
ros2 topic echo /NAV_CMD
ros2 service list | grep reset_localization
```

判断标准：

- `/livox/lidar`、`/livox/imu` 恢复后，`/Odometry_loc` 和 `/cloud_registered_1` 应该重新连续输出。
- 如果启用了 adapter 置信度门控，`/localization_3d_confidence` 低于 `localization_confidence_threshold` 时，`/NAV_CMD` 应该保持零速度。
- `/localization_3d_confidence` 恢复到 `localization_confidence_threshold` 以上后，adapter 才会重新允许 Nav2 控制输出。
- 如果一直看到 `TF_NAN_INPUT` 或 `TF_DENORMALIZED_QUATERNION` 指向 `motion_link`，先确认运行的是重编译后的 `open3d_loc`，并完整重启 Fast-LIO、Open3D、Nav2 和 adapter。
- 如果 `fastlio_mapping` 仍然 `exit code -11`，优先确认已经完整重启定位 launch。旧进程不会吃到安全点复位逻辑。

rosbag 调试时必须让整条定位导航链路使用仿真时间，否则包重播或换包后，TF buffer 里会保留上一轮播放后半段的“未来 TF”，新一轮包开头的 TF 会被拒收并刷 `TF_OLD_DATA`。

启动定位导航时加 `use_sim_time:=true`：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml
```

然后播放 bag：

```bash
ros2 bag play /path/to/bag --clock
```

`fastlio_odom_bridge` 会在 `/clock` 或 `/Odometry_loc` 时间回退时清空自己的 TF buffer 并清速度历史。Nav2、RViz、`pointcloud_to_laserscan` 也需要通过 `use_sim_time:=true` 跟随 `/clock`，否则它们自己的 TF buffer 仍可能继续报 `TF_OLD_DATA`。
如果是远程笔记本单独开 RViz，也要用：

```bash
rviz2 -d ~/m20_nav3d.rviz --ros-args -p use_sim_time:=true
```

如果已经按上面方式启动但仍然刷 `TF_OLD_DATA`，先完整重启一次 launch 和 RViz，确认运行的是重编译后的节点。旧进程不会吃到 `fastlio_odom_bridge` 的时间回退清 buffer 逻辑。

## 推荐启动终端布局

Terminal 1: Livox

```bash
cd ~/liv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Terminal 2: Fast-LIO + Open3D + Nav2

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

Terminal 3: global path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/move/move/global_path_publisher.py
```

Terminal 4: pure pursuit subgoal

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/move/move/pure_pursuit.py --ros-args -r plan:=global_path
```

Terminal 5: RL local path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venv/m20_nav/bin/activate
python src/move/move/priest_rl_publisher_nav_cmd_fast.py
```

Terminal 6: adapter / debug

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py
```

调试终端：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

## 清理和重编译

只清 Fast-LIO 工作区：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
rm -rf build install log
```

然后重新按"完整构建"执行。

不要在 `m20_ws` 里重新加入 `fast_lio`、`fast_lio_map`、`open3d_loc`、`pcd2pgm` 的软链接。当前约定是：

- `fast_lio_ws`: 建图、定位、Nav2 位姿链路、当前验证用的 `move` / `RL2Path`。
- `m20_ws`: 原始工程备份和未迁移的机器人代码，不作为当前启动入口。
