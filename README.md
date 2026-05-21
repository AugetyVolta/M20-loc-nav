# M20 MID360 Fast-LIO2 建图定位导航系统

这个工作空间用于把 MID360 的 Fast-LIO2 建图/定位结果接入 Nav2，并运行当前拷贝到本工作区的
RL local path + Nav2 DWB adapter。

当前工作空间是独立复制出来的，不依赖 `m20_ws/src` 里的软链接。

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
                              ├── 建图: 保存 /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd
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
  controller_server -> /cmd_vel_nav
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
├── level_pcd.py                  # PCD 摆正脚本（补偿 MID360 安装倾角）
├── maps/
│   └── fastlio/
│       ├── m20_map.pcd           # Fast-LIO2 生成的 3D 点云地图（原始）
│       ├── m20_map_leveled.pcd   # 摆正后的 3D 点云地图（3D定位 + pcd2pgm 共用）
│       ├── m20_2d_map.pgm        # pcd2pgm + map_saver 生成
│       └── m20_2d_map.yaml       # Nav2 map_server 使用
└── src/
    ├── fast_lio/                 # 定位用 Fast-LIO2，输出 /Odometry_loc
    ├── fast_lio_map/             # 建图用 Fast-LIO2，负责保存 PCD
    ├── open3d_loc/               # Open3D 点云地图定位，发布 map -> odom
    ├── pcd2pgm/                  # PCD 转 Nav2 2D OccupancyGrid
    └── m20_fastlio_nav/          # M20 专用 launch、参数、odom bridge
```

## 包说明

| 包 | 来源/用途 | 运行阶段 |
|---|---|---|
| `fast_lio_map` | luckrobot 的 Fast-LIO2 建图包，用 `/map_save` 保存 PCD | 建图 |
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
src/m20_fastlio_nav/config/open3d_localization_m20.yaml
src/m20_fastlio_nav/config/pcd2pgm_m20.yaml
src/m20_fastlio_nav/config/nav2_dwb_fastlio.yaml
src/m20_fastlio_nav/config/m20_nav3d.rviz      # 定位+导航全景 RViz 配置

src/m20_fastlio_nav/m20_fastlio_nav/fastlio_odom_bridge.py

level_pcd.py                     # PCD 坐标系摆正脚本
```

## Fast-LIO 配置要点

MID360 必须使用 `lidar_type: 4`（MID360），不能用 AVIA（1）。否则 Fast-LIO 会错误地订阅 `CustomMsg` 而非 `PointCloud2`，导致 `/livox/lidar` 数据类型不匹配，收不到点云。

| 参数 | 正确值 | 说明 |
|---|---:|---|
| `preprocess.lidar_type` | `4` | MID360 = 4, AVIA = 1, VELO16 = 2, OUST64 = 3 |
| `common.time_sync_en` | `false` | 关闭 IMU-LiDAR 时间同步，MID360 时间戳无对齐 |
| `common.lid_topic` | `/livox/lidar` | Livox 驱动话题 |
| `common.imu_topic` | `/livox/imu` | Livox IMU 话题 |

当前 **建图和定位两个配置文件已经改为正确值**，不要再改回旧值。

## 外参

### MID360 安装姿态

`base_link` 是水平的，但 MID360 **前倾约 28°** 安装在机器狗上：

```text
translation: [0.32713234, 0.01413551, 0.31238696]
quaternion:  [-0.00394028, 0.24367785, 0.00970223, 0.96979969]   ->  Euler: roll -0.2°, pitch 28.2°, yaw 1.1°
```

这个倾角导致建图坐标系（`camera_init`）的 Z 轴不是真正垂直的。因此 **PCD 必须摆正后才能给 pcd2pgm 生成 2D 地图**（见 Step 3），否则 2D 地图是歪的。

定位阶段的实时 `/scan` 使用 `pointcloud_to_laserscan(target_frame=base_footprint)`，会通过 TF 自动转平，不受影响。

### 外参修改范围

这个外参同时用于：

- `base_link -> livox_frame` 静态 TF
- `base_link -> imu_link` 静态 TF
- `fastlio_odom_bridge` 内部同时计算 `map -> odom_nav`、`odom_nav -> base_footprint` 和 `odom -> base_link`
- `level_pcd.py` 摆正脚本

如果后面重新标定 MID360，必须同步修改：

```text
src/m20_fastlio_nav/launch/m20_fastlio_localization.launch.py
src/m20_fastlio_nav/launch/m20_fastlio_mapping.launch.py
src/m20_fastlio_nav/m20_fastlio_nav/fastlio_odom_bridge.py
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

colcon build --symlink-install --executor sequential \
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

注意：当前 `fast_lio_map` 是 Fast-LIO2 增量建图，不是带 pose graph 的闭环 SLAM。环形走廊即使最后走回起点，也不会自动做回环约束；如果中途累计漂移，保存出来的 PCD 会带开口或重影。需要严格闭环地图时，应改用带回环优化的建图链路（例如 LIO-SAM/Cartographer/其他 pose graph 后处理），再把优化后的 PCD 和 2D map 接入本导航链路。

建图阶段关注：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_registered_body
ros2 service list | grep map_save
```

保存 PCD：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

默认保存到：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd
```

确认文件存在：

```bash
ls -lh /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd
```

## Step 3: 摆正 PCD（补偿 MID360 安装倾角）

MID360 前倾 ~28°，导致 PCD 在 `camera_init` 坐标系里是歪的。直接用 pcd2pgm 沿 Z 轴切片会产生错误倾斜的 2D 地图。

`level_pcd.py` 用已知外参旋转矩阵将点云从倾斜方向转到接近 `base_link` 的水平方向：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ~/venv/m20_nav/bin/activate
python3 level_pcd.py \
  maps/fastlio/m20_map.pcd \
  maps/fastlio/m20_map_leveled.pcd
```

输出：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

**注意**：摆正后的 PCD 同时用于 3D 定位和 2D 地图生成，保证两者坐标系一致。原始 PCD 保留备用。

## Step 4: 生成 Nav2 2D 地图

用**摆正后的 PCD** 生成 2D occupancy grid：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
ros2 launch m20_fastlio_nav m20_pcd2pgm_save.launch.py \
  pcd_file:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
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

重要参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `map_resolution` | `0.05` | 2D 地图分辨率 |
| `thre_z_min` | `-0.3` | 参与投影的最低高度 |
| `thre_z_max` | `0.35` | 参与投影的最高高度 |
| `thre_radius` | `0.8` | 点云膨胀/聚合半径 |
| `thres_point_count` | `10` | 判定占据的点数阈值 |

如果生成的 2D 地图墙太粗或障碍太多，优先调 `thre_z_min/thre_z_max` 和 `thres_point_count`。

## Step 5: 启动 Fast-LIO 定位 + Nav2

确保已经有：

```text
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd          # 原始 PCD（给 Open3D）
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

这个 launch 会启动：

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

## Step 6: 启动 RL local path 和 DWB adapter

Nav2 启动后，再按当前验证过的方式运行 RL 路径发布和 adapter。当前拷贝到本工作区的 `move` / `RL2Path` 默认按 2D 链路运行：`global_path(map)` -> `local_path(base_footprint)` -> adapter 输出到 `odom_nav`。

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/move/move/global_path_publisher.py
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

python src/move/move/priest_mppi_adapter_nav_cmd.py
```

这个 adapter 会订阅 `/localization_3d_confidence`。当 Open3D 还没定位成功、正在重初始化，或者置信度低于默认阈值 `0.65` 时，adapter 会：

- 暂停给 Nav2 发送新的 `FollowPath` goal
- 取消已经激活的 `FollowPath` goal
- 持续向 `/NAV_CMD` 发布零速度，避免定位没锁住时底盘继续执行旧速度

恢复定位后，如果 `/localization_3d_confidence >= 0.65`，adapter 会自动恢复转发 `/cmd_vel` 到 `/NAV_CMD`。如果 1.5 秒内收不到新的 `/localization_3d_confidence`，adapter 也会按定位失效处理并冻结输出。

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
ros2 node list
ros2 lifecycle nodes
```

期望现象：

- `/Odometry_loc` 有稳定输出（几秒内就应该有数据）。
- `/odom` 有稳定输出，`header.frame_id` 是 `odom_nav`，`child_frame_id` 是 `base_footprint`。
- `tf2_echo map odom_nav` 能持续输出，而且基本只有 yaw。
- `/scan` 有 LaserScan 数据。
- Nav2 lifecycle 节点处于 `active`。

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
| `/cmd_vel_nav` | `Twist` | Nav2 controller | 后续 adapter/底盘 |
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
- `/odom` 来自 Fast-LIO bridge，`header.frame_id` 是 `odom_nav`。
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
| `voxelsize_coarse` | `0.04` | 粗地图体素 |
| `voxelsize_fine` | `0.25` | 精配准体素 |
| `threshold_fitness` | `0.45` | 定位接受阈值 |
| `loc_frequence` | `2.0` | Open3D 定位频率 |
| `confidence_loc_th` | `0.65` | 置信度阈值 |
| `reset_fastlio_on_initialpose` | `true` | RViz `2D Pose Estimate` 后同时请求重置 Fast-LIO |
| `fastlio_reset_service` | `/fastlio_localization_odom/reset_localization` | Fast-LIO 安全复位服务 |

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

**修复**：确认 `fastlio_localization_mid360.yaml` 中 `preprocess.lidar_type: 4`。

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

**原因 A**（不水平）：MID360 安装有倾角，PCD 坐标系本身是歪的。必须先跑 Step 3 用 `level_pcd.py` 摆正，再用摆正后的 PCD 生成 2D 地图。

**原因 B**（障碍过多/过少）：调整 `pcd2pgm_m20.yaml` 中的参数：

- `thre_z_min` / `thre_z_max`：控制切片高度范围
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
ros2 topic echo /cmd_vel_nav
```

如果 `/cmd_vel_nav` 已经不稳，优先看 Nav2 controller/costmap 负载。如果稳定但底盘不稳，再看 adapter 或底盘控制接口。

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
- adapter 在置信度低于 `0.65` 时会冻结 `/NAV_CMD`，避免定位没定上时继续执行旧速度。

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
- `/localization_3d_confidence` 低于 `0.65` 时，`/NAV_CMD` 应该保持零速度。
- `/localization_3d_confidence` 恢复到 `0.65` 以上后，adapter 才会重新允许 Nav2 控制输出。
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

Terminal 4: RL local path

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venv/m20_nav/bin/activate
python src/move/move/priest_rl_publisher_nav_cmd_fast.py
```

Terminal 5: adapter / debug

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python src/move/move/priest_mppi_adapter_nav_cmd.py
```

调试终端：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel_nav
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
