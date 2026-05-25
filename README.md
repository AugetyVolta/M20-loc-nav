# M20 MID360 Fast-LIO 建图定位导航

这个工作空间现在分成两条明确链路：

- 建图：`fast_lio_map` 前端 + `slam_mapping` 后端 PGO
- 定位：`fast_lio` + `open3d_loc` + `fastlio_odom_bridge`

建图和定位已经解耦。`src/fast_lio` 只保留定位稳定版，不参与建图。

## 工作空间

```text
/mnt/nvme/workspace/fast_lio_ws/
├── README.md
├── level_pcd.py
├── maps/
│   └── fastlio/
│       ├── m20_map.pcd
│       ├── m20_map_leveled.pcd
│       ├── global_map.pcd
│       ├── sc_database.txt
│       ├── m20_2d_map.pgm
│       └── m20_2d_map.yaml
└── src/
    ├── fast_lio/          # 定位前端
    ├── fast_lio_map/      # 建图前端
    ├── slam_mapping/      # 建图后端 PGO
    ├── open3d_loc/        # 全局点云定位
    ├── pcd2pgm/           # 3D PCD 转 2D 栅格
    └── m20_fastlio_nav/   # launch、配置、bridge
```

## 环境

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source /mnt/nvme/workspace/fast_lio_ws/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0
```

`LD_PRELOAD` 现在仍然建议保留，不然 PCL 相关节点可能因为系统里的旧 `libusb` 起不来。

## 编译

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash

colcon build --packages-select fast_lio fast_lio_map slam_mapping open3d_loc pcd2pgm m20_fastlio_nav \
  --symlink-install \
  --executor sequential \
  --cmake-clean-cache
```

这台机器的 `CMake 4.3 + OpenMPI` 有兼容性坑，我已经把 `fast_lio_map` 和 `slam_mapping` 的工程侧绕过做进仓库了，所以直接按上面编即可。

## 1. 建图

建图入口还是原来的：

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

这个 launch 现在内部会同时起：

- `fast_lio_map/fastlio_mapping`
- `slam_mapping/alaserPGO`

建图阶段会有两类产物：

- 前端原始图：`map_pcd` 指向的文件，默认是 `maps/fastlio/m20_map.pcd`
- 后端 PGO 产物：
  - `maps/fastlio/global_map.pcd`
  - `maps/fastlio/sc_database.txt`

如果你只看最终建图效果，优先关注 `global_map.pcd`。

建图过程中可以显式保存：

- 保存前端原始图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

- 保存后端 PGO 图：

```bash
ros2 service call /save_pgo_map std_srvs/srv/Trigger {}
```

其中 `/save_pgo_map` 会写：

- `maps/fastlio/global_map.pcd`
- `maps/fastlio/sc_database.txt`

即使不手动调用，正常 `Ctrl+C` 结束建图时，`slam_mapping` 也会自动保存一次后端 PGO 图。

## 2. 3D 地图摆正

因为雷达安装有俯仰，建好的 PCD 通常要摆正后再用于定位和 2D 栅格转换。

如果你要处理原始前端图：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
python3 level_pcd.py \
  --input /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map.pcd \
  --output /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

如果你要处理后端 PGO 图：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
python3 level_pcd.py \
  --input /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/global_map.pcd \
  --output /mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd
```

实际建议：优先把 `global_map.pcd` 摆正后作为最终定位地图。

## 3. 3D 地图转 2D 地图

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

- `maps/fastlio/m20_2d_map.pgm`
- `maps/fastlio/m20_2d_map.yaml`

## 4. 纯定位

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_fastlio_localization.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  rviz:=true
```

这条链内部是：

- `fast_lio` 输出 `/Odometry_loc`
- `open3d_loc` 用 3D 地图做全局校正
- `fastlio_odom_bridge` 负责 `/odom` 和 Nav2 所需 TF

如果播包测试：

```bash
ros2 bag play <bag目录> --clock
```

同时定位 launch 加：

```bash
use_sim_time:=true
```

如果切换 bag 之间停顿较大，现在已经加了自动 reset 逻辑；必要时也可以手动 reset：

```bash
ros2 service call /fastlio_localization_odom/reset_localization std_srvs/srv/Trigger {}
```

## 5. 定位 + 导航

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
source ~/liv_ws/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  map_pcd:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_map_leveled.pcd \
  map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml \
  rviz:=true
```

这条链会起：

- `m20_fastlio_localization.launch.py`
- `nav2_map_server`
- `nav2_bringup/navigation_launch.py`

## TF 和话题

建图核心话题：

- `/Odometry`
- `/cloud_registered_body`
- `/aft_pgo_odom`
- `/aft_pgo_path`
- `/aft_pgo_map`

定位核心话题：

- `/Odometry_loc`
- `/odom`
- `/scan`

定位核心 TF：

- `map -> odom`
- `map -> odom_nav`
- `odom_nav -> base_footprint`
- `odom -> base_link`

## 当前入口总结

建图：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_mapping.launch.py map_pcd:=... rviz:=true
```

转 2D：

```bash
ros2 launch m20_fastlio_nav m20_pcd2pgm_save.launch.py pcd_file:=... output_map:=...
```

纯定位：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_localization.launch.py map_pcd:=... rviz:=true
```

定位+导航：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py map_pcd:=... map:=... rviz:=true
```
