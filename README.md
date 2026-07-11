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

这一节是**建图后的离线后处理**，不是启动导航。

Fast-LIO/PGO 保存出来的 `global_map.pcd` 是原始 3D 点云地图，可能存在轻微倾斜、离群点或高度基准不方便导航的问题。`prepare_3d_nav_map.py` 会把它整理成导航/定位更适合使用的 3D PCD：

- 输入：默认优先读取 `maps/fastlio/global_map.pcd`，没有它时读取 `maps/fastlio/m20_map_raw.pcd`
- 输出：默认写到 `maps/fastlio/m20_3d_map.pcd`
- 处理：应用 MID360 外参粗旋平、RANSAC 地面平面微调、离群点过滤
- 注意：输出仍然是 **3D PCD**，不是 2D 栅格地图

你本机当前已有：

```text
/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/global_map.pcd
```

本机运行脚本用这个 Python 环境，因为里面有 `open3d`：

```bash
cd /home/ubuntu/xlab/M20-loc-nav

/home/ubuntu/miniconda3/envs/m20_nav/bin/python \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py
```

这会读取：

```text
maps/fastlio/global_map.pcd
```

并生成：

```text
maps/fastlio/m20_3d_map.pcd
```

如果只是先试跑，不想覆盖正式输出：

```bash
cd /home/ubuntu/xlab/M20-loc-nav

/home/ubuntu/miniconda3/envs/m20_nav/bin/python \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --output /tmp/m20_3d_map_check.pcd
```

如果你明确要指定输入地图：

```bash
/home/ubuntu/miniconda3/envs/m20_nav/bin/python \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --input maps/fastlio/global_map.pcd \
  --output maps/fastlio/m20_3d_map.pcd
```

如果只想改写/过滤 PCD，不做旋平：

```bash
/home/ubuntu/miniconda3/envs/m20_nav/bin/python \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --skip-align \
  --output /tmp/m20_3d_map_no_align_check.pcd
```

如果确认地面识别正确，再使用 `--ground-zero` 把拟合地面平移到 `z=0`。建议先输出到临时文件检查：

```bash
/home/ubuntu/miniconda3/envs/m20_nav/bin/python \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --ground-zero \
  --output /tmp/m20_ground_zero_check.pcd
```

确认没问题后再覆盖正式地图：

```bash
/home/ubuntu/miniconda3/envs/m20_nav/bin/python \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --ground-zero \
  --output maps/fastlio/m20_3d_map.pcd
```

如果脚本提示 ground plane near vertical，说明当前点云姿态或地面选择不可靠，不要强行覆盖正式地图。

生成后快速确认：

```bash
ls -lh maps/fastlio/m20_3d_map.pcd
head -10 maps/fastlio/m20_3d_map.pcd
```

如果后续导航要使用这个处理后的地图，启动时把 `map_pcd` 指向它：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:=maps/fastlio/m20_3d_map.pcd \
  rviz:=true
```

## 启动 PCT Planner

PCT planner 已放进本仓库。本机路径和环境如下：

```text
workspace: /home/ubuntu/xlab/M20-loc-nav
ROS:       /opt/ros/jazzy
Livox:     /home/ubuntu/xlab/liv_ws/install
PCT venv:  /home/ubuntu/xlab/M20-loc-nav/.venv/m20_nav_jazzy
```

每个新终端先加载环境：

```bash
cd /home/ubuntu/xlab/M20-loc-nav

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/xlab/liv_ws/install/setup.bash
source /home/ubuntu/xlab/M20-loc-nav/install/setup.bash
```

先确认 PCT 包已安装、CuPy 可用：

```bash
ros2 pkg prefix pct_planner_ros2
ros2 pkg executables pct_planner_ros2

CUDA_LIBS="$(find .venv/m20_nav_jazzy/lib/python3.12/site-packages/nvidia \
  -type d -name lib -printf '%p:')"

LD_LIBRARY_PATH="${CUDA_LIBS}${LD_LIBRARY_PATH:-}" \
  .venv/m20_nav_jazzy/bin/python -c \
  "import cupy as cp; x=cp.arange(5); print(cp.__version__, int(cp.asnumpy(x.sum())))"
```

### 生成或查看 tomogram

Tomography 会把处理后的 3D 导航 PCD 转成 PCT planner 使用的 traversability tomogram。

当前默认输入地图来自“准备导航 PCD”这一步的输出，而不是原始 `global_map.pcd`：

```text
/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/m20_3d_map.pcd
```

链路是：

```text
global_map.pcd -> prepare_3d_nav_map.py -> m20_3d_map.pcd -> tomography -> m20_3d_map.pickle
```

参数文件是：

```text
src/pct_planner_ros2/config/tomography.yaml
```

关键参数：

```yaml
pcd_file: "/home/ubuntu/xlab/M20-loc-nav/maps/fastlio/m20_3d_map.pcd"
output_tomogram_name: "m20_3d_map"
```

启动 tomography 并打开 RViz：

```bash
ros2 launch pct_planner_ros2 m20_tomography_rviz.launch.py
```

只生成 tomogram，不启动 RViz：

```bash
ros2 launch pct_planner_ros2 tomography.launch.py
```

默认输出文件为：

```text
/home/ubuntu/xlab/M20-loc-nav/install/pct_planner_ros2/share/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle
```

输出文件名规则：

```text
<pct_root>/rsc/tomogram/<output_tomogram_name>.pickle
```

生成后确认：

```bash
ls -lh install/pct_planner_ros2/share/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle

.venv/m20_nav_jazzy/bin/python - <<'PY'
import pickle
p = "install/pct_planner_ros2/share/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle"
d = pickle.load(open(p, "rb"))
print("shape:", d["data"].shape)
print("resolution:", d["resolution"])
print("center:", d["center"])
print("slice_h0:", d["slice_h0"])
print("slice_dh:", d["slice_dh"])
PY
```

如果修改了 `src/pct_planner_ros2/config/tomography.yaml`，需要重新构建，否则
`ros2 launch` 仍会读取 install 里的旧 YAML：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/xlab/liv_ws/install/setup.bash

colcon build --symlink-install --packages-select pct_planner_ros2
source /home/ubuntu/xlab/M20-loc-nav/install/setup.bash
```

### 启动 PCT 全局规划

PCT planner 默认读取：

```text
tomogram_file: m20_3d_map
```

也就是上面生成的：

```text
install/pct_planner_ros2/share/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle
```

启动 PCT 全局规划，带 PCT 自己的 RViz：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py
```

只启动 planner，不启动 PCT RViz：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py launch_rviz:=false
```

默认 `start_source:=tf`，起点来自 `map -> base_link`。如果要让 PCT 起点来自
当前定位里程计：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  start_source:=odom \
  odom_topic:=/odom_body
```

如果要在单独 PCT RViz 调试时启用全局路径感知更新：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  global_path_perception_enabled:=true \
  global_path_perception_scan_topic:=/scan
```

如果要显式指定 tomogram：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  tomogram_file:=m20_3d_map \
  launch_rviz:=true
```

PCT 规划成功后应发布：

```bash
ros2 topic echo /pct_path --once
```

也可以手动发一个目标点测试：

```bash
ros2 topic pub --once /pct_goal_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 0.5}}"
```

如果没有 `/pct_path` 输出，优先检查：

```bash
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo map base_link
ls -lh install/pct_planner_ros2/share/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle
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
  start_livox:=false \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

### Fast-LIO 前端选择

主导航默认使用 `fast_lio` 作为 Fast-LIO 前端。当前测试结果是：

```text
fast_lio     + filter_size_surf=0.2, filter_size_map=0.3 可以用，实机实时性更好
fast_lio_map + filter_size_surf=0.2, filter_size_map=0.3 可以用，但实机反馈会卡
```

所以目前优先结论是：之前下楼梯定位飘，主要是定位参数太粗，尤其是
`filter_size_surf` 和 `filter_size_map`，不是某一个前端完全不能用。现在默认切回
`fast_lio`，优先保证前端实时性；`fast_lio_map` 作为 A/B 测试和备用前端保留。

默认前端：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

切到 `fast_lio_map` 前端做 A/B 测试：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true \
  fastlio_frontend:=fast_lio_map
```

只跑定位时也可以切：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_localization.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true \
  fastlio_frontend:=fast_lio_map
```

定位参数文件：

```text
src/m20_fastlio_nav/config/fastlio_localization_mid360.yaml
```

当前楼梯定位推荐值：

```yaml
filter_size_surf: 0.2
filter_size_map: 0.3
acc_cov: 0.2
gyr_cov: 0.2
```

`filter_size_surf=0.2` 会让当前帧点云保留更多点面约束，`filter_size_map=0.3` 保留局部地图细节但不至于太重；`acc_cov/gyr_cov=0.2` 当前实测比上一版更适合楼梯振动场景。更详细的前端差异、测试方法和保留建议见：

```text
docs/fastlio_frontend_notes.md
```

默认参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `fastlio_frontend` | `fast_lio` | Fast-LIO 前端包名；可切到 `fast_lio_map` 做 A/B 测试 |
| `global_path_topic` | `/pct_path` | pure pursuit 和 RL local path 使用的全局路径 |
| `start_pct_planner` | `true` | 默认随主导航启动 PCT planner |
| `pct_start_source` | `tf` | PCT 起点默认来自 TF `map -> base_link`，会随机器人位置更新 |
| `pct_replan_interval` | `1.0` | 每 1 秒检查是否需要重规划 |
| `pct_position_epsilon` | `0.2` | 起点或终点变化超过 0.2m 才重规划 |
| `pct_always_replan` | `true` | 主导航默认按 `pct_replan_interval` 定周期更新 `/pct_path`；动态感知只更新代价层，不单独触发即时重规划 |
| `pct_global_path_perception_enabled` | `true` | 主导航默认用 traversability 过滤后的 scan 触发 PCT 全局路径感知更新 |
| `pct_global_path_perception_scan_topic` | `/traversability_filtered_scan` | PCT 动态避障输入；楼梯/平台交界处由 traversability_layer 先过滤 |
| `pct_global_path_perception_width` | `6.0` | PCT 全局路径感知窗口宽度，覆盖机器人前后左右近场障碍 |
| `pct_global_path_perception_height` | `6.0` | PCT 全局路径感知窗口高度，覆盖机器人前后左右近场障碍 |
| `pct_global_path_perception_inflation_radius` | `0.60` | PCT 动态障碍膨胀半径，对齐 2D local costmap 的近场避障宽度 |
| `pct_global_path_perception_cost_scaling_factor` | `5.0` | 动态代价衰减速度，对齐 Nav2 costmap inflation 的默认调法 |
| `pct_global_path_perception_persistence` | `5.0` | 动态观测兜底保留时间；主要清除机制是 raytrace clearing |
| `pct_global_path_perception_raytrace_enabled` | `true` | 使用 LaserScan 射线清除动态层自由空间，行为更接近 Nav2 obstacle_layer |
| `pct_global_path_perception_raytrace_max_range` | `0.0` | raytrace 最大距离；`0.0` 表示自动使用感知窗口和 scan `range_max` 的较小值 |
| `pct_global_path_perception_raytrace_max_rays` | `360` | 每次最多处理的清除射线数，限制 Python 动态层计算量 |
| `scan_topic` | `/scan` | costmap 与 RL 使用同一个 LaserScan |
| `output_odom_topic` | `/odom_body` | DWB、RL 和 adapter 使用的 body-plane odom |
| `start_pure_pursuit` | `true` | 默认启动 pure pursuit |
| `start_rl_local_path` | `true` | 默认启动 RL local path |
| `start_adapter` | `true` | 默认启动 `/local_path -> /NAV_CMD` 适配 |
| `body_scan_min_height` | `-0.1` | 从 body 点云生成 scan 的最低高度 |
| `body_scan_max_height` | `0.55` | 从 body 点云生成 scan 的最高高度 |

PCT 全局路径感知更新在 C++ core 中维护，不会修改或重新生成完整 tomogram；离线 3D tomogram 仍是长期地形地图。主导航默认不再直接使用原始 `/scan`，而是使用 `traversability_layer` 发布的 `/traversability_filtered_scan`。这个 scan 来自原始 `/scan`，但只有 endpoint 落在 traversability 明确判为高代价/不可通行的格子时才保留；可通行、未知、无地面区域都会置为 `inf`。当前 `local_costmap` 是 `5m x 5m`，filtered scan 的有效范围先受 local costmap 限制。PCT 动态层按 Nav2 costmap 思路维护 source grid：有限 hit beam 只 mark 障碍源点，`inf`/远距离 beam 或 hit 前方自由空间会 raytrace clear 障碍源点；source 变化后再统一重算 inflation cost。`persistence` 只是兜底超时清除。

PCT 动态层保留静态障碍跳过保护，避免静态 tomogram 已经包含的墙体又作为动态障碍重复膨胀：

- `global_path_perception_skip_static_obstacles=true`：静态 tomogram cost 已经高于阈值的点不会重复写入动态层。`global_path_perception_static_skip_cost=-1.0` 表示自动使用 `a_star_cost_threshold`，主导航里实际是 45.0。
- `global_path_perception_path_corridor_radius=0.0`：默认不按已有路径裁剪动态障碍，行为更接近 Nav2 global costmap；需要临时限制路径附近障碍时再手动调大。

如需对比原始 `/scan`：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  pct_global_path_perception_scan_topic:=/scan
```

修改 PCT C++ core 后需要重新构建 pybind：

```bash
./src/pct_planner_ros2/scripts/build_pct_core.sh
colcon build --packages-select pct_planner_ros2 m20_fastlio_nav
```

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
ros2 node list | grep -E "global_localization|fastlio|initialpose"
ros2 topic echo /localization_3d_confidence --once
ros2 topic echo /baselink2map --once
ros2 topic echo /odom2map --once
ros2 topic echo /Odometry_loc --once
ros2 topic echo /odom_body --once
ros2 topic echo /map_3d --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom_body base_link
```

手动拉全局初始位姿：

```bash
# RViz 2D Pose Estimate 和 3D Initial Pose marker 都发布 /initialpose。
# /initialpose 的语义是目标 map -> base_link 初始位姿。
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

RViz 里默认还有一个 `3D Initial Pose` interactive marker。用 `Interact` 工具拖动或旋转橙色球体，拖动过程中会在播放 bag 前持续发布 `/initialpose`，松开鼠标时也会发布一次。这样可以先拖初始位姿再播放 rosbag，避免第一帧定位回到原点。Open3D 收到 `/initialpose` 后会更新 `map -> odom`；如果定位已经初始化，还会请求 Fast-LIO 前端 `/reset_localization`。如果只想用命令行，不启动这个 marker：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py start_initialpose_3d_marker:=false
```

如果 `base_link` 会动，但沿着旧 bag 的倾斜坐标跑出旋平后的地图，优先按下面查：

```bash
ros2 node list | grep global_localization
ros2 topic echo /localization_3d_confidence --once
ros2 topic echo /baselink2map --once
ros2 topic echo /odom2map --once
ros2 run tf2_ros tf2_echo map odom
```

- 没有 `/global_localization_node`：Open3D 全局定位没有启动，检查主 launch 是否正常启动。
- 没有 `/baselink2map` 或 `/odom2map`：Open3D 没有收到 `/Odometry_loc` 或 `/cloud_registered_1`。
- `map -> odom` 不变或明显不对：先暂停 bag，在 RViz 里用 `3D Initial Pose` 把橙色球拖到 bag 起点，再继续播放。
- 旋平地图下不要继续使用带大 pitch 的旧初始姿态；默认地图应为 `maps/fastlio/m20_3d_map.pcd`。

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

## 常见启动报错与解决

### `Package 'pct_planner_ros2' not found`

典型报错：

```text
Package 'pct_planner_ros2' not found
```

源码包位于 `src/pct_planner_ros2`，但只有源码存在还不够；它必须被
`colcon` 构建到 `install/pct_planner_ros2`，并且当前终端必须在构建后重新
加载工作区环境。

先确认源码包能够被 `colcon` 识别：

```bash
cd /home/ubuntu/xlab/M20-loc-nav
colcon list | grep '^pct_planner_ros2'
```

单独构建 PCT 包：

```bash
source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash

colcon build --symlink-install --packages-select pct_planner_ros2
```

构建完成后，必须在执行 launch 的同一个终端重新 source。已经打开的终端不会
自动刷新 `AMENT_PREFIX_PATH`：

```bash
source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash
source ~/xlab/M20-loc-nav/install/setup.bash
```

验证包和可执行入口：

```bash
ros2 pkg prefix pct_planner_ros2
ros2 pkg executables pct_planner_ros2
```

正常情况下，第一条命令应输出：

```text
/home/ubuntu/xlab/M20-loc-nav/install/pct_planner_ros2
```

随后再启动：

```bash
ros2 launch pct_planner_ros2 m20_tomography_rviz.launch.py
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py
```

如果报错中的 `searching:` 路径列表不包含
`/home/ubuntu/xlab/M20-loc-nav/install/pct_planner_ros2`，说明当前终端仍在
使用构建前的环境，需要再次执行上面的三个 `source` 命令。

### `ModuleNotFoundError: No module named 'cupy'`

PCT tomography 使用 CuPy 在 NVIDIA GPU 上计算。ROS Jazzy 节点运行在项目的
Python 3.12 环境中，安装兼容当前 NumPy/SciPy 的固定版本：

```bash
cd /home/ubuntu/xlab/M20-loc-nav

.venv/m20_nav_jazzy/bin/python -m pip install \
  -r src/pct_planner_ros2/requirements-cupy.txt
```

当前固定版本为 `cupy-cuda12x==13.6.0`。不要直接安装最新 CuPy 14.x，因为它
会把 NumPy 1.26 升级到 2.x，可能破坏当前 SciPy、Open3D 和 ROS Python 环境。

安装后重新构建并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash

colcon build --symlink-install --packages-select pct_planner_ros2 m20_fastlio_nav
source ~/xlab/M20-loc-nav/install/setup.bash
```

PCT launch 会自动把 `.venv/m20_nav_jazzy` 的 site-packages 和
`site-packages/nvidia/*/lib` 加入 `PYTHONPATH`、`LD_LIBRARY_PATH`，无需手动
导出 `libnvrtc.so.12` 路径。

验证 CuPy：

```bash
CUDA_LIBS="$(find .venv/m20_nav_jazzy/lib/python3.12/site-packages/nvidia \
  -type d -name lib -printf '%p:')"

LD_LIBRARY_PATH="${CUDA_LIBS}${LD_LIBRARY_PATH:-}" \
  .venv/m20_nav_jazzy/bin/python -c \
  "import cupy as cp; x=cp.arange(5); print(cp.__version__, int(cp.asnumpy(x.sum())))"
```

正常输出应包含 CuPy 版本和求和结果 `10`。

## Jie/OctoMap 归档

Jie/OctoMap 方案没有放在当前主线分支里。需要查看当时的 OctoMap 编辑 GUI、Jie planner launch 和相关命令时：

```bash
git switch archive/jie-octomap-global-planner
```

回到当前 PCT 主线：

```bash
git switch feature/3d-body-plane-nav
```
