# M20 MID360 3D 导航

当前主线分支：`feature/3d-body-plane-nav`

当前方案使用仓库内置的 PCT planner 输出 3D 全局路径，本仓库负责 Fast-LIO/Open3D 定位、PCT 全局规划、机体平面局部控制和底盘输出。Jie/OctoMap 全局规划方案已单独归档到分支 `archive/jie-octomap-global-planner`，归档提交为 `2907d0b`。

## 总体链路

```text
MID360 点云/IMU
  -> Fast-LIO 建图或定位
  -> Open3D 全局定位提供 map -> odom
  -> fastlio_odom_bridge 输出 /odom_body，并发布 map -> odom_body 与 odom -> base_link
  -> pointcloud_to_laserscan 输出原始 /scan（调试）
  -> local_costmap 选择 traversability_layer 或 tomogram_filter_layer
  -> 选中的插件输出 /traversability_filtered_scan
  -> pct_planner_ros2 输出 /pct_path
  -> pure_pursuit 在 3D 路径上按 1.8m 前视距离发布 /subgoal
  -> priest_rl_publisher_nav_cmd_fast 根据 /subgoal 和 /traversability_filtered_scan 发布 /local_path
  -> DWB 跟踪 /local_path 输出 /cmd_vel
  -> adapter 输出 /NAV_CMD
```

关键话题：

| 话题 | 说明 |
|---|---|
| `/Odometry_loc` | Fast-LIO 定位原始输出 |
| `/odom_body` | 局部控制使用的机体平面 odom |
| `/scan` | 原始 LaserScan，用于 RViz 和过滤前后对比 |
| `/traversability_filtered_scan` | 当前选中 costmap 过滤插件的输出，供 ObstacleLayer 和 RL local path 使用；PCT 动态层直接订阅 `/scan` |
| `/traversability_tomogram` | `tomogram_filter_layer` 可选发布的 transient-local 静态表面，仅供 RViz 调试 |
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

colcon build --symlink-install --packages-select \
  pct_planner_ros2 traversability_layer tomogram_filter_layer move m20_fastlio_nav
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
脚本会把实际用于旋平 PCD 的累计旋转同步写入
`src/m20_fastlio_nav/config/open3d_localization_m20.yaml` 的 `initialpose` 后三项
`roll, pitch, yaw`，让 Open3D 定位启动时先带上地图旋平角，而不是完全依赖 ICP 在线估计。

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

"${M20_NAV_CUPY_PYTHON}" \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py
```

如果只是试输出临时 PCD，不想修改 Open3D 定位配置，加：

```bash
"${M20_NAV_CUPY_PYTHON}" \
  src/m20_fastlio_nav/m20_fastlio_nav/prepare_3d_nav_map.py \
  --output /tmp/m20_3d_map_check.pcd \
  --no-update-open3d-initialpose
```

如果确认地面识别正确，再使用 `--ground-zero` 把地图高度归零：

```bash
source ./source_m20_nav.sh
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

如果要在单独 PCT RViz 调试时启用全局路径感知更新：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  global_path_perception_enabled:=true \
  global_path_perception_scan_topic:=/scan
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
  start_livox:=false \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

上下楼梯步态切换默认开启。需要临时关闭时，加：

```bash
pct_stair_gait_udp_enabled:=false
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
| `pct_always_replan` | `true` | 主导航默认按 `pct_replan_interval` 定周期更新 `/pct_path`；动态感知只更新代价层，不单独触发即时重规划 |
| `pct_global_path_perception_enabled` | `true` | 主导航平地模式默认用原始 scan 更新 PCT 动态障碍层；楼梯模式关闭 |
| `pct_global_path_perception_scan_topic` | `/scan` | 同一帧先 raytrace clear，再 mark 有限 endpoint |
| `pct_global_path_perception_min_range` | `0.15` | 与 `/scan.range_min` 对齐；不再按机器人半径或 footprint 额外过滤有限回波 |
| `pct_global_path_perception_width` | `8.0` | PCT 动态感知窗口 X 向尺寸，以机器人为中心前后各约 4m |
| `pct_global_path_perception_height` | `8.0` | PCT 动态感知窗口 Y 向尺寸，以机器人为中心左右各约 4m |
| `pct_global_path_perception_inflation_radius` | `1.0` | PCT 动态障碍总代价膨胀半径；硬核心外侧给 A* 渐变避障代价 |
| `pct_global_path_perception_inscribed_radius` | `0.45` | PCT 动态障碍硬核心半径；半径内的动态障碍格子在 A* 中直接不可通行 |
| `pct_global_path_perception_cost_scaling_factor` | `5.0` | 动态软膨胀代价衰减速度，数值越大衰减越快，避免路径被软代价推得过远 |
| `pct_global_path_perception_persistence` | `5.0` | 动态观测兜底保留时间；主要清除机制是 raytrace clearing |
| `pct_global_path_perception_raytrace_enabled` | `true` | 使用 LaserScan 射线清除动态层自由空间，行为更接近 Nav2 obstacle_layer |
| `pct_global_path_perception_raytrace_max_range` | `0.0` | raytrace 最大距离；`0.0` 表示自动使用感知窗口和 scan `range_max` 的较小值 |
| `pct_global_path_perception_raytrace_max_rays` | `360` | 每次最多处理的清除射线数，限制 Python 动态层计算量 |
| `global_path_perception_height_tolerance` | `0.75` | 同一物理地面上的等高 tomogram layer 同时 mark/clear，避免 A* 从重叠层穿过障碍 |
| `pct_local_replan_enabled` | `true` | 保留静态全局参考线，只重规划机器人到前方锚点并平滑衔接后缀 |
| `pct_local_replan_forward_distance` | `10.0` | 基础前视距离；内部额外联合优化 `1.0m` 参考重叠段并约束末端切线，A* 不限制左右范围 |
| `scan_topic` | `/scan` | 原始 LaserScan，由 pointcloud_to_laserscan 输出，保留用于对比和调试 |
| `rl_scan_topic` | `/traversability_filtered_scan` | RL/PRIEST local path 使用的过滤后 LaserScan |
| `local_costmap.plugins` | `traversability_layer` | 当前默认使用实时 traversability 过滤；可在 `nav2_dwb_body_plane.yaml` 中切换为静态 tomogram 过滤 |
| `output_odom_topic` | `/odom_body` | DWB、RL 和 adapter 使用的 body-plane odom |
| `start_pure_pursuit` | `true` | 默认启动 pure pursuit |
| `heading_change_guard_stair_only` | `true` | pure pursuit 大转角保护只在 `/pct_stair_state=stair_up/stair_down` 时生效，平地允许原地掉头 |
| `start_rl_local_path` | `true` | 默认启动 RL local path |
| `start_adapter` | `true` | 默认启动 `/local_path -> /NAV_CMD` 适配 |
| `adapter_path_timeout` | `1.2` | 局部路径超过 1.2 秒未更新才取消 FollowPath，允许一次推理延迟但不会长期沿旧路径运动 |
| `pct_stair_down_gait_param` | `4099` | 下楼默认切到标准楼梯步态；需要测试敏捷楼梯步态时改为 `12291` |
| `body_scan_min_height` | `-0.1` | 从 body 点云生成 scan 的最低高度 |
| `body_scan_max_height` | `0.8` | 从 body 点云生成 scan 的最高高度 |

### Local costmap scan 过滤插件

PCT 全局路径感知更新在 C++ core 中维护，不会修改或重新生成完整 tomogram；离线 3D tomogram 仍是长期地形地图。scan 过滤实现不再由 launch 参数和独立进程切换，而是由 `src/m20_fastlio_nav/config/nav2_dwb_body_plane.yaml` 的 `local_costmap.plugins` 直接选择：

```yaml
# 当前默认：实时 traversability cost 过滤，不依赖离线 tomogram
plugins: ["traversability_layer", "inflation_layer"]

# 可选：静态 PCT XYZ 表面过滤
# plugins: ["tomogram_filter_layer", "obstacle_layer", "inflation_layer"]
```

只允许启用其中一个过滤插件，因为二者都会发布 `/traversability_filtered_scan`。Nav2 只实例化 `plugins` 列表中的过滤插件，另一个不会启动；旧的独立 `tomogram_scan_filter_node` 已删除。

- `traversability_layer` 自己计算并写入 local costmap，所以默认列表不再重复加载 `ObstacleLayer`；它发布 filtered scan 供 RL local path 使用，PCT 动态层直接使用原始 `/scan`。
- `tomogram_filter_layer` 只生成 filtered scan，不直接写 master costmap，所以 tomogram 列表必须同时保留 `ObstacleLayer`。

tomogram 插件启动时直接读取 `nav2_dwb_body_plane.yaml` 中配置的 `.surface.pcd`，不再订阅 PCT planner。主 launch 不会自动拼接文件路径或覆盖 cost 阈值；切换 `pct_tomogram_file` 时，需要手动把 `tomogram_surface_file` 改成对应的同名 PCD，并让 `traversable_cost_max` 与 `pct_a_star_cost_threshold` 一致。当前三项对应关系是：

```text
pct_tomogram_file=m20_3d_map
tomogram_surface_file=.../m20_3d_map.surface.pcd
pct_a_star_cost_threshold=traversable_cost_max=45.0
```

表面文件或 TF 未就绪时不会发布未过滤 scan，避免楼梯地面混入 costmap。`/traversability_tomogram` 现在只是插件可选发布的 RViz 调试点云，关闭它不会影响过滤。

新生成 tomogram 会同时生成 `.pickle` 和 `.surface.pcd`。已有地图补生成：

```bash
ros2 run pct_planner_ros2 pct_export_tomogram_surface m20_3d_map
```

新拉取代码后首次编译：

```bash
colcon build --symlink-install --packages-select \
  tomogram_filter_layer traversability_layer pct_planner_ros2 m20_fastlio_nav
source install/setup.bash
```

以后只修改 `nav2_dwb_body_plane.yaml` 的插件选择或过滤参数时，重启导航即可，不需要重新编译。启动指令不增加过滤模式参数：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

运行时可确认只有一个 filtered scan 发布者：

```bash
ros2 topic info /traversability_filtered_scan -v
ros2 topic echo /traversability_tomogram --once --field width
```

PCT 动态层按 Nav2 costmap 语义维护：原始 `/scan` 的同一帧先 raytrace clear，再 mark 有限 endpoint。障碍源为 `254`，内切区为 `253`，外圈按 `cost_scaling_factor` 指数衰减。A* 把 `253/254` 当硬障碍，优化器使用同一动态代价的梯度。同一物理地面的重叠 tomogram layer 会在高度容差内一起更新，防止 A* 换层穿过动态障碍；不同楼层不会互相污染。楼梯状态下 PCT 动态层关闭。

`254/253` 是动态层内部保留的 Nav2 障碍标签，不会直接写进 PCT 的 `0..50` cost。软代价按 `PCT峰值 * Nav2代价 / 254` 换算；当前 `a_star_cost_threshold=45`、自动动态峰值为 `50`。A* 先无条件拒绝 `253/254`，再使用映射后的软代价参与搜索。

动态更新只遍历活动 source 和其膨胀格，旧 source 由同帧 raytrace 或 `persistence` 清除，不再按参考路径走廊裁剪。路径规划保留静态全局参考线，每个周期以基础前视距离选择局部段，再额外联合优化 `1.0m` 静态参考重叠段。重叠段末端位置和切线与静态后缀对齐后再拼接，避免 RViz 中出现接缝尖角；A* 可以在完整 tomogram 范围内绕行。

近距离障碍输入下限与 `/scan.range_min=0.15m` 对齐，不再按机器人半径或 footprint 额外过滤、清空动态障碍点。PCT 全局路径不是近距离急停器，最终防撞仍由 local costmap 和 DWB 的合法轨迹检查负责。

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

### DWB 与 adapter 控制参数

`nav2_dwb_body_plane.yaml` 的底盘动力学和 critic 参数已同步 2D 稳定版提交 `6588506`：最大线速度 `0.8 m/s`、最大角速度 `0.65 rad/s`、非零线速度下限 `0.2 m/s`、非零角速度下限 `0.5 rad/s`，并使用 `ObstacleFootprint` 对完整矩形 footprint 做碰撞评价。`velocity_smoother` 使用同一组速度、加速度和减速度限制，避免 DWB 输出在后级被另一套约束改变。

这里只移植了与同一底盘相关的控制参数。3D 导航仍使用 `odom_body -> base_link`、`/traversability_filtered_scan`、滚动局部 costmap 和现有楼梯逻辑；2D 仓库的 `odom_nav/base_footprint`、静态地图层及原始 `/scan` 没有复制。`sim_time=2.0` 和 `min_speed_xy=0.2` 会让前向预测更远、底盘更快越过速度死区，但在窄楼梯平台上可能表现得比原配置更积极，实车测试时应重点观察转弯半径、贴栏杆距离和制动距离。

adapter 向 Nav2 发送 `FollowPath` 使用异步 action。若发送期间收到更新的 `/local_path`，已经被 controller 接受的旧 goal 不会立即取消；它会持续到下一次发送周期由新路径接管，避免中间插入零速度。只有 local path 被清空或超过 `adapter_path_timeout` 才会取消当前 goal。

## 发布终点

主导航默认已经启动 PCT planner，不需要再单独启动 `m20_pct_rviz.launch.py`。PCT 起点默认来自 TF `map -> base_link`，所以全局路径会从机器人真实 `base_link` 位置开始；默认每 `1.0s` 从最新机器人位置重新规划并更新 `/pct_path`。

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
