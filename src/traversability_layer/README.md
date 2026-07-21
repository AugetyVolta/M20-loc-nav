# Traversability Layer - 3D可通行性代价地图层

## 概述

Traversability Layer 是一个 nav2_costmap_2d 插件，用于从3D点云数据生成可通行性代价地图。相比传统2.5D高程图方案，它解决了室内环境中天花板被误判为不可通行区域的问题。

### 解决的核心问题

1. **天花板误判**：传统2.5D高程图取每个cell的最高点作为地面高度，室内环境中天花板会被当作不可通行区域
2. **平行墙体盲区**：与视线平行的墙体在raycast找梯度时可能找到相邻等高墙体，导致无cost

## 核心算法流程

```
点云输入 (PointCloud2)
    ↓
[1] 3D Voxel Grid 构建 (OpenMP并行)
    ↓
[2] Ray Tracing: 从传感器原点投射射线
    ↓ 区分地面点(hit + 上方free)和天花板点(hit + 下方free)
[3] 地面提取 (Ground Extraction)
    ↓ 计算每个cell的地面高度z
[4] 障碍物比率检测 (Obstacle Ratio)
    ↓ 检测地面上方robot_height范围内的障碍密度
    ↓ 解决平行墙体盲区问题
[5] 立方插值补全 (Interpolation)
    ↓ 填充无观测数据的地面cell
[6] 坡度与高度差计算
    ↓ 计算slope_x, slope_y, slope_magnitude, height_diff
[7] 代价计算 → Costmap2D
    LETHAL: 坡度>阈值 或 障碍比率>阈值
    INFLATED: 坡度/高度差线性加权
    FREE: 可通行区域
```

## 关键数据结构

### VoxelData - 3D体素
```cpp
struct VoxelData
{
  uint8_t hit_count = 0;   // 点云命中计数
  uint8_t pass_count = 0;  // 射线穿过计数(free space)
};
```

### GroundCell - 地面cell
```cpp
struct GroundCell
{
  float ground_z = 0.0f;          // 地面高度
  bool has_ground = false;        // 是否有地面数据
  float height_diff = 0.0f;       // 邻域最大高度差
  float slope_x = 0.0f;          // X方向坡度
  float slope_y = 0.0f;          // Y方向坡度
  float slope_magnitude = 0.0f;  // 坡度幅值
  float obstacle_ratio = 0.0f;   // 障碍物比率
};
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pointcloud_topic` | `/lio/body/cloud` | 点云输入话题 |
| `sensor_frame` | `""` (自动) | 传感器坐标系 |
| `voxel_z_resolution` | 0.1m | 体素Z轴分辨率 |
| `voxel_z_min` | -1.0m | 体素Z轴最小值 |
| `voxel_z_max` | 3.0m | 体素Z轴最大值 |
| `ground_hit_threshold` | 1 | 地面判定最小hit数 |
| `free_space_threshold` | 1 | 自由空间判定阈值 |
| `free_space_window` | 3 | 自由空间搜索窗口(Z方向cell数) |
| `robot_height` | 0.5m | 机器人高度（用于障碍比率检测） |
| `obstacle_ratio_threshold` | 0.5 | 障碍比率阈值（超过则LETHAL） |
| `max_slope_traversable` | 45.0° | 最大可通行坡度 |
| `slope_cost_start` | 15.0° | 开始产生cost的坡度 |
| `step_height_threshold` | 0.15m | 台阶高度阈值 |
| `height_cost_start` | 0.0m | 开始产生cost的高度差 |
| `slope_cost_scale` | 5.0 | 坡度cost缩放系数 |
| `height_cost_scale` | 10.0 | 高度差cost缩放系数 |
| `lethal_cost_threshold` | 254.0 | 致命cost阈值 |
| `observation_persistence` | 5.0s | 观测数据持久时间 |
| `cloud_buffer_size` | 5 | 点云缓冲帧数 |
| `interp_search_radius` | 3 | 插值搜索半径(cell数) |
| `min_interp_neighbors` | 2 | 插值最小邻居数 |
| `num_threads` | 0 (自动) | OpenMP线程数 |

## nav2 集成配置

在 nav2 参数文件中添加 traversability_layer 插件：

```yaml
local_costmap:
  ros__parameters:
    plugins: ["static_layer", "traversability_layer", "inflation_layer"]
    traversability_layer:
      plugin: "traversability_layer::TraversabilityLayer"
      enabled: true
      pointcloud_topic: "/lio/body/cloud"
      voxel_z_resolution: 0.1
      voxel_z_min: -1.0
      voxel_z_max: 3.0
      ground_hit_threshold: 1
      robot_height: 0.5
      obstacle_ratio_threshold: 0.5
      max_slope_traversable: 45.0
      step_height_threshold: 0.15
      num_threads: 0
```

## 性能优化

- **OpenMP并行**: 体素构建和地面提取均使用多线程，`num_threads=0`自动检测核心数
- **Thread-local缓冲**: 减少原子操作竞争，voxel写入使用线程局部缓冲后合并
- **局部costmap缓冲**: 消除cost写入的临界区
- **数据压缩**: voxel数据从uint16_t压缩为uint8_t，使用memset快速初始化

## 独立 filtered scan 节点

`TraversabilityLayer` 只负责 Nav2 local costmap，当前主导航配置中默认关闭。过滤后的
LaserScan 由独立的 `tomogram_scan_filter_node` 生成，不再用二维 costmap cost 判断一个
beam 是否为地面；local costmap 的 `ObstacleLayer` 订阅这份过滤结果。

节点订阅：

- `/cloud_registered_body_1`：保留真实 Z 的当前帧三维点云。
- `/traversability_tomogram`：主导航专用的 PCT 可通行表面；与 tomography
  调试使用的 `/tomogram` 分开，避免多个 transient-local 发布者互相覆盖。

节点对每个三维点同时计算 `base_link` 坐标和 `map` 坐标。先用 `base_link.z` 选择参与
LaserScan 的高度范围，再用点的 `map.x/y/z` 查询最近的 tomogram 可通行表面。只有 XY
足够接近且 Z 高度落在地面容差内的点才删除。过滤只使用静态 tomogram，不再使用
`slope_map`，避免局部地面提取把稀疏栏杆误判为地面并删除。栏杆、人和高于地面的障碍
继续进入 `/traversability_filtered_scan`；tomogram 未覆盖的位置默认保留。

M20 参数位于：

```text
src/m20_fastlio_nav/config/tomogram_scan_filter.yaml
```

主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `enabled` | `true` | `false` 时不删地面，只从三维点云生成原始 scan，便于 A/B 测试 |
| `min_height` / `max_height` | `-0.10m` / `0.80m` | 生成 scan 的 `base_link` 高度范围；上限覆盖稀疏栏杆的更多回波 |
| `tomogram_grid_resolution` | `0.20m` | 必须与当前 `m20_3d_map.pickle` 的生成分辨率一致 |
| `ground_xy_tolerance` | `0.20m` | 点与可通行 tomogram cell 的最大平面距离；包含网格角点及少量定位误差 |
| `ground_z_min_offset` | `-0.20m` | M20 主导航允许点低于 tomogram 地面的高度误差 |
| `ground_z_max_offset` | `0.22m` | M20 主导航允许点高于 tomogram 地面的高度；调大可能漏掉低矮障碍 |
| `traversable_cost_max` | `45.0` | 可作为地面的最大 tomogram cost，主 launch 与 PCT 阈值同步 |
| `keep_points_without_ground` | `true` | tomogram 未覆盖位置保留点，保护未知障碍和稀疏栏杆 |
| `min_tomogram_points` | `1000` | tomogram 可通行点数完整性下限；不足时拒绝该消息并继续等待，避免空地图或测试地图锁死过滤器 |

### 旧版 cost 过滤对比

`TraversabilityLayer` 仍保留按实时 traversability cost 过滤原始 `/scan` 的旧方案，参数位于
`nav2_dwb_body_plane.yaml` 的 `local_costmap.traversability_layer`：

| 参数 | M20 配置值 | 说明 |
|---|---:|---|
| `publish_filtered_scan` | `false` | 是否启用旧版 cost 过滤；默认关闭 |
| `filtered_scan_input_topic` | `/scan` | 输入原始 LaserScan |
| `filtered_scan_topic` | `/traversability_filtered_scan` | 过滤后输出话题 |
| `filtered_scan_min_cost` | `160.0` | endpoint 所在格 cost 不低于该值才保留；未知或无地面格删除 |

旧版过滤与独立 `tomogram_scan_filter_node` 使用相同输出话题，不能同时开启。测试旧版时将
`traversability_layer.enabled` 和 `publish_filtered_scan` 都改为 `true`，并在主 launch
命令中传入 `start_tomogram_scan_filter:=false`。

修改 YAML 后只需要重启主导航，不需要重新编译。修改 C++ 源码后执行：

```bash
colcon build --symlink-install --packages-select traversability_layer m20_fastlio_nav
```

## 编译

```bash
cd ~/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select traversability_layer
```

## 备份

原始2.5D实现已备份为：
- `src/traversability_layer.cpp.bk`
- `include/traversability_layer/traversability_layer.hpp.bk`

## 项目位置

```
LIO-SAM_MID360_ROS2_PKG/ros2/src/traversability_layer/
├── include/traversability_layer/
│   ├── traversability_layer.hpp      # 头文件
│   └── traversability_layer.hpp.bk   # 原始2.5D版本备份
├── src/
│   ├── traversability_layer.cpp      # 3D实现
│   └── traversability_layer.cpp.bk   # 原始2.5D版本备份
├── traversability_layer_plugin.xml   # 插件描述
├── CMakeLists.txt
└── package.xml
```
