# tomogram_filter_layer

`tomogram_filter_layer` 是独立的 Nav2 local costmap 插件。它启动时直接加载离线
`*.surface.pcd`，再订阅当前三维点云，删除与静态表面 XYZ 匹配的地面/楼梯点，并发布
`/traversability_filtered_scan`。后续 `ObstacleLayer` 使用这份 scan 标记和清除 local
costmap，因此插件本身不重复写 master costmap，也不要求 PCT planner 在线运行。

## 启用

M20 的完整参数位于
`src/m20_fastlio_nav/config/nav2_dwb_body_plane.yaml`。在
`local_costmap.local_costmap.ros__parameters` 中选择：

```yaml
plugins: ["tomogram_filter_layer", "obstacle_layer", "inflation_layer"]

tomogram_filter_layer:
  plugin: "tomogram_filter_layer::TomogramFilterLayer"
  enabled: true
  input_cloud_topic: /cloud_registered_body_1
  tomogram_surface_file: /mnt/nvme/workspace/fast_lio_ws/src/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.surface.pcd
  output_scan_topic: /traversability_filtered_scan
  map_frame: map
  base_frame: base_link
```

不要与会发布同一 filtered scan 的 `traversability_layer` 同时加载。要切回实时
traversability cost 过滤，只需把 `plugins` 改回：

```yaml
plugins: ["traversability_layer", "inflation_layer"]
```

tomogram 模式中的 `ObstacleLayer` 不能删除或关闭：本插件负责生成过滤后的 LaserScan，
真正向 local costmap 写入和清除障碍的是后续 `ObstacleLayer`。没有列入 `plugins` 的
过滤插件不会被 Nav2 创建，因此不需要再通过 launch 参数启动、停止或同步两个过滤进程。

本包与 `traversability_layer`、PCT planner 都没有节点级运行依赖。插件只读取
`nav2_dwb_body_plane.yaml` 中配置的 `tomogram_surface_file`，主 launch 不再覆盖文件或
cost 阈值，也不再提供对应的 launch 参数。切换地图时需要手动同步：

```text
PCT pct_tomogram_file          -> m20_3d_map
Nav2 tomogram_surface_file     -> .../m20_3d_map.surface.pcd
PCT pct_a_star_cost_threshold  = Nav2 traversable_cost_max = 45.0
```

## 处理流程

1. 离线 tomography 同时生成完整规划地图 `.pickle` 和过滤表面 `.surface.pcd`。
2. 插件启动时读取 PCD，在 `map` frame 中建立带多层 Z 样本的 XY 索引。
3. 当前点云先变换到 `base_link`，按高度、角度和距离选取 LaserScan 候选点。
4. 候选点再变换到 `map`，与 tomogram 表面进行 XY/Z 匹配；匹配的地面点删除。
5. 未匹配点按每个角度 bin 的最近距离发布，栏杆、人和未知区域障碍继续保留。

Nav2 的 clear-costmap/reset 只清除后续 `ObstacleLayer` 的局部代价，不会清掉插件已加载的
静态 tomogram；因此清图后过滤会继续工作。

新生成 tomogram 时会自动得到两个文件：

```text
PCT_planner/rsc/tomogram/m20_3d_map.pickle
PCT_planner/rsc/tomogram/m20_3d_map.surface.pcd
```

已有 pickle 不需要重新运行 tomography，可直接转换：

```bash
ros2 run pct_planner_ros2 pct_export_tomogram_surface m20_3d_map
```

## 参数

| 参数 | M20 配置值 | 说明 |
|---|---:|---|
| `tomogram_surface_file` | YAML 指定 | 静态 XYZI 表面 PCD；文件缺失会阻止插件启动 |
| `publish_surface_debug` | `true` | 是否将已加载且通过 cost 阈值的表面发布给 RViz |
| `surface_debug_topic` | `/traversability_tomogram` | transient-local 调试点云；不是插件输入 |
| `publish_unfiltered_when_not_ready` | `false` | tomogram 或 TF 未就绪时是否透传未过滤 scan；默认禁止 |
| `min_height` / `max_height` | `-0.10 / 0.80m` | 生成 scan 的 `base_link` 高度范围 |
| `range_min` / `range_max` | `0.15 / 10.0m` | 输出 scan 距离范围 |
| `tomogram_grid_resolution` | `0.0` | 默认从 PCD 头自动读取；旧 PCD 没有元数据时才手动填写 |
| `ground_xy_tolerance` | `0.20m` | 点与表面样本允许的平面距离 |
| `ground_z_min_offset` | `-0.20m` | 点允许低于 tomogram 表面的高度 |
| `ground_z_max_offset` | `0.22m` | 点允许高于表面的高度；过大会删除低矮障碍 |
| `traversable_cost_max` | `45.0` | 允许作为地面的最大 tomogram cost，应与 PCT A* 阈值一致 |
| `keep_points_without_ground` | `true` | tomogram 未覆盖位置是否保留点，建议保持开启 |
| `min_tomogram_points` | `1000` | 加载后允许投入使用的最低点数，防止残缺文件投入过滤 |

修改 YAML 只需重启导航。修改本包 C++ 后重新编译：

```bash
colcon build --symlink-install --packages-select tomogram_filter_layer
```

正常启动后应看到类似日志：

```text
TomogramFilterLayer loaded .../... surface points in ... cells from ...surface.pcd
```

插件加载失败时先检查 `tomogram_surface_file`。如果表面加载成功但只有
`suppressing scan` 警告，再检查 `/cloud_registered_body_1` 和点云时间戳对应的
`map -> base_link` TF。`/traversability_tomogram` 仅用于 RViz，可关闭而不影响过滤。
