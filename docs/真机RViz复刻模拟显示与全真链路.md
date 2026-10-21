# 真机 RViz 目标显示效果

> 权威架构：根目录 [`必看.md`](../必看.md)。本文只记录显示验收，不另立架构。
> 更新：2026-10-21

## 1. 唯一目标

目标画面：

![目标显示效果](images/target_effect_simulation_20261021.png)

真机完成重定位后，RViz 的信息结构、图层、颜色、面板布局必须与这张模拟目标图一致。模拟只用于确定显示脚本和视觉验收；真机数据必须来自规定链路，不能用模拟 odom、TF、扫描或 Marker 补画面。

错误画面：

![错误：真机只有 OverallMap](images/failure_real_only_overall_map_20261021.png)

错误画面只有白色 `OverallMap`。真机如果同时缺少 Go2 模型、彩色 RegScan、TerrainMap、TerrainMapExt、局部路径、FAR V-Graph 和 Odom 箭头，就不算达到目标。

## 2. 唯一地图

```text
/home/anyacoder/go2_unitree_slam/GlobalMap_clean_20261020_1920_nav_green_edited.pcd
```

这是唯一地图输入。

禁止：

- 人工可走区 PCD
- `spaceViz`
- `/sim/free_space`
- `/sim/free_boundary`
- 手画可走区、墙面、FAR V-Graph 或障碍边界

## 3. RViz 文件

```text
nav/far_real.rviz
```

约束：

```text
nav/far_real.rviz 与 nav/map_sim.rviz 必须逐字节一致
```

当前正确 SHA-256：

```text
429ded0a4e841c1dde376d1dc970990adf16a957d4976acf0ac4e93b344cac17  nav/map_sim.rviz
429ded0a4e841c1dde376d1dc970990adf16a957d4976acf0ac4e93b344cac17  nav/far_real.rviz
```

## 4. 全局设置

```yaml
Fixed Frame: map
Background Color: 0; 0; 0
Frame Rate: 15
Transformation: TF
```

必须保留的面板：

```text
Displays
Selection
Tool Properties
Views
TeleopPanel
Image（No Image）
SemanticImage（No Image）
```

工具：

```text
MoveCamera
2D Pose Estimate -> /initialpose
Goalpoint -> 插件硬编码 /goal_point
```

## 5. 必须开启的 Display

```text
Go2 EDU model
RegScan
TerrainMap
TerrainMapExt
Local Collision Obstacles
Local path
FreePaths
Goalpoint
OverallMap
FAR global path
FAR V-Graph / obstacle edges
ViewpointExtend
Trajectory
Go2 Collision Footprint
Boundary
Odom arrow
```

数据来源：

| Display | Topic/来源 | 说明 |
|---|---|---|
| Go2 EDU model | `/robot_description`、真实 `/joint_states`、真实 TF | 必须是 Go2 实体模型 |
| RegScan | `/registered_scan` | `pcdMapSim` 跟随真机 odom 从固定静态 PCD 截取 |
| TerrainMap | `/terrain_map` | 电脑本地 terrainAnalysis |
| TerrainMapExt | `/terrain_map_ext` | 电脑本地 terrainAnalysisExt |
| Local Collision Obstacles | `/local_planner/collision_obstacles` | 本地 localPlanner 显示层；上游当前可能为空 |
| Local path | `/path` | 本地 localPlanner |
| FreePaths | `/free_paths` | 本地 localPlanner |
| Goalpoint | `/viz_node_topic` | 电脑端 FAR 原生输出 |
| OverallMap | `/overall_map` | 固定静态 PCD，388,413 点 |
| FAR global path | `/viz_path_topic` | 电脑端 FAR 原生输出 |
| FAR V-Graph | `/viz_graph_topic` | 电脑端 FAR 原生输出 |
| ViewpointExtend | `/viz_viewpoint_extend_topic` | 电脑端 FAR |
| Trajectory | `/trajectory` | 真实 odom 轨迹 |
| Go2 Collision Footprint | `/local_planner/collision_footprint` | 本地 localPlanner；上游当前可能为空 |
| Boundary | `/navigation_boundary` | 电脑端 FAR |
| Odom arrow | `/state_estimation` | 狗端真实 `re_location_odometry` 经 TCP 8898 |

## 6. 必须关闭的 Display

```text
Go2Model marker
FAR Contour
FAR GridMap
FAR Poly
Image
Local Terrain Map
Local Terrain Path
SemanticImage
SensorScan
TravArea
```

## 7. FAR V-Graph Namespace

开启：

```text
freespace_vgraph
polygon_edge
visibility_edge
```

关闭其余 namespace。

红色 `polygon_edge` 必须来自 FAR 原生 `/viz_graph_topic`，不是人工绘制的墙线。

## 8. 真机达到目标显示的必要条件

```text
/lio_sam_ros2/mapping/re_location_odometry 持续发布
/state_estimation 持续发布
map -> sensor TF 可持续查询
/joint_states 持续发布
/pcdMapSim 收到真机 /state_estimation
/registered_scan 持续发布
/terrain_map 持续发布
/terrain_map_ext 持续发布
FAR /viz_graph_topic、/viz_path_topic、/navigation_boundary 持续发布
```

启动后先做一次 RViz `2D Pose Estimate`。狗端重定位成功后，接收日志必须出现：

```text
first SLAM odometry received
```

并且计数持续增长：

```text
odom > 0
tf > 0
joints > 0
```

在这些条件满足前，只出现 OverallMap 是预期失败状态，不能算真机显示完成。

## 9. 启动命令

本地模拟：

```bash
cd ~/go2_unitree_slam
./nav/run_map_sim.sh
```

真机：

```bash
cd ~/go2_unitree_slam
./nav/run_real_view.sh
```

两者使用同一个固定静态地图和同一个 RViz 目标布局。区别只在数据源：模拟使用内部 odom；真机使用狗端真实重定位 odom、TF 和关节状态。

## 10. 只读验收

```bash
cd ~/go2_unitree_slam
./nav/check_real_chain.sh
```

必须确认：

```text
RViz 使用 nav/far_real.rviz
pcdMapSim.external_odom_topic=/state_estimation
pcdMapSim.pcd_path 指向唯一固定地图
spaceViz 未启动
/sim/free_space 不存在
/sim/free_boundary 不存在
/overall_map、/registered_scan、/terrain_map、/terrain_map_ext 均存在
FAR 与机器人模型节点存在
```

完成重定位后，还必须检查接收器日志中 odom、TF、joints 消息计数持续增长。

## 11. 一句话验收

**固定静态地图直接显示；没有人工可走区；模拟图和真机图使用同一 RViz 布局，真机在重定位后必须补齐 Go2 模型、滚动扫描、terrain、localPlanner、FAR 和 Odom 全部原生显示。**
