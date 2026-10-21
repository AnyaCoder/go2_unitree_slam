# 真机 RViz 显示与数据来源

> 权威架构：[`必看.md`](../必看.md)。本文只记录最终真机显示验收。
> 更新：2026-10-21

## 1. 目标

真机 RViz 使用：

```text
nav/far_real.rviz
```

约束：

```text
nav/far_real.rviz 与 nav/map_sim.rviz 逐字节一致
Fixed Frame: map
```

工具：

```text
MoveCamera
2D Pose Estimate -> /initialpose
Goalpoint -> /goal_point（电脑本地 FAR 输入）
```

## 2. 最终架构

```text
电脑：
  固定 PCD -> pcdMapSim -> terrain -> FAR -> global_path_sender

狗端：
  Livox/LIO -> terrain -> localPlanner -> pathFollower

显示：
  电脑本地 FAR Marker + 电脑固定 PCD
  狗端真实 odom/TF/joints/path/小点云
```

电脑端必须运行：

```text
pcdMapSim
terrainAnalysis
terrainAnalysisExt
far_planner
```

电脑端禁止运行：

```text
localPlanner
pathFollower
```

狗端禁止运行：

```text
far_planner
```

## 3. 固定地图

```text
/home/anyacoder/go2_unitree_slam/GlobalMap_clean_20261020_1920_nav_green_edited.pcd
```

同一张 PCD 有两项用途：

1. `pcdMapSim` 在电脑上生成固定地图虚拟扫描，供电脑 FAR 规划；
2. `far_viz_receiver.py` 发布 `/overall_map`，作为 RViz 背景。

狗端实时避障仍使用 Livox/terrain，不把完整固定图上传给狗。

## 4. 显示数据

| Display | 来源 | 说明 |
|---|---|---|
| Go2 EDU model | `/robot_description`、狗端 `/joint_states`、真实 TF | 电脑 `robot_state_publisher` |
| OverallMap | `/overall_map` | 电脑固定 PCD 背景 |
| RegScan | `/registered_scan` | 电脑 `pcdMapSim` 虚拟扫描 |
| TerrainMap | `/terrain_map` | 电脑 terrainAnalysis |
| TerrainMapExt | `/terrain_map_ext` | 电脑 terrainAnalysisExt |
| Go2 Live Cloud | `/go2/min_obstacle_cloud` | 狗端小点云，仅显示 |
| Local path | `/path` | 狗端 localPlanner |
| FAR global path | `/viz_path_topic` | 电脑本地 FAR |
| FAR V-Graph / obstacle edges | `/viz_graph_topic` | 电脑本地 FAR |
| Goalpoint / graph nodes | `/viz_node_topic` | 电脑本地 FAR |
| FAR Poly | `/viz_poly_topic` | 电脑本地 FAR |
| Boundary | `/navigation_boundary` | 电脑本地 FAR |
| Odom arrow | `/state_estimation` | 狗端真实重定位 odom |

红线是电脑本地 FAR 原生 `polygon_edge`；不得自己重画。真正的障碍轮廓来自
`/viz_poly_topic`，默认显示开关按现有 RViz 配置保持不变。

## 5. TCP 上行和下行

TCP 8898 只上行：

```text
/state_estimation
/tf
/tf_static
/joint_states
/path
可选：狗端小点云
```

FAR Marker 不再上行；它们在电脑本地生成。

TCP 8899 只下行：

```text
电脑 FAR /viz_path_topic
  -> ordered global path
  -> 狗端 global_path_receiver
```

`/way_point` 与 `/goal_point` 不跨 8898 上行。

小点云限制：

```text
约 6 m
0.15 m 体素
最多 12000 点
最高 1 Hz
仅 RViz
```

## 6. TF 与模型

电脑显示 TF：

```text
world -> map
map   -> sensor   （狗端重定位 odom 经 TCP 8898）
sensor -> base
base   -> vehicle
```

Go2 URDF 来自：

```text
nav/go2_model_params.yaml
```

关节动画来自狗端真实 `/joint_states`。

## 7. 启动

真机：

```bash
cd ~/go2_unitree_slam
./nav/start_far_real.sh
```

操作：

1. 使用一次 `2D Pose Estimate`。
2. 确认 Go2 模型在固定地图上。
3. 确认 `first SLAM odometry received`。
4. 使用 `Goalpoint`。
5. 观察电脑本地 FAR 路径和狗端 `/path`。

独立模拟对照：

```bash
cd ~/go2_unitree_slam
./nav/run_map_sim.sh
```

模拟只用于比较布局，不能作为真机数据源。

## 8. 只读验收

```bash
cd ~/go2_unitree_slam
./nav/check_real_chain.sh
```

必须确认：

```text
电脑存在 /pcdMapSim /terrainAnalysis /terrainAnalysisExt /far_planner
电脑存在 /global_path_sender /far_viz_receiver /go2_initialpose_sender
电脑没有 /localPlanner /pathFollower
电脑 FAR Marker 各只有一个发布者
电脑 /way_point 没有发布者
电脑 /far/way_point 发布者是 /far_planner
狗端存在 terrain/localPlanner/pathFollower/global_path_receiver
狗端没有 far_planner/far_command_link
狗端 /way_point 唯一发布者
狗端没有旧运动控制节点
```

日志：

```text
/tmp/go2_real_state_receiver.log
/tmp/go2_laptop_global_planner.log
/tmp/go2_global_path_sender.log
/tmp/global_path_receiver.log      # 狗端
```

## 9. 一句话验收

**固定 PCD、Go2 模型、真实 odom、电脑本地 FAR 输出和狗端局部路径可见；电脑没有局部执行器，狗端没有 FAR，TCP 只传小体积状态与路径。**
