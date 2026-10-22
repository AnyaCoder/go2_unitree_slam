# 真机 RViz 显示与狗端完整规划链路

> 权威文档：[`必看.md`](../必看.md)。更新：2026-10-22

## 1. 目标

真机使用 `nav/far_real.rviz`，模拟对照使用 `nav/map_sim.rviz`。两者保持同一
显示布局，Fixed Frame 为 `map`。

RViz 工具：

```text
MoveCamera
2D Pose Estimate -> /initialpose
Goalpoint -> /goal_point
```

## 2. 数据链路

```text
狗端：
  Livox/LIO -> /nav/state_estimation
  固定导航 PCD -> static_map_replay -> terrainAnalysis -> terrainAnalysisExt
  -> FAR -> /way_point -> localPlanner -> /path -> pathFollower

电脑：
  同一固定 PCD -> /overall_map（背景）
  8898 接收狗端 odom/TF/joints/path/waypoint/FAR Marker
  8899 下行最终 /goal_point
  8900 下行 /initialpose
```

狗端使用固定导航 PCD 生成 0.08 m / 6 m × 6 m / 5 Hz 点云。
电脑不运行 FAR、localPlanner 或 pathFollower，也不上传实时点云。

## 3. RViz 显示源

| Display | Topic | 来源 |
|---|---|---|
| Go2 EDU model | `/robot_description`、`/joint_states`、TF | 电脑模型 + 狗端真实关节 |
| OverallMap | `/overall_map` | 与狗端规划相同的固定 PCD |
| Static path (pathFollower) | `/path` | 狗端 localPlanner |
| FAR global path | `/viz_path_topic` | 狗端 FAR |
| FAR V-Graph / obstacle edges | `/viz_graph_topic` | 狗端 FAR |
| Goalpoint / graph nodes | `/viz_node_topic` | 狗端 FAR |
| FAR Poly（按需开启） | `/viz_poly_topic` | 狗端 FAR |
| Boundary | `/navigation_boundary` | 狗端 FAR |
| Odom arrow | `/state_estimation` | 狗端重定位 |

不显示实时扫描、terrain map 或局部碰撞云。

## 4. TCP 契约

8898 狗端 -> 电脑：

```text
/state_estimation /tf /tf_static /joint_states
/path /way_point
/viz_path_topic /viz_graph_topic /viz_node_topic /viz_poly_topic
/navigation_boundary
```

8899 电脑 -> 狗端：

```text
仅最终 /goal_point，无路径、无速度
```

8900 电脑 -> 狗端：

```text
/initialpose
```

## 5. 启动与验收

```bash
cd ~/go2_unitree_slam
./nav/start_far_real.sh
./nav/check_real_chain.sh
```

必须看到电脑端无规划器、狗端完整链路存在、`/api/sport/request` 只有
`pathFollower` 发布，并且 FAR Marker 均由狗端唯一发布。

## 6. 一句话验收

**电脑只显示固定 PCD、真实 Go2 状态和狗端 FAR 的原生输出；狗端用同一固定 PCD 规划，规划与执行只发生在狗端。**
