# Go2 点选导航：狗端完整 FAR 规划与执行

一句话：狗端运行原版
`static_map_replay -> terrainAnalysis -> terrainAnalysisExt -> FAR -> localPlanner
-> pathFollower`；电脑只加载固定 PCD 背景、显示狗端真实状态/FAR 输出，并通过
TCP 8899 下发一个最终 `Goalpoint`。权威说明见 [`必看.md`](必看.md)。

## 1. 参数唯一来源

| 节点 | 取值来源 |
|---|---|
| `static_map_replay` | 本仓库 `nav_dog/static_map_replay.py`，复现 `map_sim` 输入 |
| `terrainAnalysis` | 与 `map_sim` 相同的显式参数 |
| `terrainAnalysisExt` | 与 `map_sim` 相同的显式参数 |
| `far_planner` | `far_planner/config/default.yaml` |
| `localPlanner` | `local_planner/launch/local_planner.launch` |
| `pathFollower` | `local_planner/launch/local_planner.launch` |

`nav_dog/go2_nav.launch.py` 的规划算法来自已安装的上游包；点云输入和 terrain
分类参数严格复现验收过的 `map_sim`。本机覆盖项：

```text
sensorOffsetX / sensorOffsetY：Livox MID-360 安装位置
maxSpeed = 0.5 m/s
pathFollower collisionCloudTimeout = 2.0 s
FAR c_detector/contour_padding_m = 0.12 m
localPlanner 启动 goalX/goalY = NaN
```

`goalX/goalY = NaN` 用于阻止重定位后在未收到 Goalpoint 时朝地图原点移动。

## 2. 唯一数据链路

狗端：

```text
Livox/LIO -> /nav/state_estimation
固定导航 PCD -> static_map_replay -> /nav/static_cloud_map
  -> terrainAnalysis -> /terrain_map
  -> terrainAnalysisExt -> /terrain_map_ext
  -> FAR -> /way_point
  -> localPlanner -> /path
  -> pathFollower -> /api/sport/request
```

电脑：

```text
固定 PCD -> /overall_map（仅 RViz 背景）
TCP 8898 -> 状态、TF、关节、/path、/way_point、FAR Marker
TCP 8899 -> 最终 /goal_point
TCP 8900 -> /initialpose
```

电脑不运行 `pcdMapSim`、terrain、FAR、`localPlanner` 或 `pathFollower`。
狗端实时点云不上传。

## 3. MID-360 安装偏移

本机雷达为 **Livox MID-360**，安装在狗头/前上方，向下倾斜约 13°。
`sensorOffsetX/Y` 只描述 MID-360 相对 pathFollower 车体中心的平面平移，不承载
pitch。上游约定：

```text
vehicle = sensor - R(yaw) * [sensorOffsetX, sensorOffsetY]
```

符号：

- MID360 在车体中心前方 d 米：`sensorOffsetX = +d`
- MID360 在车体中心后方 d 米：`sensorOffsetX = -d`
- MID360 在车体中心左侧 d 米：`sensorOffsetY = +d`

默认值：

```text
SENSOR_OFFSET_X=0.16143
SENSOR_OFFSET_Y=0.0
GO2_MAX_SPEED=0.5
```

来源是公开 Go2 + MID-360 URDF：

```text
base_link -> imu:         x = -0.02557
imu -> livox_frame:       x = +0.1870, pitch = 0.22689 rad (~13°)
base_link -> livox_frame: x = 0.16143 m
```

本车可实测覆盖：

```bash
cd /home/unitree/go2_nav && SENSOR_OFFSET_X=0.187 ./run_nav.sh dry
```

所有入口共用：

```text
nav_dog/go2_mid360_hardware.env
```

## 4. 启动与验证

唯一真机入口：

```bash
cd ~/go2_unitree_slam
./nav/start_far_real.sh
```

排障三终端：

```text
狗端 A：./0_unitree_slam.sh
狗端 B：./run_nav.sh live
电脑端 C：./nav/run_real_view.sh
```

狗端参数检查：

```bash
cd /home/unitree/go2_nav
python3 check_upstream_params.py
ros2 node list | grep -E 'far_planner|localPlanner|pathFollower|terrain|static_map_replay'
ros2 topic hz /path
ros2 topic hz /way_point
```

电脑验收：

```bash
./nav/check_real_chain.sh
tail -f /tmp/go2_real_state_receiver.log
tail -f /tmp/go2_goal_sender.log
```

必须满足：

```text
电脑没有 pcdMapSim / terrain / FAR / localPlanner / pathFollower
狗端存在 static_map_replay / terrain / FAR / localPlanner / pathFollower
狗端 /path、/way_point、FAR Marker 均为唯一发布者
/api/sport/request 只有 pathFollower 一个发布者
没有 dog_control_B1_one / go2_control_by_sdk send_cmd
```

操作顺序：`2D Pose Estimate` -> 等待重定位成功 -> `Goalpoint`。

## 5. 部署到狗

`nav/start_far_real.sh` 会自动同步：

```text
nav_dog/go2_nav.launch.py
nav_dog/run_nav.sh
nav_dog/static_map_replay.py
nav_dog/go2_mid360_hardware.env
GlobalMap_clean_20261020_1920_nav_green_edited.pcd
```

固定 PCD 会放在 `/home/unitree/go2_nav/maps/`，`static_map_replay.py`
会同步到 `/home/unitree/go2_nav/`。启动脚本会验证两者都存在后才启动规划链。
PCD 仅在狗端缺失或大小变化时传输；正常启动不会重复传大文件。修改 PCD 后：

```bash
GO2_FORCE_STATIC_MAP_SYNC=1 ./nav/start_far_real.sh
```

通信桥还会同步到 `/home/unitree/go2_nav`：

```text
nav_dog/far_viz_protocol.py
nav_dog/far_viz_link.py
nav_dog/far_command_protocol.py
nav_dog/far_command_link.py
nav_dog/initialpose_protocol.py
nav_dog/initialpose_link.py
nav_dog/lowstate/live_robot_state.py
```

前提：狗上 `/home/unitree/autonomy_stack_go2/install` 已编译且可 source。

## 6. 固定 PCD 的作用

```text
/home/anyacoder/go2_unitree_slam/GlobalMap_clean_20261020_1920_nav_green_edited.pcd
```

电脑端用它作为 RViz 背景；狗端使用同一文件的本地副本，经 `static_map_replay`
生成 0.08 m 体素、6 m × 6 m 滚动窗口、5 Hz terrain 输入。Livox 实时扫描
只用于 LIO 定位，不进入 FAR 规划。2D 空地编辑、天花板裁剪和绿色地面生成工具
会同时影响下一次真机规划输入，修改后必须重新同步 PCD。
