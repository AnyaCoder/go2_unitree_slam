# Go2 点选导航：配置来源与最终链路

一句话：电脑用固定 PCD 运行 `pcdMapSim -> terrain -> FAR`，通过 TCP 8899
只向狗端发送有序全局路径；狗端不运行 FAR，只运行 terrain、
localPlanner 和 pathFollower。本机额外设置 MID-360 安装偏移、真机
`0.5 m/s` 限速和禁止 `(0,0)` 初始目标。权威说明见 [`必看.md`](必看.md)。

## 1. 参数唯一来源

| 节点 | 取值来源（上游文件） |
|---|---|
| `pcdMapSim` | `autonomy_stack_go2/src/utilities/map_sim/launch/map_sim.launch.py` |
| `terrainAnalysis`（电脑） | `terrain_analysis/launch/terrain_analysis.launch` |
| `terrainAnalysisExt`（电脑） | `terrain_analysis_ext/launch/terrain_analysis_ext.launch` |
| `far_planner`（电脑） | `far_planner/config/default.yaml` |
| `terrainAnalysis`（狗端） | `terrain_analysis/launch/terrain_analysis.launch` |
| `terrainAnalysisExt`（狗端） | `terrain_analysis_ext/launch/terrain_analysis_ext.launch` |
| `localPlanner`（狗端） | `local_planner/launch/local_planner.launch` |
| `pathFollower`（狗端） | `local_planner/launch/local_planner.launch` |

电脑端 `nav/global_planner.launch.py` 动态加载上游已安装的
`map_sim.launch.py`，只保留 `pcdMapSim`、`terrainAnalysis`、
`terrainAnalysisExt`，再按上游 `default.yaml` 启动 FAR。电脑 FAR 的
`/way_point` 固定重映射到 `/far/way_point`，确保公共 `/way_point` 只存在于
狗端。这样上游参数变化会自动跟随，也不会在电脑上误启动
`localPlanner/pathFollower`。

FAR 的障碍多边形安全边界统一设为
`c_detector/contour_padding_m=0.12 m`，由电脑全局规划启动文件和狗端 FAR
调试启动文件显式覆盖。狗端 `go2_nav.launch.py` 仍直接解析上游 launch XML，
原样传递其余参数。狗端覆盖项：

- `sensorOffsetX/Y`：MID-360 安装位置
- `maxSpeed=0.5 m/s`：真机限速
- `goalX/goalY=NaN`：没有有效 `/way_point` 前禁止追 `(0,0)`

## 2. 最终数据链路

电脑：

```text
固定 PCD
  -> pcdMapSim(external_odom_topic=/state_estimation)
  -> terrainAnalysis / terrainAnalysisExt
  -> FAR
  -> /viz_path_topic
  -> global_path_sender
  -> TCP 8899
```

狗端：

```text
TCP 8899
  -> global_path_receiver
  -> /way_point
  -> localPlanner
  -> /path
  -> pathFollower
  -> /api/sport/request
```

狗端不运行 FAR。电脑端不运行 `localPlanner/pathFollower`。电脑端也不发布
公共 `/way_point`；电脑 FAR 的本机 waypoint 仅为 `/far/way_point`。

状态上行：

```text
狗端 re_location_odometry / tf / tf_static / joint_states / path
  -> TCP 8898
  -> 电脑 far_viz_receiver
```

## 3. `sensorOffsetX` 怎么定

本机雷达为 **Livox MID-360**，安装在狗头/前上方，向下倾斜约 13°。
`sensorOffsetX/Y` 只描述 MID-360 相对 localPlanner 车体中心的平面平移，不承载
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
```

来源是公开 Go2 + MID-360 URDF：

```text
base_link -> imu:         x = -0.02557
imu -> livox_frame:       x = +0.1870, pitch = 0.22689 rad (~13°)
base_link -> livox_frame: x = 0.16143 m
```

这不是全球统一的厘米级真值。本车可实测覆盖：

```bash
SENSOR_OFFSET_X=0.187 ./run_nav.sh dry
```

所有入口共用：

```text
nav_dog/go2_mid360_hardware.env
```

`start_far_real.sh`、狗端 `run_nav.sh` 和 `global_path_receiver` 必须使用同一个
偏移。否则保持目标相对车体会产生固定“胡萝卜牵引”。

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

不要只运行 `run_nav.sh` 就认为整套导航已启动；不要再启动
`demo_mid360`、第二套 Livox/LIO、狗端 FAR 或第二个 RViz。

狗端参数检查：

```bash
cd /home/unitree/go2_nav
python3 check_upstream_params.py
ros2 param get /localPlanner sensorOffsetX
ros2 param get /localPlanner autonomyMode
ros2 param get /localPlanner goalX
ros2 param get /localPlanner goalY
timeout 3 ros2 topic echo /path
```

`goalX/goalY` 启动时应为 NaN。狗端 `global_path_receiver.py` 是公共
`/way_point` 唯一发布者：

```text
无路径 -> 当前重定位车体位姿
有效路径 -> 全局路径看前点
路径超时/空 -> 回到当前重定位车体位姿
```

电脑检查：

```bash
./nav/check_real_chain.sh
tail -f /tmp/go2_laptop_global_planner.log
tail -f /tmp/go2_global_path_sender.log
```

电脑必须有 `pcdMapSim/terrain/FAR/global_path_sender`，不能有
`localPlanner/pathFollower`。狗端必须有 `terrain/localPlanner/pathFollower/
global_path_receiver`，不能有 `far_planner/far_command_link`。

## 5. 部署到狗

```bash
scp go2_unitree_slam/nav_dog/{go2_nav.launch.py,check_upstream_params.py,run_nav.sh,nav_path_protocol.py,global_path_receiver.py,run_global_path_receiver.sh,go2_mid360_hardware.env} go2-wifi:/home/unitree/go2_nav/
ssh go2-wifi 'chmod +x /home/unitree/go2_nav/run_nav.sh /home/unitree/go2_nav/run_global_path_receiver.sh'
```

前提：狗上 `/home/unitree/autonomy_stack_go2/install` 已编译且可 source。

## 6. 用 2D 界面修空地，并同步到 3D 点云

UI 用现成的 [ROS-SLAM-Map-Editor](https://gyropalm.github.io/ROS-SLAM-Map-Editor/editor.html)
（浏览器打开，支持画笔/擦除/矩形/撤销/未知区）。它只认 ROS 的
`PGM + YAML`，所以本仓库只保留一个薄转换器 `tools/pcd_map_edit.py`。

### 6.1 导出 2D 地图

```bash
cd /home/anyacoder/go2_unitree_slam
python3 tools/pcd_map_edit.py export GlobalMap_clean_20261020.pcd -o map_edit_20261020
```

得到：

```
map_edit_20261020/map.pgm
map_edit_20261020/map.yaml
map_edit_20261020/map_preview.png
```

浏览器打开上面的编辑器，把 `map.pgm` 和 `map.yaml` 一起拖进去。

也可以完全本地运行（依赖已经放进仓库，不访问 CDN）：

```bash
cd /home/anyacoder/go2_unitree_slam
tools/serve_map_editor.sh 8090
# 浏览器打开 http://127.0.0.1:8090/editor.html
```

端口可改，例如 `tools/serve_map_editor.sh 8091`；要让同一局域网的其他电脑访问，
用 `tools/serve_map_editor.sh 8090 0.0.0.0`。

### 6.2 编辑规则

- **FREE / 刷白**：该 2D 格子从地面上方一直清到天花板。应用后会删除这个
  垂直柱里所有障碍点、人、墙、天花板点，只保留地面点。
- **BLOCKED / 刷黑**：保留原点云。需要人为补障碍柱时，apply 加
  `--add-blocked`，会在该格插入稀疏竖直标记点。
- **UNKNOWN / 刷灰**：不改 3D 点云，保留原始点。
- 建议先只修“明显是空地但被误判为障碍”的区域，不要大面积刷 FREE。

### 6.3 应用回 3D PCD

在编辑器里下载 `map_edited.yaml` 和 `map_edited.pgm`，放到同一目录，然后：

```bash
python3 tools/pcd_map_edit.py apply GlobalMap_clean_20261020.pcd \
    --yaml map_edit_20261020/map_edited.yaml \
    --pgm  map_edit_20261020/map_edited.pgm \
    --out  GlobalMap_clean_20261020_edited.pcd
```

输出旁边会生成 `GlobalMap_clean_20261020_edited_apply.json`，记录删除了多少点。
显示和点选使用 `map_sim.launch.py` 的 `pcd:=` 参数加载编辑后的 PCD。

> 编辑后的 PCD 只用于 RViz 显示/点选目标。**重定位必须继续使用原始
> `GlobalMap.pcd`**，否则闭环和定位会受人为改动影响。

### 6.4 去掉天花板层

编辑后的点云如果还带天花板，按 z 高度裁掉即可。当前办公室天花板大约在
`z=1.8..2.1 m`，保留 `z <= 1.80 m`：

```bash
python3 tools/pcd_map_edit.py crop-z GlobalMap_clean_20261020_edited.pcd \
    --zmax 1.80 \
    -o GlobalMap_clean_20261020_edited_noroof.pcd
```

RViz/map_sim 通过 `pcd:=` 参数加载裁剪后的无天花板 PCD。
天花板层只从显示/点选地图中删掉，重定位仍用原始完整地图。

### 6.5 3D 里把可通行地面显示成绿色

在无天花板点云上生成带 RGB 的显示版，并额外生成一个只含绿色地面点的图层：

```bash
python3 tools/pcd_map_edit.py color-ground GlobalMap_clean_20261020_edited_noroof.pcd \
    --yaml map_edit_20261020/map_edited.yaml \
    --pgm  map_edit_20261020/map_edited.pgm \
    --out  GlobalMap_clean_20261020_nav_green.pcd \
    --walkable-out GlobalMap_clean_20261020_walkable_ground.pcd
```

- `GlobalMap_clean_20261020_nav_green.pcd`：PCL Viewer 用，绿色地面 + 灰色结构
- `GlobalMap_clean_20261020_walkable_ground.pcd`：RViz 用，只含绿色地面点

FAR 导航运行架构不使用 `spaceViz`、`/sim/free_space` 或
`/sim/free_boundary`。固定导航 PCD 只是电脑 RViz/PCL 背景；绿色地面若存在，
也只是显示层，不参与狗端 terrain、FAR、localPlanner 或碰撞计算。
