# Go2 点选导航：配置来源与改动清单

一句话：规划参数**不再手抄**，`go2_nav.launch.py` 在启动时直接解析上游
`autonomy_stack_go2` 的 launch XML 取值，所以永远等于上游默认值。
我们相对上游只保留 3 处差异，其中 1 处（雷达安装偏移）必须按实车测量。

## 1. 参数唯一来源

| 节点 | 取值来源（上游文件） |
|---|---|
| `terrainAnalysis` | `terrain_analysis/launch/terrain_analysis.launch` |
| `terrainAnalysisExt` | `terrain_analysis_ext/launch/terrain_analysis_ext.launch`（`checkTerrainConn=true`，与上游 `system_real_robot*.launch` 一致） |
| `localPlanner` | `local_planner/launch/local_planner.launch` |
| `pathFollower` | `local_planner/launch/local_planner.launch`（含 `/api/sport/request`、`/way_point` 两条 remap） |
| `far_planner` | `far_planner/config/default.yaml` + 上游 `far_planner.launch` 的 4 条 remap |

`go2_nav.launch.py` 里没有第二份数字：它 `ET.parse` 上面这些文件，展开
`$(var ...)`、`$(find-pkg-share ...)`，把 `<param>`、`<remap>` 原样交给节点。
上游改参数，我们重启即跟随，不会出现两边不一致。

2026-10-20 逐项对账结果（旧手抄版 vs 上游）：

```
localPlanner      38/38 项一致     terrainAnalysis     31/31 项一致
pathFollower      49/49 项一致     terrainAnalysisExt  17/17 项一致
唯一偏差：sensorOffsetX  旧写 0.0 / 上游 system_real_robot 用 0.3
```

## 2. 相对上游的 3 处改动（都在 `go2_nav.launch.py` 里）

1. **用 LIO-SAM 替换 Point-LIO**
   `/state_estimation` → `/lio_sam_ros2/mapping/odometry`，
   `/registered_scan` → `/lio_sam_ros2/mapping/cloud_registered`。
   `far_planner` 仍按上游写成 `/odom_world → /state_estimation`，由 ROS 2
   重映射链自动落到 LIO-SAM 上，far 的配置一个字没动。
2. **`sensorOffsetX/Y` 提成 launch 参数**（默认 0.3 / 0.0，即上游值）。
   这是唯一跟硬件安装位置绑定、必须实测的量，见第 3 节。
3. **保留上游局部规划链**，并把 `pathFollower` 的 `/way_point` 按上游
   `system_real_robot_with_route_planner.launch` 的写法指到 `/goal_point`。
   状态上行、waypoint 下行和初始位姿分别由三个独立 Python 桥处理，不再启动
   旧 C++/Python `nav_link`。同时按需把 `sport_out` 换成 `/nav/dry_cmd` 做干跑。

不启动、也不需要的东西（均为上游原本就没跑在本机上的）：

- `joy_node`：狗上没有 `/dev/input/js0`；FAR 导航不依赖手柄节点
- `point_lio_unilidar`：已被 LIO-SAM 取代
- `rviz2`、`go2_camera_bridge.py`：狗端无显示、相机桥由其它服务占用 8765
- `/sensor → /vehicle` 静态 TF：规划链只读 `/state_estimation`，不看 TF

## 3. `sensorOffsetX` 怎么定（唯一必须实测的项）

上游代码的约定（`localPlanner.cpp:156`、`pathFollower.cpp:175`）：

```
vehicle = sensor - R(yaw) * [sensorOffsetX, sensorOffsetY]
```

即 offset 是**车体中心在雷达坐标系里的位置**。所以：

- 雷达装在车体中心**前方** d 米 → `sensorOffsetX = +d`（L1 在狗头前 0.3 m，正是上游的 0.3）
- 雷达装在车体中心**后方** d 米 → `sensorOffsetX = -d`
- 雷达在中心**左侧** d 米 → `sensorOffsetY = +d`，右侧为 `-d`

量法（两人五秒）：让狗站平，量四条腿围成矩形的中心点，到 MID360 光学中心
（正面玻璃中心）沿机身前后/左右方向的投影距离。

> MID360 装在背上偏后，**大概率是负值**（例如 -0.10 ~ -0.20）。
> 不实测就沿用 0.3 的话，碰撞外框会整体前移 0.3+d 米：
> `localPlanner` 的打分会因为 footprint 判定全中而全部清零，
> 表现为 `/path` 只有 1 个 `(0,0,0)` 点、`pathFollower` 报
> `Waypoint aborted: no progress`。这是当前 1-pose 卡点的首要嫌疑。

先按实测值跑，命令行临时改也不用编辑文件：

```bash
SENSOR_OFFSET_X=-0.15 ./run_nav.sh dry
```

## 4. 启动与验证

真机导航的唯一启动入口见
`docs/真机SLAM冷启动与重定位成功流程_20261021.md`：

```text
狗端 A：./0_unitree_slam.sh
狗端 B：./run_nav.sh live
电脑端 C：./nav/run_real_view.sh
```

不要只运行 `run_nav.sh` 就认为整套导航已启动；也不要再启动
`demo_mid360`、第二套 Livox/LIO 或第二个 RViz。
下面仅保留狗端局部导航链的参数与验证方法。

```bash
cd /home/unitree/go2_nav
./run_nav.sh dry          # 干跑：运动指令发到 /nav/dry_cmd
./run_nav.sh live         # 实机：运动指令发到 /api/sport/request
```

启动后核对参数确实等于上游（不需要肉眼比对 XML）：

```bash
python3 /home/unitree/go2_nav/check_upstream_params.py
```

期望输出每个节点“一致”，最后一行 `结果: 全部与上游一致`；
`sensorOffsetX/Y` 会单独列为“硬件项”供确认，不计为错误。

其他常用检查：

```bash
ros2 param get /localPlanner sensorOffsetX
ros2 param get /localPlanner autonomyMode      # go2_nav.launch.py 固定为 true
ros2 topic echo --once /path                   # 正常应有几十个 pose，不是 1 个
```

## 5. 部署到狗

```bash
scp go2_unitree_slam/nav_dog/go2_nav.launch.py \
    go2_unitree_slam/nav_dog/check_upstream_params.py \
    go2_unitree_slam/nav_dog/run_nav.sh \
    go2-wifi:/home/unitree/go2_nav/
ssh go2-wifi 'chmod +x /home/unitree/go2_nav/run_nav.sh'
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

RViz 使用 `space_viz` 发布的 `/sim/free_space` 和 `/sim/free_boundary` 显示可走区域及其边界；路径与目标显示使用 FAR Planner 原生话题。
绿色地面是显示层，不参与定位和碰撞计算。
