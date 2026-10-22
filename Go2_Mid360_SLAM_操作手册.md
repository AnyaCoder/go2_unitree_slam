# Go2 + Livox Mid-360 SLAM 操作手册

> **真机导航唯一启动链已变更（2026-10-21）**：导航必须按
> `必看.md` 的“唯一启动链”执行，即狗端 `0_unitree_slam.sh` +
> 狗端 `run_nav.sh live` + 电脑端 `nav/view_real.sh`。
> 本文后续 `demo_mid360` / `run_demo.sh` 内容仅保留作建图操作参考，
> 导航时不得与本链路同时启动。

> 适用：Go2 机器狗 + Livox Mid-360 雷达 + Unitree SLAM（QT_Server + LIO-SAM 定制版）
> 工作目录：`/unitree/module/graph_pid_ws`（下称"工作区"）
> 地图目录：`/unitree/module/graph_pid_ws/src/task/maps/pcd/default`（下称"地图目录"）

---

## 1. 启动前检查（30 秒）

电脑上：

```bash
ssh -t go2-wifi            # 登录机器人（首次会让你选 ROS 版本，输入 1 = ros:foxy）
```

登录后确认：

```bash
ip -4 addr show eth0 | grep 192.168.123.18   # 网卡地址在
ping -c1 -W1 192.168.123.20                  # 雷达能通
ps -ef | grep -E "UnitreeSlam|demo_mid360" | grep -v grep   # 无残留进程
```

- 雷达 IP：`192.168.123.20`；机器人 eth0：`192.168.123.18`
- 若上次异常退出留下进程，先 `sudo pkill -f UnitreeSlam; pkill -f demo_mid360` 再继续。

---

## 2. 启动（两个终端）

### 终端 A：启动 SLAM 服务（保持不关）

```bash
ssh -t go2-wifi
cd /unitree/module/graph_pid_ws
./0_unitree_slam.sh
```

看到 `RemakeMatrix success!` 之类的输出即正常，**这个终端全程开着**。

### 终端 B：启动交互程序

```bash
ssh -t go2-wifi
cd /unitree/module/graph_pid_ws
./run_demo.sh
```

> ⚠️ **不要**直接运行 `./demo_mid360 eth0`。
> `demo_mid360` 必须使用自带的 CycloneDDS（RUNPATH 里的 `unitree_robotics/lib/aarch64`）。
> 直接运行会报 `undefined symbol: ddsi_sertype_v0`（加载了 ROS 的旧版）或一段 malloc 崩溃。
> `run_demo.sh` 已经处理好了这些环境变量。

出现按键菜单：

```
***********************  Unitree SLAM Demo ***********************
  q w e / a s d / z x c v / p
```

### （可选）终端 C：盯关键帧数量

```bash
watch -n 2 'ls /unitree/module/graph_pid_ws/src/task/maps/pcd/default/Scans | wc -l'
```

每走约 0.5 m 或转约 6° 会增加一个 `xxxxxx.pcd`，不增长说明没在正常建图。

---

## 3. 建图 / 定位 / 导航流程

| 按键 | 作用 | 说明 |
|---|---|---|
| `w` | 开始建图 | 会清空已有拓扑点/边；等待约 15 秒启动雷达和 LIO-SAM |
| `e` | 结束建图并保存 | 地图写入地图目录，见第 4 节 |
| `z` | 重定位 + 位姿初始化 | **复用旧图时直接按**（先别按 `w`），狗要站在建图起点；首次建图则要 `e` 之后再按 |
| `p` | 打印当前 SLAM 坐标 | 定位成功后才有输出；建图阶段提示 `No SLAM odometry received yet.` 是正常的 |
| `x` | 采集当前位置为导航拓扑点 | 可多次 |
| `c` | 保存拓扑点和边 | 采完点后按 |
| `a` | 开始循环导航 | |
| `s` | 暂停导航 | |
| `d` | 恢复导航 | |
| `v` | 删除所有拓扑点和边 | |
| `q` | 关闭所有功能节点 | 不退出 demo 本身 |
| `Ctrl+C` | 退出 demo | |

### 3.1 首次建图（完整流程）

1. 把狗放到建图起点，按 `w`，等日志出现 `Waitting 15s to open mapping node.` → `Start mapping.`
2. 遥控机器狗**慢速**走完目标区域（尽量走闭环，转弯放缓）。
3. 回到起点附近，按 `e`，等日志出现 `Waitting 5s to save pcd map.` 和 `save PCD map success.`
4. 回到**建图起点**，按 `z`（重定位），再按 `p` 确认坐标正常。
5. 在需要的位置按 `x` 采导航拓扑点，全部采完按 `c` 保存（采法见第 6 节）。
6. 按 `a` 开始循环导航；`s`/`d` 暂停/恢复。
7. 结束时按 `q` 关闭节点，再 `Ctrl+C` 退出 demo。

### 3.2 日常复用（不重建图）

已经有图、只想直接定位导航时：

1. 把狗放到**建图时的起点**（当初按 `w` 的位置，朝向尽量一致）。
2. 终端 A `./0_unitree_slam.sh`，终端 B `./run_demo.sh`。
3. **直接按 `z`**（不要按 `w`！），等约 15 秒。
4. 按 `p` 确认位姿接近建图起点；坐标稳定后再按 `a` 导航。
5. `s`/`d` 暂停/恢复，`q` 收工，`Ctrl+C` 退出。

> 为什么不能按 `w`：按 `w` 会进建图模式并删掉 `SCDs/`、`Scans/`，再按 `e` 会把 `GlobalMap.pcd` 覆盖掉。
> 详细说明见第 5 节「地图复用」。

---

## 4. 地图保存与检查

保存位置（按 `e` 后）：

```bash
ls -lh /unitree/module/graph_pid_ws/src/task/maps/pcd/default/
```

| 文件 | 内容 |
|---|---|
| `GlobalMap.pcd` | **整张地图（导航用这个）** |
| `CornerMap.pcd` / `SurfMap.pcd` | 角点/面点地图 |
| `Scans/`、`SCDs/` | 关键帧点云 / ScanContext 描述子（重定位用） |
| `trajectory.pcd`、`transformations.pcd`、`optimized_poses.txt` | 轨迹与位姿 |

快速确认 `GlobalMap.pcd` 完整：

```bash
head -12 /unitree/module/graph_pid_ws/src/task/maps/pcd/default/GlobalMap.pcd | grep -E "POINTS|DATA"
# POINTS 后面的数 × 16 字节 + 头部 ≈ 文件大小
```

下载到电脑查看：

```bash
scp go2-wifi:/unitree/module/graph_pid_ws/src/task/maps/pcd/default/GlobalMap.pcd ~/go2_unitree_slam/GlobalMap.pcd
pcl_viewer ~/go2_unitree_slam/GlobalMap.pcd      # 按 h 查看鼠标操作
```

### 重新建图前清空旧地图

```bash
D=/unitree/module/graph_pid_ws/src/task/maps/pcd/default
rm -rf "$D"/* && mkdir -p "$D/Scans" "$D/SCDs"
```

（`w` 会清空拓扑点/边，但不会删除上一轮的 pcd 文件。）

---

## 5. 地图复用（一次建图，长期使用）

建完一次图之后**不需要每次开机都重建**。只要地图文件还在，开机按 `z` 重定位就能加载旧图，然后 `a` 直接导航。

### 5.1 复用会用到的文件

| 用途 | 文件 | 说明 |
|---|---|---|
| 定位（必需） | `地图目录/GlobalMap.pcd` | `lio_sam_ros2_globalLocalize` 启动时加载，路径写死在二进制里 |
| 初始位姿 | `Make_Graph_node.txt` 的**第一行** | `lio_sam_ros2_IniPoseFromText` 读出来发 `/initialpose` |
| 导航路径 | `Make_Graph_node.txt` + `Make_Graph_edge.txt` | 拓扑点/边，规划用（见第 6 节） |
| 回环/关键帧 | `地图目录/SCDs/`、`Scans/` | 定位不读，但必须保留 |

拓扑文件在 `src/QT_Server/graph_map/` 和 `src/graph_process/graph_map/` 下各有一份副本。按 `c` 保存后，用下面命令确认哪份被更新，备份时一起备份：

```bash
find /unitree/module/graph_pid_ws/src -name "Make_Graph_*.txt" -newermt "-10 min" -exec ls -l {} \;
```

### 5.2 复用前必须满足的三个条件

1. **地图目录完整**：`GlobalMap.pcd` 存在，大小合理（房间级 10–30 MB）。
2. **狗站在建图起点**：初始位姿取自拓扑节点 0，也就是当初按 `w` 时狗的位置和朝向。放偏或朝向差太多，重定位会收敛到错误位置**而且不报错**，只能靠按 `p` 发现。
3. **本次开机没按过 `w`**：`z` 只在 `free` 状态可用，按过 `w` 会变成 mapping 状态，`z` 会回 `The current system state is not free`。

### 5.3 日常复用流程

```text
终端 A:  ./0_unitree_slam.sh
终端 B:  ./run_demo.sh
         z        # 重定位（不要按 w），等约 15 s
         p        # 确认位姿接近建图起点（一般 0,0,0 附近）
         a        # 开始导航
         s / d    # 暂停 / 恢复
         q        # 收工（关节点），再 Ctrl+C
```

- 定位收敛前别急着按 `a`，先 `p` 看两三次坐标是否稳定。
- 起步慢走 1–2 m，让 LIO-SAM 和地图对齐。
- 导航目标点来自之前按 `x` 采的点；没采过点按 `a` 会提示 `The navigation has been opened, please input the target node.`

### 5.4 绝对不要做的事

| 动作 | 后果 |
|---|---|
| 有图时按 `w` | `mapOptmization` 启动即 `rm -r` 掉 `SCDs/`、`Scans/`，随后按 `e` 会覆盖 `GlobalMap.pcd`，这张图就废了 |
| 没回到起点就按 `z` | 位姿初始化错误，导航跑偏，而且不报错 |
| 手动 `rm` 地图目录 | 没有版本管理，删了就没了 |

### 5.5 备份与恢复

每次建完图、验证成功后立刻备份：

```bash
cd /unitree/module/graph_pid_ws/src/task/maps/pcd
cp -a default default_bak_$(date +%Y%m%d)
du -sh default_bak_*
```

恢复（必须在 `q` + `Ctrl+C` 关掉所有节点之后）：

```bash
cd /unitree/module/graph_pid_ws/src/task/maps/pcd
rm -rf default && cp -a default_bak_20261019 default
```

### 5.6 多张地图切换

`relio_qt_test.launch.py` 支持 `map_name:=<名字>`，但 QT_Server 只传 `default`，所以实际做法是**换目录**：

```bash
cd /unitree/module/graph_pid_ws/src/task/maps/pcd
mv default default_lab
mv default_home default          # 切到另一张图
```

拓扑文件要跟着一起换，否则初值和导航点对不上。

### 5.7 复用能撑到什么程度

| 情况 | 影响 |
|---|---|
| 家具/货架小幅移动 | 一般没事，靠代价地图避障 |
| 大范围改造、隔断变化 | 重定位会飘，需要重建图 |
| 建图起点被挡住 | 这张图基本用不了（初值固定在节点 0），只能重建图 |
| 上下楼/多层 | 单张图不适用，需要多层地图 |

---

## 6. 导航拓扑（采点与路径规划）

建图给你一张**点云地图**（用来定位），拓扑给你一张**行走图**（用来到处跑）。两者独立保存，但必须成对使用：换图要换拓扑，换拓扑也要换图。

### 6.1 拓扑是什么

| 文件 | 内容 |
|---|---|
| `Make_Graph_node.txt` | 路点：编号 x y z yaw 点性质 门点id 楼层id 点性质2 |
| `Make_Graph_edge.txt` | 边：边号 起点 终点 权重 步态 倒退 速度 状态×5 |

- 按 `x` 存的是**当前 SLAM 位姿**，不是整条轨迹
- 相邻两次 `x` 之间自动生成一条边，权重 = 两点直线距离
- 边还带属性：**步态**（0 普通 / 3 爬楼梯）、**是否倒退**、**速度**（默认 0.5 m/s）

### 6.2 和"录制回放"的区别

| | 录制回放 | 这套拓扑导航 |
|---|---|---|
| 录什么 | 完整轨迹（每帧位姿 + 速度） | 稀疏路点 + 连接关系 |
| 怎么跑 | 照原轨迹死走 | 运行时在图上**规划最短路径**，再 PID 跟踪 |
| 起点终点 | 只能原路重放 | 任意两个连通节点之间都能导 |
| 跑偏了 | 不纠偏 | 靠 SLAM 定位闭环纠正 |
| 多个目的地 | 要录多条 | 一张图，组合出走法 |
| 改速度/步态 | 要重录 | 改边属性即可（爬楼梯段设步态 3） |

结论：**一张图能跑出很多种路线**，比回放灵活得多。

### 6.3 采点规则

1. **停稳再按 `x`**：点位就是那一刻的 SLAM 位姿，走动着按会录偏
2. **转弯、门口必须放点**：边是两点之间的**直线**，弧形走廊只放两端会切墙角
3. **直线段可以稀一点**（2–3 m 一个点），弯道密一点
4. **按行进顺序走**：`x` 是「加当前点 + 连到上一个点」
5. **0 号点是建图起点**：它同时是重定位初值的来源，别删
6. **采完必须按 `c`**：不保存等于没采

### 6.4 保存后验证

```bash
cd /unitree/module/graph_pid_ws/src/QT_Server/graph_map
ls -l Make_Graph_node.txt Make_Graph_edge.txt      # 不应该再是 0 字节
cat Make_Graph_node.txt                            # 每行一个节点
cat Make_Graph_edge.txt                            # 每行一条边

# 如果上面还是空的，说明写到了别的副本，用这条找出真正被更新的文件：
find /unitree/module/graph_pid_ws/src -name "Make_Graph_*" -newermt "-5 min" -exec ls -l {} \;
```

### 6.5 运行时发生了什么

按 `a` 会依次发出 4 条命令：

```
startRelocation()    → cmd 5   启动重定位
startNavigation()    → cmd 8   打开导航节点（打印 "please input the target node"）
initPose()           → cmd 13  初始化位姿
defaultNavigation()  → cmd 10  按拓扑自动导航（multi_nav_all）
```

之后：

```
graph_process/path_management   在图上算路径
graph_process/main_process      按边逐段下发目标
pid_tracing                     纯跟踪控制，边走边用 SLAM 纠正
demo 打印 " I arrived node N"    表示到达某个节点
```

> `please input the target node` 是留给 **Qt 客户端**（在 GUI 上点某个节点）的提示，
> 键盘 demo 没有发单个目标的能力，**忽略即可**——真正让它跑起来的是随后那条 `defaultNavigation`。

### 6.6 常见坑

| 现象 | 原因 |
|---|---|
| 狗走直线撞墙角 | 弧线/转角没放点，边是直线 |
| 每条边都跑偏一点 | 采点时狗在动，或者定位没收敛就按了 `x` |
| 按 `a` 后原地不动 | 拓扑是空的，先 `x` 采点再 `c` 保存 |
| 换图后导航全乱 | 拓扑没跟着换，点位和地图对不上 |
| 重定位初值不对 | 0 号节点被删了，或者停的位置离 0 号点太远 |

---

## 7. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `undefined symbol: ddsi_sertype_v0` | 直接运行了 `./demo_mid360`。改用 `./run_demo.sh` |
| `free(): invalid pointer` / malloc 崩溃 | 同上，`LD_LIBRARY_PATH` 里混入了其它 CycloneDDS |
| `The current system state is not free`（按 `z` 时刷一串） | 已经按过 `w`，处于 mapping 状态。复用旧图时：重启终端 A/B，**直接按 `z`**；首次建图时：先按 `e` 结束建图再按 `z` |
| 按 `p` 显示的位姿不对（不在起点附近） | 初值取自 `Make_Graph_node.txt` 第一行，狗没站在建图起点，或朝向差太多 |
| 按 `a` 提示 `please input the target node` | 还没采过导航拓扑点，用 `x` 采点、`c` 保存 |
| `No SLAM odometry received yet.` | 建图阶段读不到定位里程计，正常；`e` + `z` 之后再看 `p` |
| `Large velocity, reset IMU-preintegration!` | IMU 单位/零偏问题（已修）。若复现，见第 8 节重新测零偏 |
| 保存后出现 `process has died ... exit code -11`（mapOptimization） | **已知问题**：保存写完 `GlobalMap.pcd` 后在写 `cloudSurf.pcd`/`cloudGlobal.pcd` 时崩溃。导航不依赖这两个文件，`GlobalMap.pcd` 已完整，可忽略 |
| 地图看起来整体倾斜 | 检查 `MID360_config.json` 的 `extrinsic_parameter` 是否为 0（已置 0，见第 8 节） |
| `ros2 topic echo` / `bag record` 段错误 | 先 `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` 再用 |
| 关键帧数量不增长 | 检查雷达是否在转、`/livox/lidar` 是否有数据、是否在 `mapping` 状态 |

---

## 8. 维护信息（本机做过的修改）

### 8.1 `run_demo.sh`

`/unitree/module/graph_pid_ws/run_demo.sh`：source ROS 环境后，用 `env -u LD_LIBRARY_PATH` 启动 demo，避免加载错误版本的 CycloneDDS。**不要删除或改回普通启动方式。**

### 8.2 Livox IMU 零偏扣除（已打在驱动里）

- 补丁：`src/livox_ros_driver2/src/lddc.cpp` 里发布 IMU 时扣除 `gyro - bias`
- 零偏文件：`/home/unitree/livox_gyro_bias.txt`（3 个数，rad/s，当前 `-0.0678 0.0016 0.0035`）
- 应用脚本（幂等，可重复执行）：`sudo bash /home/unitree/patch_livox_gyro_bias.sh`
- 修改零偏文件后需**重启建图会话**（驱动启动时读取一次）
- 原始文件备份：`lddc.cpp.bak-gyrobias-*`

重新测零偏（雷达已启动、狗静止）：

```bash
bash -lc 'set +u; source /opt/ros/foxy/setup.bash; source /home/unitree/imu_probe_ws/install/setup.bash; \
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
/home/unitree/imu_probe_ws/install/imu_probe/lib/imu_probe/imu_probe 15 /livox/imu'
```

把输出的 `gyr_x/gyr_y/gyr_z mean` 写回 `/home/unitree/livox_gyro_bias.txt` 即可。

### 8.3 雷达安装外参已清零

`MID360_config.json`（`src` 和 `install` 各一份）里的 `extrinsic_parameter` 已改为全 0（原值是 `1.57079632 / 3.14159265`，被当"度"解释造成 ~1.5° 的系统倾斜）。备份文件：`MID360_config.json.bak-*`。

### 8.4 LIO-SAM 配置

- `src/lio_sam_ros2/config/params_mid360.yaml`：`imuTopic: livox/imu`、`extrinsicTrans [0,0,0]`、`extrinsicRot/RPY` 单位阵
- 建图/定位 launch 中已注释掉 Hesai 专用的 `lidar_checkout` 节点
- 建图 launch 的 `use_sim_time: False`

### 8.5 相关脚本

| 路径 | 用途 |
|---|---|
| `/home/unitree/patch_livox_gyro_bias.sh` | 打零偏补丁并重编译驱动（sudo） |
| `/home/unitree/build_livox.sh` | 只重编译 livox 驱动 |
| `/home/unitree/imu_probe_ws/` | IMU 静态测量探针（C++ 节点） |
| `/home/unitree/imu_static_check.py` | 早期 Python 探针（本机 CLI 订阅会崩，建议用上面的 C++ 版） |

---

## 9. 一键检查清单

### 9.1 首次建图

- ☐ `ssh -t go2-wifi` 能登录，菜单选 1（ros:foxy）
- ☐ `ping 192.168.123.20` 通
- ☐ 终端 A 运行 `./0_unitree_slam.sh`
- ☐ 终端 B 运行 `./run_demo.sh`（不是 `./demo_mid360`）
- ☐ 按 `w`，日志出现 `Start mapping.`
- ☐ 走完按 `e`，日志出现 `save PCD map success.`
- ☐ `GlobalMap.pcd` 存在且大小合理（房间级一般 10–30 MB）
- ☐ 回起点按 `z`，再按 `p` 有坐标输出
- ☐ `x` 按顺序采点（转弯/门口必放）、`c` 保存、`a` 导航
- ☐ 采完确认 `Make_Graph_node.txt` / `Make_Graph_edge.txt` 不是 0 字节
- ☐ **立刻备份地图**（见 5.5）
- ☐ 结束 `q` + `Ctrl+C`

### 9.2 日常复用（不重建图）

- ☐ 狗停在**建图起点**，朝向与建图时一致
- ☐ 终端 A `./0_unitree_slam.sh`，终端 B `./run_demo.sh`
- ☐ **直接按 `z`**（全程没按 `w`），等约 15 s
- ☐ `p` 看两三次，位姿稳定且接近节点 0
- ☐ `a` 开始导航，`s`/`d` 暂停恢复
- ☐ `q` + `Ctrl+C` 收工
