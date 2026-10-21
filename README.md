# Go2 Fixed-Map FAR Navigation

This repository contains the laptop-side and Go2-side integration for a
fixed-map navigation stack on Unitree Go2 EDU with a Livox MID-360.

The design keeps global planning on the laptop and local planning on the robot:

```text
laptop fixed PCD map
  -> pcdMapSim + FAR planner
  -> /goal_point and /way_point over TCP 8899
  -> Go2 localPlanner
  -> Go2 pathFollower
  -> /nav/path_follower_request
  -> nav_safety_gate (IMU + initial pose + localization + fresh Goalpoint/path)
  -> /api/sport/request
```

Robot odometry, TF and joint states are forwarded to the laptop over TCP 8898.
The robot does not run FAR or RViz. The physical remote controller remains the
final takeover mechanism.

## Upstream Algorithms

This repository is integration and operations glue, not a fork of either planner:

- FAR global planning comes from the original
  [FAR Planner](https://github.com/MichaelFYang/far_planner) algorithm used by
  [jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2).
- `localPlanner`, `pathFollower`, `terrainAnalysis` and `terrainAnalysisExt`
  are the original jizhang-cmu autonomy-stack implementations.
- Unitree SDK 2 at the repository root remains the upstream robot SDK.

Project-specific code under `nav/` and `nav_dog/` supplies static-map loading,
network isolation, telemetry, point transport, initial-pose handling, launch
orchestration and visualization. It does not replace the planner algorithms.

## Requirements

- Unitree Go2 EDU with Livox MID-360
- Ubuntu 20.04 and ROS 2 Foxy
- `autonomy_stack_go2` installed on both the laptop and the Go2
- UnitreeSlam and its `graph_pid_ws` workspace installed on the Go2
- SSH alias `go2-wifi` pointing to the robot
- One canonical map at:

```text
GlobalMap_clean_20261020_1920_nav_green_edited.pcd
```

The upstream Unitree SDK 2 directories remain at the repository root because
this project is built on top of that SDK. Third-party checkouts and generated
build workspaces are intentionally ignored.

## Quick Start

Start the complete real-robot stack from the laptop:

```bash
cd ~/go2_unitree_slam
./nav/start_far_real.sh
```

After RViz opens, keep the dog completely still for about 3 seconds so the IMU
health check can complete. Then use `2D Pose Estimate` once. Wait for the dog-side
`Pose initialization successful` / `Initializing Succeed` message and for
`/nav/safety_status` to report `goal_commands_ready=true`. Only then send a FAR
goal point. Motion is additionally blocked until that localization epoch has a
Goalpoint, a FAR waypoint and a local path. Goalpoints and waypoints do not
expire while the same goal remains active; the planner path must still refresh
at least once every 3 seconds.

Stop only the processes owned by this project:

```bash
./nav/stop_real_all.sh
```

Read-only acceptance check:

```bash
./nav/check_real_chain.sh
```

## Repository Layout

```text
nav/                 Laptop fixed-map, FAR, RViz, TCP receivers and launchers
nav_dog/             Go2 launch configuration, telemetry and command bridges
tools/               Map cleanup, quality report and walkable-area editor
docs/                Architecture, cold-start, display and known-issue notes
必看.md              Authoritative operating and safety rules
unitree_slam/        Local external checkout; ignored by Git
```

## Documentation

- [必看.md](必看.md): mandatory architecture and control-ownership rules
- [docs/README.md](docs/README.md): documentation index
- [Go2_Mid360_SLAM_操作手册.md](Go2_Mid360_SLAM_操作手册.md): mapping and operation guide
- [Go2点选导航_配置说明.md](Go2点选导航_配置说明.md): point-to-point navigation setup
- [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md): current safety and integration risks
- [docs/导航安全门控与初始化流程_20261021.md](docs/导航安全门控与初始化流程_20261021.md): fail-closed Goalpoint gate and test evidence

## Safety Boundary

When FAR navigation is active, the only allowed motion-control chain is:

```text
laptop FAR -> TCP 8899 -> Go2 localPlanner/pathFollower
  -> /nav/path_follower_request -> nav_safety_gate -> /api/sport/request
```

The gate starts disarmed and accepts only `StopMove`/`Move`. It requires stationary
IMU calibration, a current-epoch initial pose, confirmed localization, fresh
odometry, a FAR Goalpoint, a FAR waypoint and a local path generated for that
goal. The initial-pose distance/yaw check is performed once while acquiring
localization, not continuously during travel. A new initial pose invalidates all
old goals and paths; a stale IMU/odometry/path causes an immediate stop.

The legacy UnitreeSlam motion nodes
`dog_control_B1_one` and `go2_control_by_sdk send_cmd` must not run at the same
time. Always verify this before allowing the robot into locomotion mode.
