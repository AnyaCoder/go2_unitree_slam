# Go2 Fixed-Map FAR Navigation

This repository contains the laptop-side and Go2-side integration for a
fixed-map navigation stack on Unitree Go2 EDU with a Livox MID-360.

Planning is split at the network boundary:

```text
Laptop fixed PCD
  -> pcdMapSim -> terrain -> FAR
  -> ordered global path over TCP 8899
  -> Go2 global_path_receiver -> /way_point
  -> Go2 localPlanner -> pathFollower
  -> /api/sport/request
```

The Go2 never runs FAR. The laptop never runs `localPlanner` or `pathFollower`.
Robot odometry, TF, joints, the Go2 local execution path and a compact 1 Hz
near-field cloud are forwarded to the laptop over TCP 8898. The laptop's FAR
consumes the fixed PCD through `pcdMapSim` and follows the robot's real
relocalized pose. RViz Goalpoint feeds the laptop-local `/goal_point`; only a
small, ordered FAR polyline crosses TCP 8899. `/way_point` is published on the
Go2 by `global_path_receiver.py` only. The laptop FAR waypoint is remapped to
the local-only `/far/way_point` topic. The compact cloud is display-only, and
the physical remote controller remains the final takeover mechanism.

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

After RViz opens, use `2D Pose Estimate` once. Wait until the receiver logs show
real SLAM odometry, TF and joint data. Then use Goalpoint: the laptop FAR plans
on the fixed map and sends the resulting ordered path to the Go2.

The dog-side launch replaces upstream localPlanner's initial `(0, 0)` target
with NaN. `global_path_receiver.py` publishes the current relocalized body pose
as a hold target until a valid global path arrives, and falls back to that hold
target if the path becomes empty or stale. This prevents startup motion toward
the map origin or continued motion along an old path.

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
nav/                 Laptop fixed-map FAR, global-path sender, display and RViz
nav_dog/             Go2 terrain/local execution, path receiver and telemetry
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

## Safety Boundary

When navigation is active, the only allowed motion-control chain is:

```text
Laptop RViz Goalpoint -> Laptop FAR
  -> ordered global path over TCP 8899
  -> Go2 global_path_receiver -> /way_point
  -> Go2 localPlanner/pathFollower
```

The Go2 must not run FAR. The laptop must not run `localPlanner` or
`pathFollower`, and the laptop must never publish `/way_point`. Only
`global_path_receiver.py` may publish the dog-side public `/way_point`.

The legacy UnitreeSlam motion nodes
`dog_control_B1_one` and `go2_control_by_sdk send_cmd` must not run at the same
time. Always verify this before allowing the robot into locomotion mode.

The old `waypoint_sender.py` / `far_command_link.py` path is retained only as
historical code and is not part of the startup chain.
