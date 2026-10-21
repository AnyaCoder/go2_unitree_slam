# Known Issues

Updated: 2026-10-21

## 1. Stale local path can resume after Damping/Stand

### Symptom

The Go2 is stationary in Damping mode. After the operator switches it back to a
standing/locomotion mode, it can resume an old local path even though no new RViz
goal was just sent.

### Evidence

- `localPlanner` publishes `/path`;
- upstream `pathFollower` retains its last path state;
- no fresh path may be generated at the instant of standing.

### Root cause

Upstream `localPlanner` initializes `goalX=0`, `goalY=0` in autonomy mode.
The integration layer now replaces those values with NaN, so startup cannot
plan toward the map origin.

Upstream `pathFollower` can still retain `pathInit=true` after receiving a local
path. The final architecture stops new global-path input from retriggering
motion: the Go2 `global_path_receiver` holds the current body pose when the
global path is empty or older than 2 seconds. However, the unchanged upstream
`pathFollower` can retain the last locally planned `/path` across a
Damping/Stand transition.

### Current status

Implemented:

- laptop owns FAR on the fixed PCD;
- laptop sends only an ordered global path over TCP 8899;
- Go2 has no `far_planner`;
- only `global_path_receiver.py` publishes `/way_point`;
- no path -> current body pose;
- stale global path -> current body pose;
- localPlanner initial goal is NaN.

Not implemented in upstream algorithm:

- explicit clear of the retained local path on Damping/Stand transition;
- a pathFollower-local freshness timeout independent of input topic rate.

### Required behavior before unattended operation

Before unattended operation, verify a deployed planner revision provides:

- waypoint reception and active-goal state;
- local-path frame validation and minimum path length;
- local-path freshness and odometry freshness handling;
- a defined first motion request after Damping/Stand transitions;
- bounded waypoint speed and yaw rate.

Until those conditions are verified, do not let the robot enter locomotion mode
while a stale local path may exist. Keep the physical remote controller available
for takeover.

## 2. `nav2_costmap` can crash during relocation

During one relocation run, `CloudToScanForCostmap` exited with a segmentation
fault. FAR navigation and SLAM odometry can continue, but the costmap process is
not part of the fixed-map global-planning chain and must not be treated as a
required node.

The relocation launch must retain LIO/global localization while suppressing
legacy motion-control/costmap processes not used by this architecture.
