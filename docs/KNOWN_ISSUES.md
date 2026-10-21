# Known Issues

Updated: 2026-10-21

## 1. Robot can resume an old local path after standing

### Symptom

The Go2 is stationary in Damping mode. When the operator switches it to a
standing/locomotion mode, it can start moving forward immediately even though no
new RViz goal was sent.

### Evidence

Runtime inspection showed:

- the laptop waypoint sender had `sent=0`;
- no fresh `/goal_point` was published;
- `pathFollower` still published `/api/sport/request`;
- `localPlanner` publishes `/path`, while `pathFollower` retains the last path.

### Root cause

The upstream `localPlanner` initializes `goalX=0` and `goalY=0`. In autonomy
mode it does not have a separate "has a goal ever been received" state, so it
can plan toward the map origin before the operator sends a waypoint.

The upstream `pathFollower` retains `pathInit=true` after receiving a local path.
Without a path timeout it can continue issuing motion requests based on stale
path data across Damping/Stand transitions.

### Current status

The running system uses the original upstream
`jizhang-cmu/autonomy_stack_go2` FAR/local-planner implementation. This
repository documents the issue but does not vendor or replace those planner
sources.

### Required fix before unattended operation

Before unattended operation, verify that the deployed upstream planner revision
provides, or that the integration layer enforces:

- waypoint reception and active-goal state;
- local-path frame validation and minimum path length;
- path and odometry freshness timeouts;
- one-shot `StopMove` when the path, goal or odometry becomes stale;
- bounded waypoint speed and yaw rate.

Until those safety conditions are deployed and verified, do not let the robot enter
locomotion mode while a stale FAR goal or local path may exist.

## 2. `nav2_costmap` can crash during relocation

During one relocation run, `CloudToScanForCostmap` exited with a segmentation
fault. FAR control and SLAM odometry can continue, but the costmap process is not
part of the fixed-map FAR control path and should not be treated as a required
node by this project.

The relocation launch must retain LIO/global localization while suppressing
legacy motion-control/costmap processes that are not used by FAR navigation.
