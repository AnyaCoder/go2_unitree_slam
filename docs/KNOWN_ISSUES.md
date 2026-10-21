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

### Mitigation now deployed

The original upstream FAR/local-planner sources remain unchanged. The integration
layer now remaps `pathFollower` output to `/nav/path_follower_request` and routes it
through `nav_dog/nav_safety_gate.py` before `/api/sport/request`. The gate starts
disarmed and requires stationary IMU calibration, a new initial pose, confirmed
localization, fresh odometry, a Goalpoint, an FAR waypoint and a local path
generated for that goal. The position/yaw match is a one-time acquisition check,
not a 5 m leash on the whole run. A Goalpoint or waypoint remains valid until the
goal changes. A new initial pose clears old goals and paths; stale odometry, IMU
or localPlanner path immediately revokes Move authorization and emits StopMove.

See `docs/导航安全门控与初始化流程_20261021.md`. This mitigation is covered by unit
tests and an isolated ROS-domain integration test. It still requires repeated real
robot acceptance testing before unattended operation.

## 2. `nav2_costmap` can crash during relocation

During one relocation run, `CloudToScanForCostmap` exited with a segmentation
fault. FAR control and SLAM odometry can continue, but the costmap process is not
part of the fixed-map FAR control path and should not be treated as a required
node by this project.

The relocation launch must retain LIO/global localization while suppressing
legacy motion-control/costmap processes that are not used by FAR navigation.
