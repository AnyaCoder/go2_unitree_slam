# Known Issues

Updated: 2026-10-22

## 1. Fixed-PCD terrain and FAR run on the Go2

The current architecture deliberately runs the complete upstream loop on the
robot:

```text
static_map_replay -> terrainAnalysis -> terrainAnalysisExt -> FAR
-> localPlanner -> pathFollower
```

`nav_dog/static_map_replay.py` reproduces the accepted `map_sim` input from the
fixed navigation PCD: deterministic 0.08 m voxel, 6 m x 6 m rolling window, 5 Hz.
It publishes only after receiving a pose less than one second old. The planning
input and laptop background therefore come from the same PCD, not from a live
scan. Use `./nav/audit_real_rates.sh` to verify `/nav/static_cloud_map`.

The laptop is display-only. It must not start `pcdMapSim`, terrain, FAR,
`localPlanner`, or `pathFollower`.

Verification must be performed with `./nav/check_real_chain.sh`. A green display
alone does not prove that planning and execution are using the same topic graph.

## 2. Startup Goalpoint suppression

`localPlanner` starts with `goalX/goalY = NaN` to prevent a relocalized robot
from driving toward map origin. A valid FAR `/way_point` is required before a
local `/path` and autonomous motion can be produced.

Do not replace these values with zero in a launch override.

## 3. Legacy UnitreeSlam motion controllers

`dog_control_B1_one` and `go2_control_by_sdk send_cmd` must remain disabled while
this stack is active. Only `pathFollower` may publish `/api/sport/request`.

The physical remote controller remains the final takeover mechanism.

## 4. No live cloud is transported to the laptop

TCP 8898 carries state, joints, paths and native FAR marker messages only. The
uplink is display-only: state is limited to 30 Hz, joints to 20 Hz, and heavy
Marker topics to 1 Hz. It coalesces by topic so a stalled Wi-Fi link resumes
with current data instead of replaying stale frames. The laptop's fixed PCD is
a visual background; the dog uses a local copy of the same file for planning.
Real-time point-cloud upload is intentionally unsupported.

## 5. LIO output rate is decoupled from navigation state rate

LIO-SAM global localization runs at about 3 Hz on the Go2. The planner must not
consume that topic directly. `nav_pose_bridge` combines it with the existing
IMU preintegration odometry and publishes a 50 Hz map-frame state. The deskewed
10 Hz cloud is used for LIO. The fixed-PCD replay uses the fused state to emit
the simulator's 5 Hz planning cloud.

If `/nav/state_estimation` disappears or `/stop` becomes 15, do not bypass the
bridge; first restore global localization and high-rate IMU odometry. Use
`./nav/audit_real_rates.sh --duration 15 --strict` to distinguish a source-rate
problem from a bridge, static-replay, or planner-rate problem.

## 6. `nav2_costmap` is outside this navigation loop

Relocation must retain LIO/global localization while suppressing
`nav2_costmap` and legacy motion-control processes. A costmap crash is not a
failure of the FAR/localPlanner/pathFollower execution chain.
