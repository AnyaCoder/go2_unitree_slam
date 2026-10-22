# Go2 Full FAR Navigation

Unitree Go2 EDU + Livox MID-360 navigation integration built on the upstream
`jizhang-cmu/autonomy_stack_go2` algorithms.

## Final architecture

The Go2 owns the complete planning and execution loop:

```text
LIO global relocalization (~3 Hz)
  -> nav_pose_bridge -> /nav/state_estimation (~50 Hz)

fixed navigation PCD
  -> static_map_replay -> 0.08 m voxel, 6 m x 6 m window, ~5 Hz
  -> terrainAnalysis -> terrainAnalysisExt
  -> FAR -> /way_point
  -> localPlanner -> /path
  -> upstream pathFollower
  -> /api/sport/request
```

The laptop is display-only and transports two small control inputs:

```text
same fixed PCD -> /overall_map (RViz background)
TCP 8898 <- display-only state/TF/joints/path/waypoint/FAR Marker topics
TCP 8899 -> final /goal_point
TCP 8900 -> selected /initialpose
```

The laptop must not run `pcdMapSim`, `terrainAnalysis`, `terrainAnalysisExt`,
`far_planner`, `localPlanner`, or `pathFollower`. No live sensor cloud is sent
to the laptop.

`nav_pose_bridge` treats LIO's relocalization odometry as the only pose source
and repeats its newest value at 50 Hz for the onboard planners. The
IMU-preintegration topic is deliberately excluded because it drifts while the
Go2 is stationary. The display uplink is independently limited to 30 Hz state,
20 Hz joints and 1 Hz heavy FAR Marker topics, and keeps only the newest pending
frame per topic so a slow Wi-Fi link cannot replay stale poses. Planning does not consume the Livox scan:
`static_map_replay` reproduces the accepted `map_sim` input exactly from the
fixed navigation PCD (deterministic 0.08 m voxel, 6 m x 6 m window, 5 Hz).
Terrain classification also uses the `map_sim` parameters, including
`vehicleHeight=0.45` and `noDataObstacle=false`.

## Requirements

- Unitree Go2 EDU with Livox MID-360
- Ubuntu 20.04 and ROS 2 Foxy
- `autonomy_stack_go2` installed on both machines
- UnitreeSlam / `graph_pid_ws` on the Go2
- SSH alias `go2-wifi`
- Fixed display background:

```text
GlobalMap_clean_20261020_1920_nav_green_edited.pcd
```

## Repository layout

- `nav/`: laptop display, RViz, acceptance checks and operator entry points.
- `nav_dog/`: pose relay, fixed-PCD replay, rate audit and TCP bridges run on
  the Go2.
- `nav_dog_cpp/`: optional C++ live-cloud adapter; disabled in the accepted
  fixed-PCD configuration.
- `tests/`: ownership, transport, pose-math and fixed-map regression tests.
- `tools/`: point-cloud cleanup and walkable-area editing utilities.
- `docs/` and `必看.md`: operating procedure and acceptance contract.
- The remaining top-level `include/`, `lib/`, `example/`, `cmake/` and
  `thirdparty/` content belongs to the bundled Unitree SDK and is not part of
  the navigation runtime.

## Start

```bash
cd ~/go2_unitree_slam
./nav/start_far_real.sh
```

Then in RViz:

1. Click `2D Pose Estimate` and wait for successful relocation.
2. Confirm the Go2 model and real odometry are visible.
3. Click `Goalpoint`.
4. The dog-side FAR produces `/way_point`; `localPlanner` produces `/path`;
   `pathFollower` drives the robot.

Before the first Goalpoint, `localPlanner` starts with `goalX/goalY = NaN`; no
autonomous path or motion should be generated.

## Useful commands

```bash
./nav/view_real.sh           # attach to existing data and open RViz
./nav/stop_view.sh           # stop only laptop-side view processes
./nav/check_real_chain.sh    # read-only ownership/link acceptance
./nav/audit_real_rates.sh    # read-only 50/5 Hz runtime audit
./nav/stop_real_all.sh       # stop the project's robot and laptop processes
./nav/run_map_sim.sh         # local visual reference using upstream map_sim
```

## Runtime rate acceptance

After relocation and once the planner is running, measure the real robot rather
than inferring rates from the two source topics:

```bash
./nav/audit_real_rates.sh --duration 15
./nav/audit_real_rates.sh --duration 15 --strict
```

The target chain is approximately:

```text
/lio_sam_ros2/mapping/re_location_odometry   ~3 Hz
/nav/state_estimation                        50 Hz
/nav/static_cloud_map                        5 Hz
/terrain_map and /terrain_map_ext            5 Hz
/path, /cmd_vel, /api/sport/request          active while a Goalpoint runs
```

`--strict` checks required topics against the configured windows; optional
motion/path topics are reported but do not fail an otherwise idle chain.

## Control ownership

Only `pathFollower` may publish `/api/sport/request`. UnitreeSlam legacy
`dog_control_B1_one` and `go2_control_by_sdk send_cmd` are disabled by the
startup script. The physical remote controller remains the final takeover.

`必看.md` is the authoritative operating document. `nav_dog_cpp` is optional;
the default fixed-PCD chain does not require that workspace to be built.
