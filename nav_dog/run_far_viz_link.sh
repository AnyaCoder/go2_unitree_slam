#!/usr/bin/env bash
# Go2 uplink: odom, TF, joints, path, waypoint and native FAR visualization.
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export FAR_VIZ_PORT="${FAR_VIZ_PORT:-8898}"
exec python3 /home/unitree/go2_nav/far_viz_link.py --port "$FAR_VIZ_PORT"
