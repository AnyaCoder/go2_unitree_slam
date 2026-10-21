#!/usr/bin/env bash
# Dedicated laptop -> Go2 initial-pose channel. No waypoint or velocity commands.
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export GO2_INITIALPOSE_PORT="${GO2_INITIALPOSE_PORT:-8900}"
exec python3 /home/unitree/go2_nav/initialpose_link.py --port "$GO2_INITIALPOSE_PORT"
