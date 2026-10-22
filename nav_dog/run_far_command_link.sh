#!/usr/bin/env bash
# Laptop final Goalpoint -> Go2 /goal_point. No waypoint or velocity input.
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export FAR_COMMAND_ENABLE="${FAR_COMMAND_ENABLE:-1}"
export FAR_COMMAND_PORT="${FAR_COMMAND_PORT:-8899}"
exec python3 /home/unitree/go2_nav/far_command_link.py --port "$FAR_COMMAND_PORT"
