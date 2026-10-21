#!/usr/bin/env bash
# Laptop global path -> Go2 look-ahead /way_point. No velocity input.
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=go2_mid360_hardware.env
source "${SCRIPT_DIR}/go2_mid360_hardware.env"
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export FAR_COMMAND_PORT="${FAR_COMMAND_PORT:-8899}"
exec python3 /home/unitree/go2_nav/global_path_receiver.py \
  --port "$FAR_COMMAND_PORT" \
  --sensor-offset-x "$FAR_SENSOR_OFFSET_X" \
  --sensor-offset-y "$FAR_SENSOR_OFFSET_Y"
