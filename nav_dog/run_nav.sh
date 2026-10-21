#!/usr/bin/env bash
# usage: ./run_nav.sh dry|live
#   SENSOR_OFFSET_X=0.187 ./run_nav.sh live  # 用实车测量值覆盖 MID360 默认值
set -e
MODE="${1:-dry}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=go2_mid360_hardware.env
source "${SCRIPT_DIR}/go2_mid360_hardware.env"
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ "$MODE" = "live" ]; then
  SPORT=/api/sport/request
  echo "=== LIVE: pathFollower will command the dog ==="
else
  SPORT=/nav/dry_cmd
  echo "=== DRY RUN: motion commands go to /nav/dry_cmd ==="
fi
exec ros2 launch /home/unitree/go2_nav/go2_nav.launch.py \
  sport_out:=$SPORT \
  sensor_offset_x:="$SENSOR_OFFSET_X" \
  sensor_offset_y:="$SENSOR_OFFSET_Y" \
  max_speed:="$GO2_MAX_SPEED" \
  start_far:=false
