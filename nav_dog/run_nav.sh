#!/usr/bin/env bash
# usage: ./run_nav.sh dry|live
# Runs pose fusion -> fixed-PCD replay -> terrain -> FAR -> planners on the dog.
set -e
MODE="${1:-dry}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=go2_mid360_hardware.env
source "${SCRIPT_DIR}/go2_mid360_hardware.env"
STATIC_MAP_PATH="${GO2_STATIC_MAP_PATH:-/home/unitree/go2_nav/maps/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
# nav_dog_cpp is optional; the default fixed-PCD chain does not require it.
if [[ -f /home/unitree/go2_nav_ws/install/setup.bash ]]; then
  source /home/unitree/go2_nav_ws/install/setup.bash
fi
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ "$MODE" = "live" ]; then
  SPORT=/api/sport/request
  echo "=== LIVE: FAR/localPlanner/pathFollower drive the Go2 ==="
else
  SPORT=/nav/dry_cmd
  echo "=== DRY RUN: motion commands go to /nav/dry_cmd ==="
fi
exec ros2 launch /home/unitree/go2_nav/go2_nav.launch.py \
  sport_out:=$SPORT \
  sensor_offset_x:="$SENSOR_OFFSET_X" \
  sensor_offset_y:="$SENSOR_OFFSET_Y" \
  max_speed:="$GO2_MAX_SPEED" \
  static_map_path:="$STATIC_MAP_PATH" \
  start_static_replay:=true \
  start_far:=true
