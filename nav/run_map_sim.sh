#!/usr/bin/env bash
# Local map simulation only. No Go2 connection, no robot state receiver, no
# command bridges, and no artificial walkable-area layer.
set -euo pipefail

ROOT="${HOME}/go2_unitree_slam"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
LOG_DIR="${LOG_DIR:-/tmp}"
MAP="${MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
LAUNCH_LOG="${LOG_DIR}/go2_map_sim_project.log"
RVIZ_LOG="${LOG_DIR}/go2_map_sim_rviz.log"
RVIZ_CONFIG="${ROOT}/nav/far_real.rviz"

[[ -f "$MAP" ]] || { echo "找不到固定静态地图: $MAP" >&2; exit 2; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "找不到 RViz 配置: $RVIZ_CONFIG" >&2; exit 2; }

terminate_matching() {
  local pattern="$1" pid
  while IFS= read -r pid; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

# Stop only the previous local simulation/view stack.
terminate_matching 'python3 .*/nav/far_viz_receiver\.py'
terminate_matching 'python3 .*/nav/waypoint_sender\.py'
terminate_matching 'python3 .*/nav/initialpose_sender\.py'
terminate_matching '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch map_sim map_sim\.launch\.py'
terminate_matching '/home/anyacoder/autonomy_stack_go2/install/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/terrain_analysis/lib/terrain_analysis/'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/terrain_analysis_ext/lib/terrain_analysis_ext/'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/local_planner/lib/local_planner/'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/far_planner/lib/far_planner/'
terminate_matching '/rviz2/rviz2 .*\.rviz'
terminate_matching '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher'
terminate_matching '^/opt/ros/foxy/lib/tf2_ros/static_transform_publisher'
sleep 1

set +u
source /opt/ros/foxy/setup.bash
source "${WS}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${ROOT}/nav/cyclonedds_lo.xml"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

PIDS=()
cleanup() {
  local i
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -TERM "${PIDS[$i]}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo '[1/2] 启动固定静态地图模拟'
ros2 launch map_sim map_sim.launch.py \
  pcd:="$MAP" \
  rviz:=false \
  robot_state_publisher:=true \
  space_viz:=false \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
PIDS+=("$LAUNCH_PID")

sleep 5
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  tail -n 120 "$LAUNCH_LOG" >&2 || true
  exit 5
fi

echo '[2/2] 启动 RViz'
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" >"$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!
PIDS+=("$RVIZ_PID")
sleep 2
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  tail -n 120 "$RVIZ_LOG" >&2 || true
  exit 6
fi

echo "本地模拟已启动"
echo "固定静态地图: $MAP"
echo "启动日志: $LAUNCH_LOG"
echo "RViz 日志: $RVIZ_LOG"
echo "人工可走区: disabled"
wait "$LAUNCH_PID" || true
