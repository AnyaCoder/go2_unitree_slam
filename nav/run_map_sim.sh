#!/usr/bin/env bash
# Restore the known-working local simulation: upstream map_sim executes the
# virtual Go2, while the fixed PCD remains the displayed map background.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
# shellcheck source=../nav_dog/go2_mid360_hardware.env
source "${ROOT}/nav_dog/go2_mid360_hardware.env"
LOG_DIR="${LOG_DIR:-/tmp}"
MAP="${MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
LAUNCH_LOG="${LOG_DIR}/go2_map_sim_project.log"
MAP_LOG="${LOG_DIR}/go2_static_map.log"
RVIZ_LOG="${LOG_DIR}/go2_map_sim_rviz.log"
RVIZ_CONFIG="${ROOT}/nav/map_sim.rviz"

[[ -f "$MAP" ]] || { echo "找不到固定静态地图: $MAP" >&2; exit 2; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "找不到 RViz 配置: $RVIZ_CONFIG" >&2; exit 2; }

terminate_matching() {
  local pattern="$1" pid pids=()
  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    pids+=("$pid")
    kill -TERM "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
  [[ ${#pids[@]} -gt 0 ]] || return 0
  sleep 0.5
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

terminate_matching 'python3 .*/nav/static_map_publisher\.py'
terminate_matching 'python3 .*/nav/far_viz_receiver\.py'
terminate_matching 'python3 .*/nav/initialpose_sender\.py'
terminate_matching '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch .*/nav_dog/go2_nav\.launch\.py'
terminate_matching '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch map_sim map_sim\.launch\.py'
terminate_matching '/autonomy_stack_go2/install/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
terminate_matching '/autonomy_stack_go2/install/terrain_analysis/lib/terrain_analysis/'
terminate_matching '/autonomy_stack_go2/install/terrain_analysis_ext/lib/terrain_analysis_ext/'
terminate_matching '/autonomy_stack_go2/install/local_planner/lib/local_planner/'
terminate_matching '/autonomy_stack_go2/install/far_planner/lib/far_planner/'
terminate_matching '/rviz2/rviz2 .*\.rviz'
terminate_matching '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher'
terminate_matching '^/opt/ros/foxy/lib/tf2_ros/static_transform_publisher'

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

echo '[1/3] 发布固定 PCD 背景'
python3 "${ROOT}/nav/static_map_publisher.py" --map "$MAP" --frame map \
  >"$MAP_LOG" 2>&1 &
PIDS+=("$!")

echo '[2/3] 启动原版 map_sim 仿真闭环'
ros2 launch map_sim map_sim.launch.py \
  pcd:="$MAP" \
  max_speed:="$GO2_MAX_SPEED" \
  rviz:=false \
  robot_state_publisher:=true \
  space_viz:=true \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
PIDS+=("$LAUNCH_PID")

sleep 5
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  tail -n 160 "$LAUNCH_LOG" >&2 || true
  exit 5
fi
if ! pgrep -f '/map_sim/lib/map_sim/space_viz' >/dev/null 2>&1; then
  echo '人工可走区节点 space_viz 启动失败：' >&2
  tail -n 160 "$LAUNCH_LOG" >&2 || true
  exit 6
fi

echo '[3/3] 启动 RViz'
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" >"$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!
PIDS+=("$RVIZ_PID")
sleep 2
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  tail -n 120 "$RVIZ_LOG" >&2 || true
  exit 7
fi

echo "本地模拟已启动"
echo "固定静态地图: $MAP"
echo "启动日志: $LAUNCH_LOG"
echo "静态地图日志: $MAP_LOG"
echo "RViz 日志: $RVIZ_LOG"
echo "人工可走区: enabled"
wait "$LAUNCH_PID" || true
