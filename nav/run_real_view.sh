#!/usr/bin/env bash
# Go2 state drives the laptop-local static map. The optional minimal obstacle
# cloud is disabled by default so live-scan mismatch cannot alter FAR obstacles.
# Waypoints and RViz initial poses use separate command ports. No raw scans,
# camera, markers, generic commands or velocity streams cross either transport.
set -euo pipefail

ROOT="${HOME}/go2_unitree_slam"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
HOST_IP="${FAR_VIZ_HOST_IP:-192.168.76.1}"
PORT="${FAR_VIZ_PORT:-8898}"
COMMAND_PORT="${FAR_COMMAND_PORT:-8899}"
INITIALPOSE_PORT="${GO2_INITIALPOSE_PORT:-8900}"
CLOUD_ENABLE="${FAR_VIZ_CLOUD_ENABLE:-0}"
WAYPOINT_ENABLE="${GO2_WAYPOINT_ENABLE:-1}"
INITIALPOSE_ENABLE="${GO2_INITIALPOSE_ENABLE:-1}"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
LOG_DIR="${LOG_DIR:-/tmp}"
MAP="${MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
RECEIVER_LOG="${LOG_DIR}/go2_real_state_receiver.log"
LAUNCH_LOG="${LOG_DIR}/go2_local_map_view.log"
WAYPOINT_LOG="${LOG_DIR}/go2_waypoint_sender.log"
OVERLAY_LOG="${LOG_DIR}/go2_pointcloud_overlay.log"
INITIALPOSE_LOG="${LOG_DIR}/go2_initialpose_sender.log"
RVIZ_LOG="${LOG_DIR}/go2_real_rviz.log"
RVIZ_CONFIG="${ROOT}/nav/far_real.rviz"
DOG_LOG="/tmp/far_viz_link.log"
DOG_COMMAND_LOG="/tmp/far_command_link.log"
DOG_INITIALPOSE_LOG="/tmp/initialpose_link.log"

for path in "$MAP"; do
  [[ -f "$path" ]] || { echo "找不到文件: $path" >&2; exit 2; }
done

terminate_matching() {
  local pattern="$1"
  local pid
  while IFS= read -r pid; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

# Stop only the previous laptop-side view stack. Never touch robot services.
terminate_matching 'python3 .*/nav/far_viz_receiver\.py'
terminate_matching 'python3 .*/nav/waypoint_sender\.py'
terminate_matching 'python3 .*/nav/initialpose_sender\.py'
terminate_matching 'python3 .*/nav/pointcloud_overlay\.py'
terminate_matching '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch map_sim map_sim\.launch\.py'
terminate_matching '/home/anyacoder/autonomy_stack_go2/install/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/terrain_analysis/lib/terrain_analysis/'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/terrain_analysis_ext/lib/terrain_analysis_ext/'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/local_planner/lib/local_planner/'
terminate_matching '^/home/anyacoder/autonomy_stack_go2/install/far_planner/lib/far_planner/'
terminate_matching '/rviz2/rviz2 .*\.rviz'
terminate_matching '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher'
terminate_matching '^/opt/ros/foxy/lib/tf2_ros/static_transform_publisher'
terminate_matching '/tf2_ros/static_transform_publisher 0 0 0 0 0 0 world map'
terminate_matching 'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map'
sleep 1

set +u
source /opt/ros/foxy/setup.bash
source "${WS}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
# Foxy + CycloneDDS in this install cannot use ROS_LOCALHOST_ONLY=1; the
# explicit loopback CycloneDDS config below provides the same isolation.
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${ROOT}/nav/cyclonedds_lo.xml"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

command -v ssh >/dev/null || { echo '缺少 ssh' >&2; exit 3; }
ssh_ready=0
for attempt in $(seq 1 20); do
  if timeout 18 ssh -o ConnectTimeout=5 -o ServerAliveInterval=3       -o ServerAliveCountMax=2 -o BatchMode=yes "$HOST" true >/dev/null 2>&1; then
    ssh_ready=1
    break
  fi
  echo "等待狗端 SSH: attempt=$attempt/20" >&2
  sleep 2
done
if [[ "$ssh_ready" -ne 1 ]]; then
  echo "无法连接狗端 SSH 别名: $HOST" >&2
  exit 3
fi

echo '[1/6] 同步并启动狗端状态、waypoint 与 initial-pose 通道'
ssh "$HOST" 'mkdir -p /home/unitree/go2_nav'
scp -q \
  "${ROOT}/nav_dog/far_viz_protocol.py" \
  "${ROOT}/nav_dog/far_viz_link.py" \
  "${ROOT}/nav_dog/run_far_viz_link.sh" \
  "${ROOT}/nav_dog/lowstate/live_robot_state.py" \
  "${ROOT}/nav_dog/lowstate/run_joint_bridge.sh" \
  "${ROOT}/nav_dog/far_command_protocol.py" \
  "${ROOT}/nav_dog/far_command_link.py" \
  "${ROOT}/nav_dog/run_far_command_link.sh" \
  "${ROOT}/nav_dog/initialpose_protocol.py" \
  "${ROOT}/nav_dog/initialpose_link.py" \
  "${ROOT}/nav_dog/run_initialpose_link.sh" \
  "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" \
  "FAR_VIZ_PORT='$PORT' FAR_VIZ_CLOUD_ENABLE='$CLOUD_ENABLE' \
   FAR_COMMAND_PORT='$COMMAND_PORT' FAR_COMMAND_ENABLE='$WAYPOINT_ENABLE' bash -s" <<'REMOTE'
set -e
chmod +x \
  /home/unitree/go2_nav/far_viz_link.py \
  /home/unitree/go2_nav/run_far_viz_link.sh \
  /home/unitree/go2_nav/live_robot_state.py \
  /home/unitree/go2_nav/run_joint_bridge.sh \
  /home/unitree/go2_nav/far_command_link.py \
  /home/unitree/go2_nav/run_far_command_link.sh \
  /home/unitree/go2_nav/initialpose_link.py \
  /home/unitree/go2_nav/run_initialpose_link.sh
pkill -TERM -f '^python3 /home/unitree/go2_nav/live_robot_state.py ros' 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/far_viz_link.py' 2>/dev/null || true
pkill -TERM -x far_viz_link 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/far_command_link.py' 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/initialpose_link.py' 2>/dev/null || true
# The legacy C++ go2_nav_link also owns TCP 8899 and accepts init-pose/stop
# commands. The verified architecture uses the waypoint-only bridge above.
if [[ "$FAR_COMMAND_ENABLE" != "0" ]]; then
  legacy_pid="$(pgrep -f '/go2_nav_link/lib/go2_nav_link/go2_nav_link' | head -n 1 || true)"
  if [[ -n "$legacy_pid" ]]; then
    kill -TERM "$legacy_pid" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "$legacy_pid" 2>/dev/null || break
      sleep 0.2
    done
  fi
fi
sleep 0.5
curl -fsS --max-time 3 http://192.168.123.18:8766/state.json >/dev/null || {
  echo 'Go2 lowstate HTTP source is unavailable: http://192.168.123.18:8766/state.json' >&2
  exit 12
}
nohup /home/unitree/go2_nav/run_joint_bridge.sh \
  >/tmp/go2_joint_bridge.log 2>&1 </dev/null &
joint_pid=$!
sleep 2
if ! kill -0 "$joint_pid" 2>/dev/null; then
  echo 'Go2 joint bridge failed:' >&2
  cat /tmp/go2_joint_bridge.log >&2 || true
  exit 13
fi
echo "joint_pid=$joint_pid"
nohup env FAR_VIZ_PORT="$FAR_VIZ_PORT" FAR_VIZ_CLOUD_ENABLE="$FAR_VIZ_CLOUD_ENABLE" \
  /home/unitree/go2_nav/run_far_viz_link.sh \
  >/tmp/far_viz_link.log 2>&1 </dev/null &
state_pid=$!
sleep 1
kill -0 "$state_pid"
echo "state_pid=$state_pid"
if [[ "$FAR_COMMAND_ENABLE" != "0" ]]; then
  nohup env FAR_COMMAND_ENABLE=1 FAR_COMMAND_PORT="$FAR_COMMAND_PORT" \
    /home/unitree/go2_nav/run_far_command_link.sh \
    >/tmp/far_command_link.log 2>&1 </dev/null &
  command_pid=$!
  sleep 3
  if ! kill -0 "$command_pid" 2>/dev/null; then
    echo "waypoint command bridge failed:" >&2
    cat /tmp/far_command_link.log >&2 || true
    exit 10
  fi
  echo "command_pid=$command_pid"
else
  echo "command_disabled=1"
fi
if [[ "$GO2_INITIALPOSE_ENABLE" != "0" ]]; then
  nohup env GO2_INITIALPOSE_PORT="$GO2_INITIALPOSE_PORT" \
    /home/unitree/go2_nav/run_initialpose_link.sh \
    >/tmp/initialpose_link.log 2>&1 </dev/null &
  initialpose_pid=$!
  sleep 2
  if ! kill -0 "$initialpose_pid" 2>/dev/null; then
    echo "initial-pose bridge failed:" >&2
    cat /tmp/initialpose_link.log >&2 || true
    exit 11
  fi
  echo "initialpose_pid=$initialpose_pid"
else
  echo "initialpose_disabled=1"
fi
REMOTE

PIDS=()
cleanup() {
  local i
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -TERM "${PIDS[$i]}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo '[2/6] 启动电脑端状态接收器'
python3 "${ROOT}/nav/far_viz_receiver.py" \
  --host "$HOST_IP" --port "$PORT" --map "$MAP" --tf-child sensor \
  >"$RECEIVER_LOG" 2>&1 &
PIDS+=("$!")

connected=0
for _ in $(seq 1 80); do
  if grep -q 'connected to Go2 FAR view' "$RECEIVER_LOG" 2>/dev/null; then
    connected=1
    break
  fi
  sleep 0.5
done
if [[ "$connected" -ne 1 ]]; then
  echo '位姿接收器未连上狗端 8898 端口：' >&2
  tail -n 80 "$RECEIVER_LOG" >&2 || true
  exit 4
fi

VIRTUAL_SCAN_TOPIC=/registered_scan
if [[ "$CLOUD_ENABLE" != "0" ]]; then
  echo '[3/6] 启动静态 PCD 虚拟扫描与狗端最小障碍云合并层'
  python3 "${ROOT}/nav/pointcloud_overlay.py" \
    --static-topic /sim/registered_scan \
    --dynamic-topic /go2/min_obstacle_cloud \
    --output-topic /registered_scan \
    --voxel-size 0.0 \
    >"$OVERLAY_LOG" 2>&1 &
  PIDS+=("$!")
  VIRTUAL_SCAN_TOPIC=/sim/registered_scan
  sleep 1
  if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo '在线扫描合并节点启动失败：' >&2
    tail -n 80 "$OVERLAY_LOG" >&2 || true
    exit 5
  fi
else
  echo '[3/6] 最小障碍云已关闭，本地虚拟扫描直接发布到 /registered_scan'
fi

if [[ "$WAYPOINT_ENABLE" != "0" ]]; then
  echo '[4/6] 启动电脑端 waypoint 与 initial-pose 下行发送器'
  python3 "${ROOT}/nav/waypoint_sender.py" \
    --host "$HOST_IP" --port "$COMMAND_PORT" \
    >"$WAYPOINT_LOG" 2>&1 &
  PIDS+=("$!")
  sleep 1
  if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo 'Waypoint 发送器启动失败：' >&2
    tail -n 80 "$WAYPOINT_LOG" >&2 || true
    exit 5
  fi
else
  echo 'Waypoint 下行发送器已由 GO2_WAYPOINT_ENABLE=0 显式关闭'
fi

if [[ "$INITIALPOSE_ENABLE" != "0" ]]; then
  python3 "${ROOT}/nav/initialpose_sender.py" \
    --host "$HOST_IP" --port "$INITIALPOSE_PORT" \
    >"$INITIALPOSE_LOG" 2>&1 &
  PIDS+=("$!")
  sleep 1
  if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo 'Initial-pose 发送器启动失败：' >&2
    tail -n 80 "$INITIALPOSE_LOG" >&2 || true
    exit 5
  fi
else
  echo 'Initial-pose 下行发送器已由 GO2_INITIALPOSE_ENABLE=0 显式关闭'
fi

# Display-only root anchor so /overall_map can render before localization.
/opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 world map \
  >"${LOG_DIR}/go2_map_anchor.log" 2>&1 &
PIDS+=("$!")

echo '[5/6] 启动电脑本地 PCD、FAR、localPlanner 与上游节点'
ros2 launch map_sim map_sim.launch.py \
  external_odom_topic:=/state_estimation \
  virtual_scan_topic:="$VIRTUAL_SCAN_TOPIC" \
  pcd:="$MAP" \
  space_viz:=false \
  rviz:=false \
  robot_state_publisher:=true \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
PIDS+=("$LAUNCH_PID")

sleep 5
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo '本地 map_sim 启动失败：' >&2
  tail -n 120 "$LAUNCH_LOG" >&2 || true
  exit 5
fi

# Use the repository copy, not map_sim's installed stock RViz layout.  This is
# display-only and does not change any planning/control topic or algorithm.
# PointCloud Decay is disabled in the RViz configs so AMDGPU/Ogre does not
# exhaust its GL vertex buffers. Hardware rendering remains the default.
# GO2_RVIZ_SOFTWARE=1 is only a troubleshooting fallback.
if [[ "${GO2_RVIZ_SOFTWARE:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  echo '[6/6] 启动目标布局 RViz（Mesa llvmpipe 临时兼容模式）'
else
  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER 2>/dev/null || true
  echo '[6/6] 启动目标布局 RViz（硬件 OpenGL）'
fi
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" >"$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!
PIDS+=("$RVIZ_PID")
sleep 2
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  echo 'RViz 启动失败：' >&2
  tail -n 120 "$RVIZ_LOG" >&2 || true
  exit 6
fi

echo '真机状态驱动的本地地图与导航视图已启动'
echo "接收日志: $RECEIVER_LOG"
echo "在线扫描合并日志: $OVERLAY_LOG"
echo "本地栈日志: $LAUNCH_LOG"
echo "RViz 日志: $RVIZ_LOG"
echo "RViz 配置: $RVIZ_CONFIG"
echo "Waypoint 日志: $WAYPOINT_LOG"
echo "Initial-pose 日志: $INITIALPOSE_LOG"
echo "狗端状态桥日志: go2-wifi:$DOG_LOG"
echo "狗端命令桥日志: go2-wifi:$DOG_COMMAND_LOG"
echo "狗端 initial-pose 桥日志: go2-wifi:$DOG_INITIALPOSE_LOG"
echo "RViz 中应确认固定静态地图、Go2 模型、FAR 在线红线和真机位姿；人工可走区: disabled；最小障碍云: $([[ "$CLOUD_ENABLE" == "0" ]] && echo disabled || echo enabled)。"
echo "Waypoint: $([[ "$WAYPOINT_ENABLE" == "0" ]] && echo disabled || echo enabled) :$COMMAND_PORT"
echo "2D Pose Estimate -> /initialpose: $([[ "$INITIALPOSE_ENABLE" == "0" ]] && echo disabled || echo enabled) :$INITIALPOSE_PORT"
wait "$LAUNCH_PID" || true
