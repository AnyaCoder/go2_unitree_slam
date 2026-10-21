#!/usr/bin/env bash
# One-command laptop side of the final real-robot architecture.
#
# Laptop: fixed PCD -> pcdMapSim -> terrain -> FAR -> ordered global path
# Go2:    Livox/LIO -> terrain -> localPlanner -> pathFollower
# Link:   TCP 8898 state uplink, TCP 8899 path downlink, TCP 8900 initial pose
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
# shellcheck source=../nav_dog/go2_mid360_hardware.env
source "${ROOT}/nav_dog/go2_mid360_hardware.env"
HOST_IP="${FAR_VIZ_HOST_IP:-192.168.76.1}"
STATE_PORT="${FAR_VIZ_PORT:-8898}"
COMMAND_PORT="${FAR_COMMAND_PORT:-8899}"
INITIALPOSE_PORT="${GO2_INITIALPOSE_PORT:-8900}"
CLOUD_ENABLE="${FAR_VIZ_CLOUD_ENABLE:-1}"
PATH_ENABLE="${GO2_WAYPOINT_ENABLE:-1}"
INITIALPOSE_ENABLE="${GO2_INITIALPOSE_ENABLE:-1}"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
LOG_DIR="${LOG_DIR:-/tmp}"
MAP="${MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
MODEL_PARAMS="${GO2_MODEL_PARAMS:-${ROOT}/nav/go2_model_params.yaml}"
RECEIVER_LOG="${LOG_DIR}/go2_real_state_receiver.log"
PLANNER_LOG="${LOG_DIR}/go2_laptop_global_planner.log"
PATH_LOG="${LOG_DIR}/go2_global_path_sender.log"
INITIALPOSE_LOG="${LOG_DIR}/go2_initialpose_sender.log"
ROBOT_STATE_LOG="${LOG_DIR}/go2_robot_state.log"
RVIZ_LOG="${LOG_DIR}/go2_real_rviz.log"
RVIZ_CONFIG="${ROOT}/nav/far_real.rviz"
DOG_STATE_LOG="/tmp/far_viz_link.log"
DOG_PATH_LOG="/tmp/global_path_receiver.log"
DOG_INITIALPOSE_LOG="/tmp/initialpose_link.log"

for path in "$MAP" "$MODEL_PARAMS" "$RVIZ_CONFIG"; do
  [[ -f "$path" ]] || { echo "找不到文件: $path" >&2; exit 2; }
done

terminate_matching() {
  local pattern="$1" pid
  while IFS= read -r pid; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

# Stop only this project's previous laptop view/planner stack.
terminate_matching 'python3 .*/nav/far_viz_receiver\.py'
terminate_matching 'python3 .*/nav/global_path_sender\.py'
terminate_matching 'python3 .*/nav/waypoint_sender\.py'
terminate_matching 'python3 .*/nav/initialpose_sender\.py'
terminate_matching '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch .*/nav/global_planner\.launch\.py'
terminate_matching '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch map_sim map_sim\.launch\.py'
terminate_matching '/autonomy_stack_go2/install/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
terminate_matching '^/autonomy_stack_go2/install/terrain_analysis/lib/terrain_analysis/'
terminate_matching '^/autonomy_stack_go2/install/terrain_analysis_ext/lib/terrain_analysis_ext/'
terminate_matching '^/autonomy_stack_go2/install/local_planner/lib/local_planner/'
terminate_matching '^/autonomy_stack_go2/install/far_planner/lib/far_planner/'
terminate_matching '/rviz2/rviz2 .*\.rviz'
terminate_matching '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher'
terminate_matching '^/opt/ros/foxy/lib/tf2_ros/static_transform_publisher'
terminate_matching 'ros2 run robot_state_publisher robot_state_publisher'
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

command -v ssh >/dev/null || { echo '缺少 ssh' >&2; exit 3; }
ssh_ready=0
for attempt in $(seq 1 20); do
  if timeout 18 ssh -o ConnectTimeout=5 -o ServerAliveInterval=3 \
      -o ServerAliveCountMax=2 -o BatchMode=yes "$HOST" true >/dev/null 2>&1; then
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

dog_nav_missing="$(
  ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
    "for p in '[l]ocalPlanner' '[p]athFollower'; do pgrep -f \"\$p\" >/dev/null || echo \"\$p\"; done" \
    2>/dev/null || true
)"
if [[ -n "$dog_nav_missing" ]]; then
  echo "警告：狗端执行进程未就绪：$dog_nav_missing" >&2
  echo '      请先运行 nav/start_far_real.sh，不要在显示脚本里擅自启动狗端规划。' >&2
fi
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
    "pgrep -f '[f]ar_planner' >/dev/null 2>&1"; then
  echo 'FAIL  狗端仍有 far_planner；最终架构禁止狗端 FAR' >&2
  exit 4
fi

echo '[1/6] 同步并启动狗端状态上行、全局路径接收和初始位姿通道'
ssh "$HOST" 'mkdir -p /home/unitree/go2_nav'
scp -q \
  "${ROOT}/nav_dog/far_viz_protocol.py" \
  "${ROOT}/nav_dog/far_viz_link.py" \
  "${ROOT}/nav_dog/run_far_viz_link.sh" \
  "${ROOT}/nav_dog/lowstate/live_robot_state.py" \
  "${ROOT}/nav_dog/lowstate/run_joint_bridge.sh" \
  "${ROOT}/nav_dog/nav_path_protocol.py" \
  "${ROOT}/nav_dog/global_path_receiver.py" \
  "${ROOT}/nav_dog/run_global_path_receiver.sh" \
  "${ROOT}/nav_dog/go2_mid360_hardware.env" \
  "${ROOT}/nav_dog/initialpose_protocol.py" \
  "${ROOT}/nav_dog/initialpose_link.py" \
  "${ROOT}/nav_dog/run_initialpose_link.sh" \
  "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" \
  "FAR_VIZ_PORT='$STATE_PORT' FAR_VIZ_CLOUD_ENABLE='$CLOUD_ENABLE' \
   FAR_COMMAND_PORT='$COMMAND_PORT' GO2_WAYPOINT_ENABLE='$PATH_ENABLE' \
   FAR_SENSOR_OFFSET_X='$SENSOR_OFFSET_X' \
   FAR_SENSOR_OFFSET_Y='$SENSOR_OFFSET_Y' \
   GO2_INITIALPOSE_PORT='$INITIALPOSE_PORT' \
   GO2_INITIALPOSE_ENABLE='$INITIALPOSE_ENABLE' bash -s" <<'REMOTE'
set -e
chmod +x \
  /home/unitree/go2_nav/far_viz_link.py \
  /home/unitree/go2_nav/run_far_viz_link.sh \
  /home/unitree/go2_nav/live_robot_state.py \
  /home/unitree/go2_nav/run_joint_bridge.sh \
  /home/unitree/go2_nav/global_path_receiver.py \
  /home/unitree/go2_nav/run_global_path_receiver.sh \
  /home/unitree/go2_nav/initialpose_link.py \
  /home/unitree/go2_nav/run_initialpose_link.sh
pkill -TERM -f '^python3 /home/unitree/go2_nav/live_robot_state.py ros' 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/far_viz_link.py' 2>/dev/null || true
pkill -TERM -x far_viz_link 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/far_command_link.py' 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/global_path_receiver.py' 2>/dev/null || true
pkill -TERM -f '/home/unitree/go2_nav/initialpose_link.py' 2>/dev/null || true
sleep 0.5
curl -fsS --max-time 3 http://192.168.123.18:8766/state.json >/dev/null || {
  echo 'Go2 lowstate HTTP source is unavailable: http://192.168.123.18:8766/state.json' >&2
  exit 12
}
nohup /home/unitree/go2_nav/run_joint_bridge.sh \
  >/tmp/go2_joint_bridge.log 2>&1 </dev/null &
joint_pid=$!
sleep 2
kill -0 "$joint_pid"
nohup env FAR_VIZ_PORT="$FAR_VIZ_PORT" FAR_VIZ_CLOUD_ENABLE="$FAR_VIZ_CLOUD_ENABLE" \
  /home/unitree/go2_nav/run_far_viz_link.sh \
  >/tmp/far_viz_link.log 2>&1 </dev/null &
state_pid=$!
sleep 1
kill -0 "$state_pid"
if [[ "$GO2_WAYPOINT_ENABLE" != "0" ]]; then
  nohup env FAR_COMMAND_PORT="$FAR_COMMAND_PORT" \
    FAR_SENSOR_OFFSET_X="$FAR_SENSOR_OFFSET_X" \
    FAR_SENSOR_OFFSET_Y="$FAR_SENSOR_OFFSET_Y" \
    /home/unitree/go2_nav/run_global_path_receiver.sh \
    >/tmp/global_path_receiver.log 2>&1 </dev/null &
  path_pid=$!
  sleep 3
  kill -0 "$path_pid"
fi
if [[ "$GO2_INITIALPOSE_ENABLE" != "0" ]]; then
  nohup env GO2_INITIALPOSE_PORT="$GO2_INITIALPOSE_PORT" \
    /home/unitree/go2_nav/run_initialpose_link.sh \
    >/tmp/initialpose_link.log 2>&1 </dev/null &
  initialpose_pid=$!
  sleep 2
  kill -0 "$initialpose_pid"
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

echo '[2/6] 启动电脑端状态接收器与固定 PCD 背景'
python3 "${ROOT}/nav/far_viz_receiver.py" \
  --host "$HOST_IP" --port "$STATE_PORT" --map "$MAP" --tf-child sensor \
  >"$RECEIVER_LOG" 2>&1 &
PIDS+=("$!")
connected=0
for _ in $(seq 1 80); do
  if grep -q 'connected to Go2 state uplink' "$RECEIVER_LOG" 2>/dev/null; then
    connected=1
    break
  fi
  sleep 0.5
done
if [[ "$connected" -ne 1 ]]; then
  echo '状态接收器未连上狗端 8898 端口：' >&2
  tail -n 80 "$RECEIVER_LOG" >&2 || true
  exit 5
fi

echo '[3/6] 启动电脑端固定地图 pcdMapSim / terrain / FAR（无 localPlanner/pathFollower）'
ros2 launch "${ROOT}/nav/global_planner.launch.py" \
  pcd:="$MAP" \
  external_odom_topic:=/state_estimation \
  max_speed:="$GO2_MAX_SPEED" \
  robot_state_publisher:=false \
  space_viz:=false \
  rviz:=false \
  >"$PLANNER_LOG" 2>&1 &
PLANNER_PID=$!
PIDS+=("$PLANNER_PID")
planner_ready=0
for _ in $(seq 1 40); do
  if pgrep -f '/map_sim/lib/map_sim/pcd_map_sim|/map_sim/pcd_map_sim' >/dev/null 2>&1 &&
     pgrep -f '/far_planner/lib/far_planner/far_planner' >/dev/null 2>&1; then
    planner_ready=1
    break
  fi
  if ! kill -0 "$PLANNER_PID" 2>/dev/null; then
    break
  fi
  sleep 0.5
done
if [[ "$planner_ready" -ne 1 ]]; then
  echo '电脑端固定地图规划栈启动失败：' >&2
  tail -n 160 "$PLANNER_LOG" >&2 || true
  exit 6
fi
if pgrep -f '/local_planner/lib/local_planner/(localPlanner|pathFollower)' >/dev/null 2>&1; then
  echo 'FAIL  电脑端意外启动了 localPlanner/pathFollower' >&2
  exit 7
fi

echo '[4/6] 启动电脑 FAR 全局路径 -> 狗端 TCP 下发'
if [[ "$PATH_ENABLE" != "0" ]]; then
  python3 "${ROOT}/nav/global_path_sender.py" \
    --host "$HOST_IP" --port "$COMMAND_PORT" \
    >"$PATH_LOG" 2>&1 &
  PIDS+=("$!")
else
  : >"$PATH_LOG"
  echo '全局路径下发已禁用' >&2
fi
if [[ "$INITIALPOSE_ENABLE" != "0" ]]; then
  python3 "${ROOT}/nav/initialpose_sender.py" \
    --host "$HOST_IP" --port "$INITIALPOSE_PORT" \
    >"$INITIALPOSE_LOG" 2>&1 &
  PIDS+=("$!")
fi

echo '[5/6] 启动 Go2 模型、TF 与固定地图显示节点'
/opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 world map \
  >"${LOG_DIR}/go2_map_anchor.log" 2>&1 &
PIDS+=("$!")
/opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 -0.06 0 0 0 sensor base \
  >"${LOG_DIR}/go2_sensor_base_tf.log" 2>&1 &
PIDS+=("$!")
/opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 base vehicle \
  >"${LOG_DIR}/go2_base_vehicle_tf.log" 2>&1 &
PIDS+=("$!")
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "$MODEL_PARAMS" \
  >"$ROBOT_STATE_LOG" 2>&1 &
ROBOT_STATE_PID=$!
PIDS+=("$ROBOT_STATE_PID")
sleep 2
if ! kill -0 "$ROBOT_STATE_PID" 2>/dev/null; then
  echo 'Go2 robot_state_publisher 启动失败：' >&2
  tail -n 80 "$ROBOT_STATE_LOG" >&2 || true
  exit 8
fi

echo '[6/6] 启动 RViz'
if [[ "${GO2_RVIZ_SOFTWARE:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  echo 'RViz 渲染：Mesa llvmpipe（临时兼容模式）'
else
  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER 2>/dev/null || true
  echo 'RViz 渲染：硬件 OpenGL'
fi
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" >"$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!
PIDS+=("$RVIZ_PID")
sleep 2
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  echo 'RViz 启动失败：' >&2
  tail -n 120 "$RVIZ_LOG" >&2 || true
  exit 9
fi

echo '最终真机链路已启动：电脑固定地图全局规划，狗端局部避障与执行'
echo "固定静态地图: $MAP"
echo "状态接收日志: $RECEIVER_LOG"
echo "电脑规划日志: $PLANNER_LOG"
echo "全局路径下发日志: $PATH_LOG"
echo "Initial-pose 日志: $INITIALPOSE_LOG"
echo "RViz 配置: $RVIZ_CONFIG"
echo "RViz 日志: $RVIZ_LOG"
echo "狗端状态桥日志: $HOST:$DOG_STATE_LOG"
echo "狗端全局路径接收日志: $HOST:$DOG_PATH_LOG"
echo "狗端 initial-pose 日志: $HOST:$DOG_INITIALPOSE_LOG"
echo "狗端点云上行: $([[ "$CLOUD_ENABLE" == "0" ]] && echo disabled || echo enabled)"
echo "全局路径 TCP $COMMAND_PORT: $([[ "$PATH_ENABLE" == "0" ]] && echo disabled || echo enabled)"
echo "2D Pose Estimate -> TCP $INITIALPOSE_PORT: $([[ "$INITIALPOSE_ENABLE" == "0" ]] && echo disabled || echo enabled)"
echo '电脑端只运行 pcdMapSim/terrain/FAR；localPlanner/pathFollower 只在狗端。'
wait "$RVIZ_PID" || true
