#!/usr/bin/env bash
# Laptop display-only stack for the full Go2 navigation loop.
#
# Go2:      LIO -> terrain -> FAR -> localPlanner -> pathFollower
# Laptop:   fixed PCD + received state/FAR markers/path in RViz
# Downlink: one final /goal_point on TCP 8899; initial pose on TCP 8900
# Uplink:   state/TF/joints/FAR visualization on TCP 8898; no live cloud
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
# shellcheck source=../nav_dog/go2_mid360_hardware.env
source "${ROOT}/nav_dog/go2_mid360_hardware.env"
HOST_IP="${FAR_VIZ_HOST_IP:-192.168.76.1}"
STATE_PORT="${FAR_VIZ_PORT:-8898}"
COMMAND_PORT="${FAR_COMMAND_PORT:-8899}"
INITIALPOSE_PORT="${GO2_INITIALPOSE_PORT:-8900}"
GOAL_ENABLE="${GO2_GOAL_ENABLE:-${FAR_COMMAND_ENABLE:-1}}"
INITIALPOSE_ENABLE="${GO2_INITIALPOSE_ENABLE:-1}"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
LOG_DIR="${LOG_DIR:-/tmp}"
MAP="${MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
MODEL_PARAMS="${GO2_MODEL_PARAMS:-${ROOT}/nav/go2_model_params.yaml}"
RECEIVER_LOG="${LOG_DIR}/go2_real_state_receiver.log"
GOAL_LOG="${LOG_DIR}/go2_goal_sender.log"
INITIALPOSE_LOG="${LOG_DIR}/go2_initialpose_sender.log"
ROBOT_STATE_LOG="${LOG_DIR}/go2_robot_state.log"
RVIZ_LOG="${LOG_DIR}/go2_real_rviz.log"
RVIZ_CONFIG="${ROOT}/nav/far_real.rviz"
DOG_STATE_LOG="/tmp/far_viz_link.log"
DOG_GOAL_LOG="/tmp/far_command_link.log"
DOG_INITIALPOSE_LOG="/tmp/initialpose_link.log"

for path in "$MAP" "$MODEL_PARAMS" "$RVIZ_CONFIG"; do
  [[ -f "$path" ]] || { echo "找不到文件: $path" >&2; exit 2; }
done

terminate_matching() {
  local pattern="$1" pid
  while IFS= read -r pid; do
    if [[ -n "$pid" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

# Replace only the laptop display/transport stack. Never start a planner here.
terminate_matching 'python3 .*/nav/far_viz_receiver\.py'
terminate_matching 'python3 .*/nav/goal_sender\.py'
terminate_matching 'python3 .*/nav/initialpose_sender\.py'
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

command -v ssh >/dev/null || { echo '缺少 ssh' >&2; exit 3; }
if ! timeout 18 ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" true >/dev/null 2>&1; then
  echo "无法连接狗端 SSH 别名: $HOST" >&2
  exit 3
fi

missing="$(
  ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
    "for p in '[n]av_pose_bridge.py' '[s]tatic_map_replay.py' '[t]errainAnalysis' '[t]errainAnalysisExt' '[f]ar_planner' '[l]ocalPlanner' '[p]athFollower'; do pgrep -f \"\$p\" >/dev/null || echo \"\$p \"; done" \
    2>/dev/null || true
)"
if [[ -n "$missing" ]]; then
  echo "警告：狗端完整导航链未就绪：$missing" >&2
  echo '      请先运行 nav/start_far_real.sh，不要在电脑端补启动规划器。' >&2
fi

echo '[1/6] 同步并启动狗端状态、FAR 可视化和 Goalpoint 桥'
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
  "${ROOT}/nav_dog/go2_mid360_hardware.env" \
  "${ROOT}/nav_dog/initialpose_protocol.py" \
  "${ROOT}/nav_dog/initialpose_link.py" \
  "${ROOT}/nav_dog/run_initialpose_link.sh" \
  "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" \
  "FAR_VIZ_PORT='$STATE_PORT' FAR_COMMAND_PORT='$COMMAND_PORT' \
   GOAL_ENABLE='$GOAL_ENABLE' GO2_INITIALPOSE_PORT='$INITIALPOSE_PORT' \
   GO2_INITIALPOSE_ENABLE='$INITIALPOSE_ENABLE' bash -s" <<'REMOTE'
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
pkill -TERM -f '/home/unitree/go2_nav/far_command_link.py' 2>/dev/null || true
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
nohup env FAR_VIZ_PORT="$FAR_VIZ_PORT" \
  /home/unitree/go2_nav/run_far_viz_link.sh \
  >/tmp/far_viz_link.log 2>&1 </dev/null &
state_pid=$!
sleep 1
kill -0 "$state_pid"
if [[ "$GOAL_ENABLE" != "0" ]]; then
  nohup env FAR_COMMAND_PORT="$FAR_COMMAND_PORT" \
    /home/unitree/go2_nav/run_far_command_link.sh \
    >/tmp/far_command_link.log 2>&1 </dev/null &
  goal_pid=$!
  sleep 2
  kill -0 "$goal_pid"
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

echo '[2/6] 启动电脑端状态/FAR 可视化接收器与固定 PCD'
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

echo '[3/6] 启动电脑端最终 Goalpoint 下行'
if [[ "$GOAL_ENABLE" != "0" ]]; then
  python3 "${ROOT}/nav/goal_sender.py" \
    --host "$HOST_IP" --port "$COMMAND_PORT" \
    >"$GOAL_LOG" 2>&1 &
  PIDS+=("$!")
else
  : >"$GOAL_LOG"
  echo 'Goalpoint 下行已禁用' >&2
fi

echo '[4/6] 启动 2D Pose Estimate 下行'
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

echo '真机显示链已启动：狗端规划/执行，电脑端只显示和下发最终 Goalpoint'
echo "固定静态地图: $MAP"
echo "状态/FAR 接收日志: $RECEIVER_LOG"
echo "Goalpoint 发送日志: $GOAL_LOG"
echo "Initial-pose 日志: $INITIALPOSE_LOG"
echo "RViz 配置: $RVIZ_CONFIG"
echo "RViz 日志: $RVIZ_LOG"
echo "狗端状态/FAR 桥日志: $HOST:$DOG_STATE_LOG"
echo "狗端 Goalpoint 桥日志: $HOST:$DOG_GOAL_LOG"
echo "狗端 initial-pose 日志: $HOST:$DOG_INITIALPOSE_LOG"
echo '狗端实时点云上行: disabled（电脑只显示固定 PCD 和狗端 FAR Marker）'
echo "Goalpoint TCP $COMMAND_PORT: $([[ "$GOAL_ENABLE" == "0" ]] && echo disabled || echo enabled)"
echo "2D Pose Estimate -> TCP $INITIALPOSE_PORT: $([[ "$INITIALPOSE_ENABLE" == "0" ]] && echo disabled || echo enabled)"
echo '电脑端不运行 pcdMapSim / terrainAnalysis / FAR / localPlanner / pathFollower。'
wait "$RVIZ_PID" || true
