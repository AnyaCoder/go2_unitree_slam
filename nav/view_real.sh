#!/usr/bin/env bash
# Attach the laptop to the existing real-robot data stream and open RViz.
#
# This script is display-only. It never starts/stops the Go2, planners, SLAM,
# or existing RViz windows. Laptop-side receiver/transport processes are reused
# through pid files, so re-running this script after an RViz crash only opens a
# new RViz window.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_IP="${FAR_VIZ_HOST_IP:-192.168.76.1}"
STATE_PORT="${FAR_VIZ_PORT:-8898}"
COMMAND_PORT="${FAR_COMMAND_PORT:-8899}"
INITIALPOSE_PORT="${GO2_INITIALPOSE_PORT:-8900}"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
MAP="${MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
MODEL_PARAMS="${GO2_MODEL_PARAMS:-${ROOT}/nav/go2_model_params.yaml}"
RVIZ_CONFIG="${ROOT}/nav/far_real.rviz"
RUNTIME_DIR="${GO2_VIEW_RUNTIME_DIR:-/tmp/go2_real_view}"

mkdir -p "$RUNTIME_DIR"

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" >&2
}

for path in "$MAP" "$MODEL_PARAMS" "$RVIZ_CONFIG"; do
  [[ -f "$path" ]] || { echo "找不到文件: $path" >&2; exit 2; }
done

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
if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
fi

if [[ "${GO2_RVIZ_SOFTWARE:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  log 'RViz 渲染：Mesa llvmpipe 软件模式'
else
  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER 2>/dev/null || true
  log 'RViz 渲染：硬件 OpenGL（可设 GO2_RVIZ_SOFTWARE=1 回退）'
fi

pid_path() {
  printf '%s/%s.pid' "$RUNTIME_DIR" "$1"
}

is_running() {
  local pid_file="$1" pid
  [[ -s "$pid_file" ]] || return 1
  pid="$(cat "$pid_file")"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null
}

tcp_open() {
  python3 - "$1" "$2" <<'PY'
import socket
import sys

host, port = sys.argv[1], int(sys.argv[2])
try:
    connection = socket.create_connection((host, port), timeout=0.8)
except OSError:
    sys.exit(1)
connection.close()
PY
}

ensure_proc() {
  local name="$1" pattern="$2" log_file="$3"
  shift 3
  local pid_file pid existing
  pid_file="$(pid_path "$name")"

  if is_running "$pid_file"; then
    log "复用 $name (pid=$(cat "$pid_file"))"
    return 0
  fi
  if [[ -n "$pattern" ]]; then
    existing="$(pgrep -f "$pattern" 2>/dev/null | head -n 1 || true)"
    if [[ -n "$existing" ]]; then
      printf '%s\n' "$existing" >"$pid_file"
      log "接管已有 $name (pid=$existing)"
      return 0
    fi
  fi

  nohup "$@" >>"$log_file" 2>&1 </dev/null &
  pid=$!
  printf '%s\n' "$pid" >"$pid_file"
  sleep 0.5
  if ! kill -0 "$pid" 2>/dev/null; then
    log "$name 启动失败；查看 $log_file"
    tail -n 30 "$log_file" >&2 || true
    return 1
  fi
  log "启动 $name (pid=$pid, log=$log_file)"
}

log "只读查看模式；不会操作狗端、规划器或已有 RViz"
log "ROS_DOMAIN_ID=$DOMAIN_ID  map=$MAP"

log "检查狗端状态通道 $HOST_IP:$STATE_PORT"
state_ready=0
for attempt in $(seq 1 20); do
  if tcp_open "$HOST_IP" "$STATE_PORT"; then
    state_ready=1
    break
  fi
  (( attempt == 10 )) && log '仍在等待狗端 8898……'
  sleep 0.5
done
if [[ "$state_ready" -ne 1 ]]; then
  log "FAIL 狗端导航链未启动：$HOST_IP:$STATE_PORT 无法连接"
  log '请先运行 ./nav/start_far_real.sh；view_real.sh 只负责查看，不会拉起狗端'
  exit 4
fi
log 'PASS 狗端状态通道已就绪'

if ! tcp_open "$HOST_IP" "$COMMAND_PORT"; then
  log "WARN Goalpoint 通道 $HOST_IP:$COMMAND_PORT 未就绪；请检查狗端 far_command_link"
fi
if ! tcp_open "$HOST_IP" "$INITIALPOSE_PORT"; then
  log "WARN InitialPose 通道 $HOST_IP:$INITIALPOSE_PORT 未就绪；请检查狗端 initialpose_link"
fi

ensure_proc \
  far_viz_receiver '^python3 .*/nav/far_viz_receiver\.py' \
  "$RUNTIME_DIR/far_viz_receiver.log" \
  python3 "$ROOT/nav/far_viz_receiver.py" \
  --host "$HOST_IP" --port "$STATE_PORT" --map "$MAP" --tf-child sensor

for attempt in $(seq 1 20); do
  if grep -q 'connected to Go2 state uplink' \
      "$RUNTIME_DIR/far_viz_receiver.log" 2>/dev/null; then
    log "状态数据已接入 $HOST_IP:$STATE_PORT"
    break
  fi
  if (( attempt == 20 )); then
    log "尚未连上狗端 $HOST_IP:$STATE_PORT；继续打开 RViz，日志见 $RUNTIME_DIR/far_viz_receiver.log"
  fi
  sleep 0.5
done

ensure_proc \
  goal_sender '^python3 .*/nav/goal_sender\.py' \
  "$RUNTIME_DIR/goal_sender.log" \
  python3 "$ROOT/nav/goal_sender.py" --host "$HOST_IP" --port "$COMMAND_PORT"

ensure_proc \
  initialpose_sender '^python3 .*/nav/initialpose_sender\.py' \
  "$RUNTIME_DIR/initialpose_sender.log" \
  python3 "$ROOT/nav/initialpose_sender.py" --host "$HOST_IP" --port "$INITIALPOSE_PORT"

ensure_proc map_anchor '' "$RUNTIME_DIR/map_anchor.log" \
  /opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 world map
ensure_proc sensor_base '' "$RUNTIME_DIR/sensor_base.log" \
  /opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 -0.06 0 0 0 sensor base
ensure_proc base_vehicle '' "$RUNTIME_DIR/base_vehicle.log" \
  /opt/ros/foxy/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 base vehicle
ensure_proc robot_state_publisher '' "$RUNTIME_DIR/robot_state_publisher.log" \
  ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$MODEL_PARAMS"

if pgrep -f '^/opt/ros/foxy/lib/rviz2/rviz2 .*far_real\.rviz$' >/dev/null 2>&1; then
  log 'RViz 已经在运行；本脚本不会创建重复窗口'
  exit 0
fi

log '打开 RViz；关闭或崩溃后直接重新运行本脚本'
log "数据进程保持目录: $RUNTIME_DIR"
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"
log 'RViz 已退出；数据查看进程保持运行，重新运行本脚本只会重开 RViz'
