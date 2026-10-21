#!/usr/bin/env bash
# One-command FAR real-robot startup:
#   1. stop this project's previous SLAM/navigation stack
#   2. keep UnitreeSlam's legacy motion nodes disabled
#   3. deploy the current localPlanner/safety-gate files to the Go2
#   4. start Go2 UnitreeSlam + go2_nav
#   5. start laptop static-map/FAR/view stack
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
SENSOR_OFFSET_X="${SENSOR_OFFSET_X:-0.3}"
SENSOR_OFFSET_Y="${SENSOR_OFFSET_Y:-0.0}"

remote_has() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
    "pgrep -f '$1' >/dev/null 2>&1"
}

wait_remote() {
  local pattern="$1" label="$2" attempts="${3:-40}" i
  for ((i = 1; i <= attempts; i++)); do
    if remote_has "$pattern"; then
      echo "PASS  $label"
      return 0
    fi
    sleep 1
  done
  echo "FAIL  等待 $label 超时" >&2
  return 1
}

echo '[1/7] 清理旧 SLAM、导航、FAR、RViz 与桥接进程'
"${ROOT}/nav/stop_real_all.sh"

echo
echo '[2/7] 检查狗端 SSH'
for ((i = 1; i <= 20; i++)); do
  if ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" true >/dev/null 2>&1; then
    echo "PASS  ssh $HOST"
    break
  fi
  [[ "$i" -eq 20 ]] && { echo "FAIL  无法连接 $HOST" >&2; exit 2; }
  sleep 2
done

echo
echo '[3/7] 阻止 UnitreeSlam 旧运动控制链（保留 LIO/重定位）'
"${ROOT}/nav_dog/disable_legacy_control.sh"

echo
echo '[4/7] 部署狗端 localPlanner 与安全闸文件'
ssh "$HOST" 'mkdir -p /home/unitree/go2_nav'
scp -q \
  "${ROOT}/nav_dog/go2_nav.launch.py" \
  "${ROOT}/nav_dog/self_filter.py" \
  "${ROOT}/nav_dog/safety_state.py" \
  "${ROOT}/nav_dog/nav_safety_gate.py" \
  "${ROOT}/nav_dog/run_nav.sh" \
  "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" \
  "chmod +x /home/unitree/go2_nav/run_nav.sh /home/unitree/go2_nav/nav_safety_gate.py && \
   python3 -m py_compile /home/unitree/go2_nav/safety_state.py \
     /home/unitree/go2_nav/nav_safety_gate.py /home/unitree/go2_nav/go2_nav.launch.py"
echo 'PASS  狗端导航文件已同步并通过 Python 语法检查'

echo
echo '[5/7] 狗端启动 UnitreeSlam / Livox / LIO 重定位服务'
ssh "$HOST" \
  "cd /unitree/module/graph_pid_ws && setsid -f ./0_unitree_slam.sh >/tmp/unitree_slam_restart.log 2>&1 < /dev/null"
if ! wait_remote '[Q]T_Server/lib/QT_Server/UnitreeSlam' 'UnitreeSlam'; then
  ssh "$HOST" "tail -n 120 /tmp/unitree_slam_restart.log" >&2 || true
  exit 3
fi

echo
echo '[6/7] 狗端启动 localPlanner / pathFollower / 安全闸'
ssh "$HOST" \
  "cd /home/unitree/go2_nav && SENSOR_OFFSET_X='$SENSOR_OFFSET_X' SENSOR_OFFSET_Y='$SENSOR_OFFSET_Y' \
   setsid -f ./run_nav.sh live >/tmp/go2_nav_live_restart.log 2>&1 < /dev/null"
if ! wait_remote '[l]ocalPlanner' 'localPlanner' 30; then
  ssh "$HOST" "tail -n 160 /tmp/go2_nav_live_restart.log" >&2 || true
  exit 4
fi
if ! wait_remote '[p]athFollower' 'pathFollower' 30; then
  ssh "$HOST" "tail -n 160 /tmp/go2_nav_live_restart.log" >&2 || true
  exit 5
fi
if ! wait_remote '[n]av_safety_gate.py' 'nav_safety_gate' 30; then
  ssh "$HOST" "tail -n 160 /tmp/go2_nav_live_restart.log" >&2 || true
  exit 6
fi

if [[ -n "$(ssh "$HOST" "ps -eo args | grep -E '[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd' || true")" ]]; then
  echo 'FAIL  检测到 UnitreeSlam 旧运动控制节点，拒绝继续' >&2
  exit 7
fi
echo 'PASS  没有 dog_control_B1_one / go2_control_by_sdk send_cmd'

echo
echo '[7/7] 启动电脑端固定地图、FAR、状态桥与 RViz'
echo '日志：'
echo '  狗端 UnitreeSlam: /tmp/unitree_slam_restart.log'
echo '  狗端 go2_nav:     /tmp/go2_nav_live_restart.log'
echo '  电脑端总日志:     /tmp/go2_local_map_view.log'
echo '  电脑端 RViz:      /tmp/go2_real_rviz.log'
echo
echo '安全启动顺序（必须遵守）：'
echo '  1. 狗站稳后保持完全静止约 3 秒，等待 IMU 校准通过。'
echo '  2. 在 RViz 点一次 2D Pose Estimate。'
echo '  3. 等待狗端出现 Pose initialization successful；此前面向 Goalpoint 的请求会被丢弃。'
echo '  4. 再点 FAR Goalpoint。没有新一轮 goal/waypoint/path 时，运动闸保持关闭。'
echo
exec "${ROOT}/nav/run_real_view.sh"
