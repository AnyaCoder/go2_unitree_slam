#!/usr/bin/env bash
# One-command real-robot startup:
#   1. stop this project's previous SLAM/navigation stack
#   2. keep UnitreeSlam's legacy motion nodes disabled
#   3. start Go2 UnitreeSlam + terrain/localPlanner/pathFollower (no Go2 FAR)
#   4. start laptop fixed-map FAR, path sender, state display and RViz
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
# shellcheck source=../nav_dog/go2_mid360_hardware.env
source "${ROOT}/nav_dog/go2_mid360_hardware.env"

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

echo '[1/6] 清理旧 SLAM、导航、FAR、RViz 与桥接进程'
"${ROOT}/nav/stop_real_all.sh"

echo
echo '[2/6] 检查狗端 SSH'
for ((i = 1; i <= 20; i++)); do
  if ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" true >/dev/null 2>&1; then
    echo "PASS  ssh $HOST"
    break
  fi
  [[ "$i" -eq 20 ]] && { echo "FAIL  无法连接 $HOST" >&2; exit 2; }
  sleep 2
done

echo
echo '[3/6] 阻止 UnitreeSlam 旧运动控制链（保留 LIO/重定位）'
"${ROOT}/nav_dog/disable_legacy_control.sh"

echo
echo '[4/6] 狗端启动 UnitreeSlam / Livox / LIO 重定位服务'
ssh "$HOST" \
  "cd /unitree/module/graph_pid_ws && setsid -f ./0_unitree_slam.sh >/tmp/unitree_slam_restart.log 2>&1 < /dev/null"
if ! wait_remote '[Q]T_Server/lib/QT_Server/UnitreeSlam' 'UnitreeSlam'; then
  ssh "$HOST" "tail -n 120 /tmp/unitree_slam_restart.log" >&2 || true
  exit 3
fi

echo
echo '[5/6] 狗端启动 terrain / localPlanner / pathFollower（不启动 FAR）'
ssh "$HOST" 'mkdir -p /home/unitree/go2_nav'
scp -q \
  "${ROOT}/nav_dog/go2_nav.launch.py" \
  "${ROOT}/nav_dog/run_nav.sh" \
  "${ROOT}/nav_dog/self_filter.py" \
  "${ROOT}/nav_dog/go2_mid360_hardware.env" \
  "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" \
  'chmod +x /home/unitree/go2_nav/run_nav.sh'
ssh "$HOST" \
  "cd /home/unitree/go2_nav && setsid -f env SENSOR_OFFSET_X='$SENSOR_OFFSET_X' SENSOR_OFFSET_Y='$SENSOR_OFFSET_Y' GO2_MAX_SPEED='$GO2_MAX_SPEED' ./run_nav.sh live >/tmp/go2_nav_live_restart.log 2>&1 < /dev/null"
if ! wait_remote '[l]ocalPlanner' 'localPlanner' 30; then
  ssh "$HOST" "tail -n 160 /tmp/go2_nav_live_restart.log" >&2 || true
  exit 4
fi
if ! wait_remote '[p]athFollower' 'pathFollower' 30; then
  ssh "$HOST" "tail -n 160 /tmp/go2_nav_live_restart.log" >&2 || true
  exit 5
fi
if ssh "$HOST" "pgrep -f '[f]ar_planner' >/dev/null 2>&1"; then
  echo 'FAIL  狗端意外启动了 far_planner；最终架构禁止狗端 FAR' >&2
  exit 5
fi
echo 'PASS  狗端没有 far_planner'

if [[ -n "$(ssh "$HOST" "ps -eo args | grep -E '[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd' || true")" ]]; then
  echo 'FAIL  检测到 UnitreeSlam 旧运动控制节点，拒绝继续' >&2
  exit 6
fi
echo 'PASS  没有 dog_control_B1_one / go2_control_by_sdk send_cmd'

echo
echo '[6/6] 启动电脑端固定地图全局规划、路径下发、状态显示与 RViz'
echo '日志：'
echo '  狗端 UnitreeSlam: /tmp/unitree_slam_restart.log'
echo '  狗端 go2_nav:     /tmp/go2_nav_live_restart.log'
echo '  电脑端 RViz:      /tmp/go2_real_rviz.log'
echo
echo 'RViz 打开后：点一次 2D Pose Estimate；重定位成功后用 Goalpoint 发起电脑端全局规划。'
exec "${ROOT}/nav/run_real_view.sh"
