#!/usr/bin/env bash
# One-command real-robot startup for the full dog-side navigation loop.
#   1. stop this project's old processes
#   2. keep UnitreeSlam legacy motion controllers disabled
#   3. start UnitreeSlam/Livox/LIO on the Go2
#   4. deploy the fixed PCD replay -> terrain -> FAR -> planners on Go2
#   5. start the laptop display-only view
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
STATIC_MAP="${GO2_STATIC_MAP:-${ROOT}/GlobalMap_clean_20261020_1920_nav_green_edited.pcd}"
REMOTE_MAP="/home/unitree/go2_nav/maps/$(basename "$STATIC_MAP")"
FORCE_STATIC_MAP_SYNC="${GO2_FORCE_STATIC_MAP_SYNC:-0}"
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

echo '[1/8] 清理旧 SLAM、导航、FAR、RViz 与桥接进程'
"${ROOT}/nav/stop_real_all.sh"

echo
echo '[2/8] 检查狗端 SSH'
for ((i = 1; i <= 20; i++)); do
  if ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" true >/dev/null 2>&1; then
    echo "PASS  ssh $HOST"
    break
  fi
  [[ "$i" -eq 20 ]] && { echo "FAIL  无法连接 $HOST" >&2; exit 2; }
  sleep 2
done

echo
echo '[3/8] 阻止 UnitreeSlam 旧运动控制链（保留 LIO/重定位）'
"${ROOT}/nav_dog/disable_legacy_control.sh"

echo
echo '[4/8] 狗端启动 UnitreeSlam / Livox / LIO 重定位服务'
ssh "$HOST" \
  "cd /unitree/module/graph_pid_ws && setsid -f ./0_unitree_slam.sh >/tmp/unitree_slam_restart.log 2>&1 < /dev/null"
if ! wait_remote '[Q]T_Server/lib/QT_Server/UnitreeSlam' 'UnitreeSlam' 45; then
  ssh "$HOST" "tail -n 160 /tmp/unitree_slam_restart.log" >&2 || true
  exit 3
fi

echo
echo '[5/8] 检查狗端固定导航 PCD（仅缺失或大小变化时传输）'
[[ -f "$STATIC_MAP" ]] || {
  echo "FAIL  找不到固定导航 PCD: $STATIC_MAP" >&2
  exit 5
}
ssh "$HOST" 'mkdir -p /home/unitree/go2_nav/maps'
local_size="$(stat -c %s "$STATIC_MAP")"
remote_size="$(ssh "${HOST}" "stat -c %s '${REMOTE_MAP}' 2>/dev/null || true")"
if [[ "$FORCE_STATIC_MAP_SYNC" == "1" || "$remote_size" != "$local_size" ]]; then
  echo "传输固定导航 PCD: ${local_size} bytes"
  scp -q "$STATIC_MAP" "$HOST:$REMOTE_MAP"
else
  echo "复用狗端固定导航 PCD: ${REMOTE_MAP} (${local_size} bytes)"
fi
scp -q "$ROOT/nav_dog/static_map_replay.py" "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" "test -s '$REMOTE_MAP' && test -r /home/unitree/go2_nav/static_map_replay.py"

echo
echo '[6/8] 狗端启动完整 FAR / localPlanner / pathFollower 链'
scp -q \
  "${ROOT}/nav_dog/go2_nav.launch.py" \
  "${ROOT}/nav_dog/run_nav.sh" \
  "${ROOT}/nav_dog/nav_pose_math.py" \
  "${ROOT}/nav_dog/nav_pose_bridge.py" \
  "${ROOT}/nav_dog/nav_rate_audit.py" \
  "${ROOT}/nav_dog/go2_mid360_hardware.env" \
  "$HOST:/home/unitree/go2_nav/"
ssh "$HOST" 'chmod +x /home/unitree/go2_nav/run_nav.sh'
ssh "$HOST" \
  "cd /home/unitree/go2_nav && setsid -f env SENSOR_OFFSET_X='$SENSOR_OFFSET_X' SENSOR_OFFSET_Y='$SENSOR_OFFSET_Y' GO2_MAX_SPEED='$GO2_MAX_SPEED' GO2_STATIC_MAP_PATH='$REMOTE_MAP' ./run_nav.sh live >/tmp/go2_nav_live_restart.log 2>&1 < /dev/null"

for entry in \
  '[n]av_pose_bridge.py|nav_pose_bridge' \
  '[s]tatic_map_replay.py|static_map_replay' \
  '[t]errainAnalysis|terrainAnalysis' \
  '[t]errainAnalysisExt|terrainAnalysisExt' \
  '[f]ar_planner|far_planner' \
  '[l]ocalPlanner|localPlanner' \
  '[p]athFollower|pathFollower'; do
  pattern="${entry%%|*}"
  label="${entry##*|}"
  if ! wait_remote "$pattern" "$label" 35; then
    ssh "$HOST" "tail -n 220 /tmp/go2_nav_live_restart.log" >&2 || true
    exit 4
  fi
done

stopped="$(ssh "$HOST" 'bash -s' <<'REMOTE'
for p in '[n]av_pose_bridge.py' '[s]tatic_map_replay.py' \
         '[t]errainAnalysis' '[t]errainAnalysisExt' \
         '[f]ar_planner' '[l]ocalPlanner' '[p]athFollower'; do
  pid="$(pgrep -f "$p" | head -n 1 || true)"
  [[ -n "$pid" ]] || continue
  state="$(ps -o stat= -p "$pid" | tr -d ' ')"
  [[ "$state" == T* ]] && echo "$p pid=$pid state=$state"
done
REMOTE
)"
if [[ -n "$stopped" ]]; then
  echo "FAIL  狗端关键进程被挂起，拒绝继续：" >&2
  echo "$stopped" >&2
  echo '       先检查 SIGSTOP/SIGTSTP 来源，不要直接运行 RViz。' >&2
  exit 4
fi
echo 'PASS  狗端关键进程均未被暂停'

if ssh "$HOST" "pgrep -f '[g]lobal_path_receiver.py' >/dev/null 2>&1"; then
  echo 'FAIL  狗端仍运行旧 global_path_receiver；会形成第二套路径所有权' >&2
  exit 5
fi
echo 'PASS  没有旧 global_path_receiver / 第二套全局路径'

echo
echo '[7/8] 检查运动控制权唯一性'
if [[ -n "$(ssh "$HOST" "ps -eo args | grep -E '[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd' || true")" ]]; then
  echo 'FAIL  检测到 UnitreeSlam 旧运动控制节点，拒绝继续' >&2
  exit 6
fi
echo 'PASS  没有 dog_control_B1_one / go2_control_by_sdk send_cmd'
echo 'PASS  自主运动链只有 far_planner -> localPlanner -> pathFollower'

echo
echo '[8/8] 启动电脑端只读显示、Goalpoint 下行与 RViz'
echo '日志：'
echo '  狗端 UnitreeSlam: /tmp/unitree_slam_restart.log'
echo '  狗端 go2_nav:     /tmp/go2_nav_live_restart.log'
echo '  电脑端数据查看:   /tmp/go2_real_view/'
echo
echo 'RViz 打开后：先用 2D Pose Estimate 完成重定位，再点 Goalpoint。'
echo '启动完成但未点 Goalpoint 前，狗端 localPlanner 的 goal 为 NaN，不会自主出发。'
exec "${ROOT}/nav/view_real.sh"
