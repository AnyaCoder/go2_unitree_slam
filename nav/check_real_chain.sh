#!/usr/bin/env bash
# Read-only acceptance check for the final split-planning architecture.
# It never starts, stops, publishes, or modifies anything.
set -uo pipefail

ROOT="${HOME}/go2_unitree_slam"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
DOG_HOST="${FAR_VIZ_HOST:-go2-wifi}"
PATH_ENABLE="${GO2_WAYPOINT_ENABLE:-1}"
INITIALPOSE_ENABLE="${GO2_INITIALPOSE_ENABLE:-1}"
failures=0
warnings=0

pass() { printf 'PASS  %s\n' "$*"; }
warn() { printf 'WARN  %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf 'FAIL  %s\n' "$*"; failures=$((failures + 1)); }

set +u
source /opt/ros/foxy/setup.bash
source "${WS}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${ROOT}/nav/cyclonedds_lo.xml"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

publisher_count() {
  timeout 5 ros2 topic info "$1" 2>/dev/null |
    awk '/Publisher count:/ {print $3; found=1} END {if (!found) print -1}'
}

require_publisher() {
  local topic="$1" count
  count="$(publisher_count "$topic")"
  if [[ "$count" =~ ^[0-9]+$ ]] && (( count >= 1 )); then
    pass "$topic publisher=$count"
  else
    fail "$topic 没有发布者 (count=$count)"
  fi
}

require_single_publisher() {
  local topic="$1" count
  count="$(publisher_count "$topic")"
  if [[ "$count" == "1" ]]; then
    pass "$topic 唯一发布者"
  else
    fail "$topic 发布者应为 1，实际=$count"
  fi
}

require_no_publisher() {
  local topic="$1" count
  count="$(publisher_count "$topic")"
  if [[ "$count" == "0" || "$count" == "-1" ]]; then
    pass "$topic 在电脑端没有发布者"
  else
    fail "$topic 在电脑端不应发布，实际发布者=$count"
  fi
}

node_exists() {
  timeout 5 ros2 node list 2>/dev/null | grep -Fxq "$1"
}

remote_process_snapshot() {
  timeout 12 ssh -o ConnectTimeout=5 -o BatchMode=yes "$DOG_HOST" \
    'ps -eo pid,args' 2>/dev/null || true
}

remote_publisher_count() {
  timeout 18 ssh -o ConnectTimeout=5 -o BatchMode=yes "$DOG_HOST" \
    "bash -lc 'set +u; source /opt/ros/foxy/setup.bash; \
      source /unitree/module/graph_pid_ws/install/local_setup.bash; \
      source /home/unitree/autonomy_stack_go2/install/setup.bash; \
      export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH; \
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
      export ROS_LOCALHOST_ONLY=0; \
      timeout 6 ros2 topic info $1'" 2>/dev/null |
    awk '/Publisher count:/ {print $3; found=1} END {if (!found) print -1}'
}

echo '== 本地 ROS 环境 =='
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI"

echo
echo '== 电脑端进程归属 =='
if pgrep -af '/rviz2/rviz2 .*far_real\.rviz' >/dev/null; then
  pass 'RViz 使用 nav/far_real.rviz'
elif pgrep -af '/rviz2/rviz2 .*\.rviz' >/dev/null; then
  fail 'RViz 已运行，但不是 nav/far_real.rviz'
else
  fail '没有运行中的 RViz'
fi
if cmp -s "${ROOT}/nav/map_sim.rviz" "${ROOT}/nav/far_real.rviz"; then
  pass 'nav/far_real.rviz 与目标布局 nav/map_sim.rviz 一致'
else
  warn 'nav/far_real.rviz 与 nav/map_sim.rviz 不一致'
fi
if pgrep -af '/local_planner/lib/local_planner/(localPlanner|pathFollower)' >/dev/null; then
  fail '电脑端出现 localPlanner/pathFollower；二者必须只在狗端'
else
  pass '电脑端没有 localPlanner/pathFollower'
fi
if pgrep -af 'python3 .*/nav/waypoint_sender\.py' >/dev/null; then
  fail '电脑端仍运行旧 waypoint_sender.py'
else
  pass '电脑端没有旧 waypoint_sender.py'
fi
if pgrep -af 'python3 .*/nav/global_path_sender\.py' >/dev/null; then
  pass '电脑端运行 global_path_sender.py'
else
  fail '电脑端缺少 global_path_sender.py'
fi

echo
echo '== 电脑端固定地图规划节点 =='
for node in /far_viz_receiver /pcdMapSim /terrainAnalysis \
            /terrainAnalysisExt /far_planner /global_path_sender \
            /robot_state_publisher; do
  if node_exists "$node"; then
    pass "$node"
  else
    fail "缺少节点 $node"
  fi
done
if [[ "$INITIALPOSE_ENABLE" != "0" ]]; then
  if node_exists /go2_initialpose_sender; then
    pass '/go2_initialpose_sender'
  else
    fail '缺少节点 /go2_initialpose_sender'
  fi
fi
for node in /localPlanner /pathFollower; do
  if node_exists "$node"; then
    fail "电脑端出现禁止节点 $node"
  else
    pass "电脑端没有 $node"
  fi
done

echo
echo '== 电脑端显示与规划话题 =='
for topic in /state_estimation /tf /tf_static /joint_states /robot_description \
             /overall_map /path /viz_path_topic /viz_graph_topic \
             /viz_poly_topic /viz_node_topic /navigation_boundary \
             /far/way_point; do
  require_publisher "$topic"
done
for topic in /viz_path_topic /viz_graph_topic /viz_poly_topic \
             /viz_node_topic /navigation_boundary /far/way_point; do
  require_single_publisher "$topic"
done
require_no_publisher /way_point

echo
echo '== TCP 通道所有权 =='
if [[ "$PATH_ENABLE" != "0" ]]; then
  if pgrep -af 'python3 .*/nav/global_path_sender\.py' >/dev/null; then
    pass '8899 只用于电脑 FAR 全局路径下行'
  else
    fail '8899 全局路径下行未启动'
  fi
fi

echo
echo '== 狗端规划与执行链 =='
dog_processes="$(remote_process_snapshot)"
if [[ -z "$dog_processes" ]]; then
  fail "无法读取狗端进程表（ssh $DOG_HOST）"
fi
for pattern in '[t]errainAnalysis' '[t]errainAnalysisExt' \
               '[l]ocalPlanner' '[p]athFollower' '[g]lobal_path_receiver.py' \
               '[f]ar_viz_link.py' '[i]nitialpose_link.py'; do
  if grep -Eq "$pattern" <<<"$dog_processes"; then
    pass "狗端存在 $pattern"
  else
    fail "狗端缺少 $pattern"
  fi
done
for pattern in '[f]ar_planner' '[f]ar_command_link.py'; do
  if grep -Eq "$pattern" <<<"$dog_processes"; then
    fail "狗端不应运行 $pattern"
  else
    pass "狗端没有 $pattern"
  fi
done
count=-1
if [[ "$PATH_ENABLE" != "0" ]]; then
  count="$(remote_publisher_count /way_point)"
  if [[ "$count" == "1" ]]; then
    pass '/way_point 在狗端唯一发布者'
  else
    fail "/way_point 在狗端发布者应为 1，实际=$count"
  fi
fi

echo
echo '== 控制权唯一性 =='
extra_control="$(
  timeout 18 ssh -o ConnectTimeout=5 -o BatchMode=yes "$DOG_HOST" \
    "ps -eo pid,args | grep -E '[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd' || true" \
    2>/dev/null || true
)"
if [[ -z "$extra_control" ]]; then
  pass '狗端没有 dog_control_B1_one / go2_control_by_sdk send_cmd'
else
  fail '狗端存在第二套运动控制节点：'
  while IFS= read -r line; do
    [[ -n "$line" ]] && printf '      %s\n' "$line"
  done <<<"$extra_control"
fi
if [[ "$count" == "1" ]]; then
  warn '局部执行链已就绪；物理遥控器接管仍由 Unitree 底层优先处理'
fi

echo
echo '== 最终链路日志 =='
receiver_log='/tmp/go2_real_state_receiver.log'
path_log='/tmp/go2_global_path_sender.log'
if [[ -r "$receiver_log" ]]; then
  if grep -q 'first SLAM odometry received' "$receiver_log"; then
    pass '电脑端已收到狗端重定位 odom'
  else
    warn '尚未收到狗端重定位 odom；先在 RViz 点 2D Pose Estimate'
  fi
  latest_status="$(tail -n 1 "$receiver_log")"
  if grep -Eq 'joints=[1-9][0-9]*' <<<"$latest_status"; then
    pass '电脑端已收到狗端关节状态'
  else
    warn '电脑端尚未收到狗端关节状态'
  fi
  if grep -Eq 'path=[1-9][0-9]*' <<<"$latest_status"; then
    pass '电脑端已收到狗端 localPlanner /path'
  else
    warn '尚未收到狗端 localPlanner /path'
  fi
else
  fail "缺少状态接收日志 $receiver_log"
fi
if [[ -r "$path_log" ]]; then
  if grep -Eq 'path_frames=[1-9][0-9]*' "$path_log"; then
    pass '电脑端已向狗端发送 FAR 全局路径'
  else
    warn '尚未发送 FAR 全局路径；先用 Goalpoint 设置电脑端目标'
  fi
else
  fail "缺少全局路径下发日志 $path_log"
fi

echo
echo "结果：failure=$failures warning=$warnings"
if (( failures > 0 )); then
  echo '结论：链路未通过；按 FAIL 修复，不要用双 FAR 或电脑端局部规划器补显示。'
  exit 1
fi
echo '结论：电脑固定地图全局规划 + 狗端局部避障执行链路通过只读检查。'
