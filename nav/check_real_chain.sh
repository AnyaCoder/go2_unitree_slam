#!/usr/bin/env bash
# Read-only acceptance check for the full Go2 planning loop and display-only laptop.
# It never starts, stops, publishes, or modifies anything.
set -uo pipefail

ROOT="${HOME}/go2_unitree_slam"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
DOG_HOST="${FAR_VIZ_HOST:-go2-wifi}"
GOAL_ENABLE="${GO2_GOAL_ENABLE:-${FAR_COMMAND_ENABLE:-1}}"
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

remote_topic_info() {
  timeout 18 ssh -o ConnectTimeout=5 -o BatchMode=yes "$DOG_HOST" \
    "bash -lc 'set +u; source /opt/ros/foxy/setup.bash; \
      source /unitree/module/graph_pid_ws/install/local_setup.bash; \
      source /home/unitree/autonomy_stack_go2/install/setup.bash; \
      export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH; \
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
      export ROS_LOCALHOST_ONLY=0; \
      timeout 6 ros2 topic info -v $1'" 2>/dev/null || true
}

remote_publisher_count() {
  remote_topic_info "$1" |
    awk '/Publisher count:/ {print $3; found=1} END {if (!found) print -1}'
}

echo '== 本地 ROS 环境 =='
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI"

echo
echo '== 电脑端进程归属（必须只显示，不规划） =='
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
if pgrep -af 'python3 .*/nav/far_viz_receiver\.py' >/dev/null; then
  pass '电脑端运行状态/FAR 可视化接收器'
else
  fail '电脑端缺少 far_viz_receiver.py'
fi
if [[ "$GOAL_ENABLE" != "0" ]]; then
  if pgrep -af 'python3 .*/nav/goal_sender\.py' >/dev/null; then
    pass '电脑端运行最终 Goalpoint 下行'
  else
    fail '电脑端缺少 goal_sender.py'
  fi
fi
for pattern in \
  'nav/global_planner\.launch\.py' \
  '/map_sim/lib/map_sim/pcd_map_sim' \
  '/terrain_analysis/lib/terrain_analysis/terrainAnalysis' \
  '/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt' \
  '/far_planner/lib/far_planner/far_planner' \
  '/local_planner/lib/local_planner/localPlanner' \
  '/local_planner/lib/local_planner/pathFollower'; do
  if pgrep -af "$pattern" >/dev/null; then
    fail "电脑端出现禁止进程 $pattern"
  else
    pass "电脑端没有 $pattern"
  fi
done

echo
echo '== 电脑端显示节点与话题 =='
for node in /far_viz_receiver /robot_state_publisher; do
  if node_exists "$node"; then
    pass "$node"
  else
    fail "缺少节点 $node"
  fi
done
if [[ "$GOAL_ENABLE" != "0" ]]; then
  if node_exists /go2_goal_sender; then
    pass '/go2_goal_sender'
  else
    fail '缺少节点 /go2_goal_sender'
  fi
fi
if [[ "$INITIALPOSE_ENABLE" != "0" ]]; then
  if node_exists /go2_initialpose_sender; then
    pass '/go2_initialpose_sender'
  else
    fail '缺少节点 /go2_initialpose_sender'
  fi
fi
for node in /pcdMapSim /terrainAnalysis /terrainAnalysisExt /far_planner \
            /localPlanner /pathFollower; do
  if node_exists "$node"; then
    fail "电脑端出现禁止节点 $node"
  else
    pass "电脑端没有 $node"
  fi
done
for topic in /state_estimation /tf /tf_static /joint_states /robot_description \
             /overall_map /path /way_point /viz_path_topic /viz_graph_topic \
             /viz_node_topic /viz_poly_topic /navigation_boundary; do
  require_publisher "$topic"
done
for topic in /path /way_point /viz_path_topic /viz_graph_topic /viz_node_topic \
             /viz_poly_topic /navigation_boundary; do
  require_single_publisher "$topic"
done
require_no_publisher /go2/min_obstacle_cloud
require_no_publisher /added_obstacles

echo
echo '== TCP 通道所有权 =='
if [[ "$GOAL_ENABLE" != "0" ]]; then
  if pgrep -af 'python3 .*/nav/goal_sender\.py' >/dev/null; then
    pass '8899 只用于电脑最终 Goalpoint 下行'
  else
    fail '8899 Goalpoint 下行未启动'
  fi
fi

echo
echo '== 狗端完整规划与执行链 =='
dog_processes="$(remote_process_snapshot)"
if [[ -z "$dog_processes" ]]; then
  fail "无法读取狗端进程表（ssh $DOG_HOST）"
fi
required_dog_processes=(
  '[n]av_pose_bridge.py'
  '[s]tatic_map_replay.py'
  '[t]errainAnalysis'
  '[t]errainAnalysisExt'
  '[f]ar_planner'
  '[l]ocalPlanner'
  '[p]athFollower'
  '[f]ar_viz_link.py'
)
if [[ "$GOAL_ENABLE" != "0" ]]; then
  required_dog_processes+=('[f]ar_command_link.py')
fi
if [[ "$INITIALPOSE_ENABLE" != "0" ]]; then
  required_dog_processes+=('[i]nitialpose_link.py')
fi
for pattern in "${required_dog_processes[@]}"; do
  if grep -Eq "$pattern" <<<"$dog_processes"; then
    pass "狗端存在 $pattern"
  else
    fail "狗端缺少 $pattern"
  fi
done
for pattern in '[g]lobal_path_receiver.py' '[g]lobal_path_sender.py'; do
  if grep -Eq "$pattern" <<<"$dog_processes"; then
    fail "狗端不应运行旧全局路径桥 $pattern"
  else
    pass "狗端没有 $pattern"
  fi
done
for topic in /nav/state_estimation /nav/static_cloud_map /path /way_point /viz_path_topic \
             /viz_graph_topic /viz_node_topic /viz_poly_topic \
             /navigation_boundary; do
  count="$(remote_publisher_count "$topic")"
  if [[ "$count" == "1" ]]; then
    pass "$topic 在狗端唯一发布者"
  else
    fail "$topic 在狗端发布者应为 1，实际=$count"
  fi
done
sport_info="$(remote_topic_info /api/sport/request)"
if grep -Fq 'Node name: pathFollower' <<<"$sport_info"; then
  pass '/api/sport/request 的有效 ROS 发布者是 pathFollower'
else
  fail '/api/sport/request 上没有 pathFollower 发布者'
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

echo
echo '== 链路日志 =='
receiver_log='/tmp/go2_real_state_receiver.log'
goal_log='/tmp/go2_goal_sender.log'
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
    pass '电脑端已收到狗端 localPlanner 的 /path'
  else
    warn '尚未收到狗端 /path；完成重定位并点 Goalpoint 后应出现'
  fi
  if grep -Eq 'graph=[1-9][0-9]*' <<<"$latest_status"; then
    pass '电脑端已收到狗端 FAR V-Graph 可视化'
  else
    warn '尚未收到狗端 FAR V-Graph'
  fi
else
  fail "缺少状态接收日志 $receiver_log"
fi
if [[ -r "$goal_log" ]]; then
  if grep -Eq 'sent=[1-9][0-9]*' "$goal_log"; then
    pass '电脑端已向狗端发送最终 Goalpoint'
  else
    warn '尚未发送 Goalpoint；完成重定位后在 RViz 点 Goalpoint'
  fi
else
  fail "缺少 Goalpoint 发送日志 $goal_log"
fi

echo
echo "结果：failure=$failures warning=$warnings"
if (( failures > 0 )); then
  echo '结论：链路未通过；电脑端不得补跑 FAR/localPlanner/pathFollower。'
  exit 1
fi
echo '结论：狗端完整规划执行 + 电脑端只读显示链路通过只读检查。'
