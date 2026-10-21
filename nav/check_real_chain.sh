#!/usr/bin/env bash
# Read-only acceptance check for the architecture in 必看.md.
# It never starts, stops, publishes, or modifies anything.
set -uo pipefail

ROOT="${HOME}/go2_unitree_slam"
DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
WS="${WS:-${HOME}/autonomy_stack_go2}"
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

node_exists() {
  timeout 5 ros2 node list 2>/dev/null | grep -Fxq "$1"
}

echo '== 本地 ROS 环境 =='
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI"

echo
echo '== 进程归属 =='
if pgrep -af '/map_sim/pcd_map_sim' | grep -q 'external_odom_topic'; then
  pass 'pcdMapSim 进程参数包含 external_odom_topic'
elif pgrep -af '/map_sim/pcd_map_sim' >/dev/null; then
  warn 'pcdMapSim 正在运行，但命令行未直接显示 external_odom_topic；继续检查 ROS 参数'
else
  fail '没有 pcdMapSim 进程'
fi

if pgrep -af '/rviz2/rviz2 .*far_real\.rviz' >/dev/null; then
  pass 'RViz 使用 nav/far_real.rviz'
elif pgrep -af '/rviz2/rviz2 .*\.rviz' >/dev/null; then
  fail 'RViz 已运行，但不是 nav/far_real.rviz'
else
  fail '没有运行中的 RViz'
fi

if cmp -s "${ROOT}/nav/map_sim.rviz" "${ROOT}/nav/far_real.rviz"; then
  pass 'nav/far_real.rviz 与目标布局 nav/map_sim.rviz 完全一致'
else
  fail 'nav/far_real.rviz 与目标布局 nav/map_sim.rviz 不一致'
fi

echo
echo '== 必需节点 =='
for node in /pcdMapSim /terrainAnalysis /terrainAnalysisExt \
            /localPlanner /pathFollower /far_planner /robot_state_publisher; do
  if node_exists "$node"; then
    pass "$node"
  else
    fail "缺少节点 $node"
  fi
done

echo
echo '== pcdMapSim 输入 =='
if node_exists /pcdMapSim; then
  value="$(timeout 5 ros2 param get /pcdMapSim external_odom_topic 2>/dev/null || true)"
  if grep -Fq '/state_estimation' <<<"$value"; then
    pass '/pcdMapSim.external_odom_topic=/state_estimation'
  else
    fail "/pcdMapSim 未跟随真机 /state_estimation：$value"
  fi
  value="$(timeout 5 ros2 param get /pcdMapSim pcd_path 2>/dev/null || true)"
  if grep -Fq 'GlobalMap_clean_20261020_1920_nav_green_edited.pcd' <<<"$value"; then
    pass '/pcdMapSim 使用固定的 nav_green_edited.pcd'
  else
    fail "/pcdMapSim 使用的 PCD 不是固定文件：$value"
  fi
  value="$(timeout 5 ros2 node info /pcdMapSim 2>/dev/null || true)"
  if grep -Fq '/registered_scan' <<<"$value"; then
    pass '/pcdMapSim 发布 /registered_scan（本模式未额外创建 /sim/registered_scan 别名）'
  else
    fail '/pcdMapSim 没有发布 /registered_scan'
  fi
fi

echo
echo '== 关键话题 =='
for topic in /state_estimation /tf /tf_static /joint_states /robot_description \
             /registered_scan /terrain_map /terrain_map_ext \
             /overall_map /path /free_paths \
             /viz_graph_topic /viz_path_topic /viz_node_topic /navigation_boundary; do
  require_publisher "$topic"
done

echo
echo '== FAR 控制权唯一性：禁止 UnitreeSlam 旧运动控制 =='
dog_host="${FAR_VIZ_HOST:-go2-wifi}"
extra_control="$(
  timeout 18 ssh -o ConnectTimeout=5 -o BatchMode=yes "$dog_host" \
    "ps -eo pid,args | grep -E '[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd' || true" \
    2>/dev/null || true
)"
if [[ -z "$extra_control" ]]; then
  pass '狗端没有 dog_control_B1_one / go2_control_by_sdk send_cmd'
else
  fail '狗端存在第二套运动控制节点；FAR 控制权不唯一：'
  while IFS= read -r line; do
    [[ -n "$line" ]] && printf '      %s\n' "$line"
  done <<<"$extra_control"
fi

echo '== 狗端 Goalpoint 安全闸 =='
dog_ros_domain="${GO2_DOG_ROS_DOMAIN_ID:-0}"
dog_ros() {
  local command="$1"
  timeout 20 ssh -o ConnectTimeout=5 -o BatchMode=yes "$dog_host" \
    "set +u; source /opt/ros/foxy/setup.bash; \
     source /unitree/module/graph_pid_ws/install/local_setup.bash; \
     source /home/unitree/autonomy_stack_go2/install/setup.bash; set -u; \
     export ROS_DOMAIN_ID='${dog_ros_domain}'; export ROS_LOCALHOST_ONLY=0; \
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
     export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}; ${command}" \
    2>/dev/null || true
}

gate_process="$(
  timeout 18 ssh -o ConnectTimeout=5 -o BatchMode=yes "$dog_host" \
    "ps -eo pid,args | grep '[n]av_safety_gate.py' || true" 2>/dev/null || true
)"
if [[ -n "$gate_process" ]]; then
  pass "狗端安全闸进程存在：$gate_process"
else
  fail '狗端没有 nav_safety_gate.py；Goalpoint 无硬安全闸'
fi

api_info="$(dog_ros 'timeout 5 ros2 topic info /api/sport/request')"
api_pub_count="$(awk '/Publisher count:/ {print $3}' <<<"$api_info")"
if [[ "$api_pub_count" =~ ^[0-9]+$ ]] && (( api_pub_count == 1 )); then
  pass '/api/sport/request 只有一个发布者（应为 nav_safety_gate）'
else
  fail "/api/sport/request 发布者数量必须恰好为 1，实际=${api_pub_count:-unknown}"
fi

internal_info="$(dog_ros 'timeout 5 ros2 topic info /nav/path_follower_request')"
internal_pub_count="$(awk '/Publisher count:/ {print $3}' <<<"$internal_info")"
if [[ "$internal_pub_count" =~ ^[0-9]+$ ]] && (( internal_pub_count >= 1 )); then
  pass "/nav/path_follower_request publisher=$internal_pub_count（pathFollower 已被重映射）"
else
  fail 'pathFollower 没有发布到安全闸内部话题 /nav/path_follower_request'
fi

safety_yaml="$(
  dog_ros 'timeout 6 ros2 topic echo /nav/safety_status std_msgs/msg/String \
    --qos-reliability reliable --qos-durability transient_local'
)"
safety_json="$(
  python3 -c 'import sys,yaml; docs=list(yaml.safe_load_all(sys.stdin)); print(docs[0].get("data", "") if docs and isinstance(docs[0], dict) else "")' \
    <<<"$safety_yaml" 2>/dev/null || true
)"
if [[ -n "$safety_json" ]]; then
  safety_verdict="$(
    python3 -c 'import json,sys; d=json.load(sys.stdin); print("ready=%s motion=%s imu=%s pose=%s loc=%s reason=%s" % (str(d.get("goal_commands_ready")).lower(), str(d.get("motion_ready")).lower(), d.get("imu",{}).get("state"), bool(d.get("initial_pose")), bool(d.get("localization_confirmed")), (d.get("reasons") or ["NONE"])[0]))' \
      <<<"$safety_json" 2>/dev/null || true
  )"
  safety_ready="$(python3 -c 'import json,sys; print(str(json.load(sys.stdin).get("goal_commands_ready")).lower())' <<<"$safety_json" 2>/dev/null || echo error)"
  safety_motion="$(python3 -c 'import json,sys; print(str(json.load(sys.stdin).get("motion_ready")).lower())' <<<"$safety_json" 2>/dev/null || echo error)"
  if [[ "$safety_ready" == "true" ]]; then
    pass "Goalpoint 前置条件已满足：$safety_verdict"
  else
    fail "不得使用 Goalpoint：$safety_verdict"
  fi
  if [[ "$safety_motion" == "true" ]]; then
    pass '当前 goal/waypoint/path 有效，安全闸允许运动'
  else
    warn '当前尚未形成允许运动的完整目标链；Goalpoint 不会被放行'
  fi
else
  fail '读不到 /nav/safety_status；无法证明 IMU/初始位姿/定位门控有效'
fi

echo
echo '== 人工可走区必须不存在 =='
if pgrep -af '/map_sim/space_viz' >/dev/null; then
  fail 'spaceViz 仍在运行；固定静态地图模式不允许人工可走区'
else
  pass 'spaceViz 未启动'
fi
if ros2 topic list 2>/dev/null | grep -Fxq '/sim/free_space'; then
  fail '仍存在 /sim/free_space'
else
  pass '没有 /sim/free_space'
fi
if ros2 topic list 2>/dev/null | grep -Fxq '/sim/free_boundary'; then
  fail '仍存在 /sim/free_boundary'
else
  pass '没有 /sim/free_boundary'
fi

echo
echo '== 模拟机器人源必须不存在 =='
if ros2 topic list 2>/dev/null | grep -Fxq '/ground_truth/state'; then
  fail '发现 /ground_truth/state，疑似混入模拟机器人源'
else
  pass '没有 /ground_truth/state'
fi
if ros2 topic list 2>/dev/null | grep -Fxq '/cmd_vel_from_sim'; then
  fail '发现 /cmd_vel_from_sim，疑似混入模拟控制源'
else
  pass '没有 /cmd_vel_from_sim'
fi

echo
echo '== Go2 状态日志 =='
receiver_log='/tmp/go2_real_state_receiver.log'
if [[ -r "$receiver_log" ]]; then
  if grep -q 'first SLAM odometry received' "$receiver_log"; then
    pass '已收到首帧真机重定位 odom'
  else
    warn '尚未收到首帧真机重定位 odom；先在 RViz 使用 2D Pose Estimate'
  fi
  latest_status="$(tail -n 1 "$receiver_log")"
  if grep -Eq 'joints=[1-9][0-9]*' <<<"$latest_status"; then
    pass '状态接收器已收到真机关节消息'
  elif grep -q 'joints=0' <<<"$latest_status"; then
    fail '状态接收器尚无关节消息：电脑端 joints=0'
  else
    warn '接收器日志中暂未找到 joints 计数'
  fi
  if grep -q 'cloud=0' <<<"$latest_status"; then
    pass '最小动态障碍云当前关闭'
  elif grep -q 'cloud=[1-9]' <<<"$latest_status"; then
    warn '最小动态障碍云有数据；确认这是显式启用而非默认行为'
  else
    warn '接收器日志中暂未找到 cloud 计数'
  fi
else
  fail "缺少状态接收日志 $receiver_log"
fi

echo
echo "结果：failure=$failures warning=$warnings"
if (( failures > 0 )); then
  echo '结论：链路未通过。按 FAIL 项修复，不允许用模拟数据补显示。'
  exit 1
fi
echo '结论：真机状态驱动链路通过当前只读检查。'
