#!/usr/bin/env bash
# Stop the Go2 SLAM/navigation loop, transport bridges and laptop display processes.
# Unitree low-level services and physical remote-controller takeover are untouched.
set -uo pipefail

HOST="${FAR_VIZ_HOST:-go2-wifi}"

stop_matching_local() {
  local pattern="$1" pid
  while IFS= read -r pid; do
    if [[ -n "$pid" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

echo '[1/3] 停止电脑端 systemd 视图服务'
systemctl --user stop go2-real-view.service 2>/dev/null || true
systemctl --user stop go2-rviz-real.service 2>/dev/null || true

echo '[2/3] 停止电脑端显示、RViz、状态/FAR、Goalpoint 与初始位姿桥'
stop_matching_local 'python3 .*/nav/far_viz_receiver\.py'
stop_matching_local 'python3 .*/nav/goal_sender\.py'
stop_matching_local 'python3 .*/nav/waypoint_sender\.py'
stop_matching_local 'python3 .*/nav/initialpose_sender\.py'
stop_matching_local '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch map_sim map_sim\.launch\.py'
stop_matching_local '/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
stop_matching_local '^/autonomy_stack_go2/install/terrain_analysis/lib/terrain_analysis/'
stop_matching_local '^/autonomy_stack_go2/install/terrain_analysis_ext/lib/terrain_analysis_ext/'
stop_matching_local '^/autonomy_stack_go2/install/local_planner/lib/local_planner/'
stop_matching_local '^/autonomy_stack_go2/install/far_planner/lib/far_planner/'
stop_matching_local '/rviz2/rviz2 .*\.rviz'
stop_matching_local '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher'
stop_matching_local '^/opt/ros/foxy/lib/tf2_ros/static_transform_publisher'
stop_matching_local '/nav/view_real\.sh'

sleep 1
# Retry only exact project processes that ignored SIGTERM.
stop_matching_local 'python3 .*/nav/far_viz_receiver\.py'
stop_matching_local 'python3 .*/nav/goal_sender\.py'
stop_matching_local 'python3 .*/nav/waypoint_sender\.py'
stop_matching_local 'python3 .*/nav/initialpose_sender\.py'
stop_matching_local '/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
stop_matching_local '/rviz2/rviz2 .*\.rviz'

echo '[3/3] 停止狗端 UnitreeSlam、LIO、Livox、完整导航链、桥接与旧控制链'
remote_status=0
ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" 'bash -s' <<'REMOTE' || remote_status=$?
set +e
patterns=(
  '[U]nitreeSlam'
  '[r]os2 launch /home/unitree/go2_nav/go2_nav.launch.py'
  '[p]ython3 /home/unitree/go2_nav/'
  '[r]os2 launch task navigation_without_nav2_qt_test.launch.py'
  '[r]os2 launch livox_ros_driver2'
  '[l]ivox_ros_driver2_node'
  '[l]io_sam_ros2_'
  '[t]errainAnalysis'
  '[l]ocalPlanner'
  '[p]athFollower'
  '[f]ar_planner'
  '[h]igh_rate_state'
  '[g]o2_control_by_sdk'
  '[d]og_control_B1_one'
  '[n]av2_costmap'
  '[p]oint_to_gridmap_node'
  '[s]tatic_layer_node'
  '[s]elf_filter.py'
  '[c]loud_to_map_filter'
  '[s]tatic_map_replay.py'
  '[p]cdMapSim'
  '[m]id360_lidar.sh'
  '[r]e_location.sh'
)
for p in "${patterns[@]}"; do
  pkill -TERM -f "$p" 2>/dev/null || true
done
sleep 3
for p in "${patterns[@]}"; do
  pkill -KILL -f "$p" 2>/dev/null || true
done
sleep 1
REMOTE
if [[ "$remote_status" -ne 0 ]]; then
  echo 'WARN 狗端 SSH 不可用，未执行远端清理' >&2
fi

echo
echo '--- 电脑端残留（应为空）---'
ps -eo pid,ppid,args | grep -E 'view_real|pcdMapSim|far_planner|localPlanner|pathFollower|terrainAnalysis|far_viz_receiver|goal_sender|waypoint_sender|initialpose_sender|map_sim.launch|/rviz2/rviz2' | grep -v grep || true

echo '--- 狗端残留（应只剩 Unitree 底层，不应有本项目的导航/SLAM 节点）---'
ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
  "ps -eo pid,ppid,args | grep -E '[U]nitreeSlam|[g]o2_nav.launch|[s]elf_filter.py|[c]loud_to_map_filter|[s]tatic_map_replay.py|[p]cdMapSim|[l]ocalPlanner|[p]athFollower|[t]errainAnalysis|[f]ar_planner|[l]io_sam_ros2|[l]ivox_ros_driver2|[f]ar_viz_link|[f]ar_command_link|[g]lobal_path_receiver|[i]nitialpose_link|[l]ive_robot_state.py ros|[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd|[h]igh_rate_state' || true" \
  2>/dev/null || true
