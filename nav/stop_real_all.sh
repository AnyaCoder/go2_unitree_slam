#!/usr/bin/env bash
# Stop only the Go2 SLAM/FAR/navigation processes managed by this project.
# Unitree low-level services and physical remote-controller takeover are untouched.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"

stop_matching_local() {
  local pattern="$1" pid
  while IFS= read -r pid; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

echo '[1/3] 停止电脑端 systemd 视图服务'
systemctl --user stop go2-real-view.service 2>/dev/null || true
systemctl --user stop go2-rviz-real.service 2>/dev/null || true

echo '[2/3] 停止电脑端 FAR、RViz、状态与命令桥'
stop_matching_local 'python3 .*/nav/far_viz_receiver\.py'
stop_matching_local 'python3 .*/nav/waypoint_sender\.py'
stop_matching_local 'python3 .*/nav/initialpose_sender\.py'
stop_matching_local 'python3 .*/nav/pointcloud_overlay\.py'
stop_matching_local '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch map_sim map_sim\.launch\.py'
stop_matching_local '/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
stop_matching_local '^/home/anyacoder/autonomy_stack_go2/install/terrain_analysis/lib/terrain_analysis/'
stop_matching_local '^/home/anyacoder/autonomy_stack_go2/install/terrain_analysis_ext/lib/terrain_analysis_ext/'
stop_matching_local '^/home/anyacoder/autonomy_stack_go2/install/local_planner/lib/local_planner/'
stop_matching_local '^/home/anyacoder/autonomy_stack_go2/install/far_planner/lib/far_planner/'
stop_matching_local '/rviz2/rviz2 .*\.rviz'
stop_matching_local '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher'
stop_matching_local '^/opt/ros/foxy/lib/tf2_ros/static_transform_publisher'
stop_matching_local '/nav/run_real_view\.sh'

sleep 1
# Retry only exact project processes that ignored SIGTERM.
stop_matching_local 'python3 .*/nav/far_viz_receiver\.py'
stop_matching_local 'python3 .*/nav/waypoint_sender\.py'
stop_matching_local 'python3 .*/nav/initialpose_sender\.py'
stop_matching_local '/map_sim/lib/map_sim/(pcd_map_sim|space_viz)'
stop_matching_local '/rviz2/rviz2 .*\.rviz'

echo '[3/3] 停止狗端 UnitreeSlam、LIO、Livox、go2_nav、桥接与旧控制链'
ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" 'bash -s' <<'REMOTE'
set +e
patterns=(
  '[U]nitreeSlam'
  '[r]os2 launch /home/unitree/go2_nav/go2_nav.launch.py'
  '[p]ython3 /home/unitree/go2_nav/'
  '[n]av_safety_gate.py'
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
[[ $? -eq 0 ]] || echo 'WARN 狗端 SSH 不可用，未执行远端清理' >&2

echo
echo '--- 电脑端残留（应为空）---'
ps -eo pid,ppid,args | grep -E 'run_real_view|pcdMapSim|far_planner|localPlanner|pathFollower|nav_safety_gate|terrainAnalysis|far_viz_receiver|waypoint_sender|initialpose_sender|map_sim.launch|/rviz2/rviz2' | grep -v grep || true

echo '--- 狗端残留（应只剩 Unitree 底层，不应有本项目的导航/SLAM 节点）---'
ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
  "ps -eo pid,ppid,args | grep -E '[U]nitreeSlam|[g]o2_nav.launch|[l]ocalPlanner|[p]athFollower|[n]av_safety_gate|[t]errainAnalysis|[l]io_sam_ros2|[l]ivox_ros_driver2|[f]ar_viz_link|[f]ar_command_link|[i]nitialpose_link|[l]ive_robot_state.py ros|[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd|[h]igh_rate_state' || true" \
  2>/dev/null || true
