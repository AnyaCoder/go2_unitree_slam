#!/usr/bin/env bash
# Stop only the laptop-side data view processes started by view_real.sh.
# The Go2, SLAM, planners and remote bridges are never touched.
set -uo pipefail

RUNTIME_DIR="${GO2_VIEW_RUNTIME_DIR:-/tmp/go2_real_view}"

stop_pid_file() {
  local name="$1" pid_file="$RUNTIME_DIR/$1.pid" pid
  if [[ ! -s "$pid_file" ]]; then
    return 0
  fi
  pid="$(cat "$pid_file")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    echo "停止 $name (pid=$pid)"
  fi
  : >"$pid_file"
}

for name in initialpose_sender goal_sender far_viz_receiver \
            robot_state_publisher base_vehicle sensor_base map_anchor; do
  stop_pid_file "$name"
done

for pattern in \
  '^/opt/ros/foxy/lib/rviz2/rviz2 .*far_real\.rviz$' \
  '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 run rviz2 rviz2'; do
  while IFS= read -r pid; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
done

echo '电脑端数据查看进程已停止；狗端未改动'
