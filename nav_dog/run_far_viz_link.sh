#!/usr/bin/env bash
# Go2 state/obstacle uplink: odom, TF, joints and an optional minimal cloud.
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/setup.bash
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export FAR_VIZ_PORT="${FAR_VIZ_PORT:-8898}"
export FAR_VIZ_CLOUD_ENABLE="${FAR_VIZ_CLOUD_ENABLE:-1}"
args=(--port "$FAR_VIZ_PORT")
if [[ "$FAR_VIZ_CLOUD_ENABLE" == "0" ]]; then
  args+=(--no-cloud)
fi
exec python3 /home/unitree/go2_nav/far_viz_link.py "${args[@]}"
