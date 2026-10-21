#!/usr/bin/env bash
# 打开真机 FAR RViz。只启动可视化，不重启狗端或本地导航栈。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROOT}/nav/far_real.rviz"
ROS_SETUP="/opt/ros/foxy/setup.bash"
AUTONOMY_SETUP="${HOME}/autonomy_stack_go2/install/setup.bash"

[[ -f "$CONFIG" ]] || { echo "缺少 RViz 配置: $CONFIG" >&2; exit 2; }
[[ -f "$ROS_SETUP" ]] || { echo "缺少 ROS 环境: $ROS_SETUP" >&2; exit 2; }
[[ -f "$AUTONOMY_SETUP" ]] || { echo "缺少 autonomy_stack_go2 环境: $AUTONOMY_SETUP" >&2; exit 2; }

# 与 nav/run_real_view.sh 使用完全相同的 ROS 域和 DDS 隔离配置。
set +u
source "$ROS_SETUP"
source "$AUTONOMY_SETUP"
set -u
export ROS_DOMAIN_ID="${GO2_LOCAL_MAP_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${ROOT}/nav/cyclonedds_lo.xml"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"

if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "RViz 配置=$CONFIG"
echo "正在检查真机地图栈..."

nodes="$(timeout 8 ros2 node list 2>/dev/null || true)"
topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"

if ! grep -qx '/pcdMapSim' <<<"$nodes"; then
  echo >&2
  echo "未发现 /pcdMapSim。请先在另一个终端运行：" >&2
  echo "  cd ${ROOT} && ./nav/run_real_view.sh" >&2
  echo "本脚本不会擅自重启该服务。" >&2
  exit 3
fi
if ! grep -qx '/overall_map' <<<"$topics"; then
  echo "警告：当前未发现 /overall_map 话题，RViz 可能暂时看不到静态地图。" >&2
fi
if ! grep -qx '/state_estimation' <<<"$topics"; then
  echo "警告：当前未发现 /state_estimation 话题，Go2 位姿可能尚未就绪。" >&2
fi

# 正式配置已关闭点云 Decay，避免 AMDGPU + Ogre 1.12 持续分配
# 顶点缓冲后报 "Cannot create GL vertex buffer"。默认使用硬件渲染。
# 仅当硬件驱动再次异常时，可临时设置 GO2_RVIZ_SOFTWARE=1。
if [[ "${GO2_RVIZ_SOFTWARE:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  echo "RViz 渲染：Mesa llvmpipe（临时兼容模式）"
else
  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER 2>/dev/null || true
  echo "RViz 渲染：硬件 OpenGL"
fi

# 当前已有 RViz 不影响新实例；不要 kill，避免误伤正在使用的窗口。
echo "启动 RViz。若稍后显示 No tf data，请等待 2 秒，并确认狗端已完成定位或点一次 2D Pose Estimate。"
exec ros2 run rviz2 rviz2 -d "$CONFIG"
