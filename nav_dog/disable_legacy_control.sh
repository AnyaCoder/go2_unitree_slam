#!/usr/bin/env bash
# Remove the UnitreeSlam legacy motion-control chain while preserving
# UnitreeSlam, Livox, LIO/relocalization, localPlanner and pathFollower.
# The physical remote controller and Unitree low-level services are untouched.
set -euo pipefail

HOST="${HOST:-go2-wifi}"

ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" 'bash -s' <<'REMOTE'
set -euo pipefail

python3 - <<'PY'
from pathlib import Path
from datetime import datetime

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

def backup(path):
    bak = path.with_name(path.name + ".bak-far-only-" + stamp)
    if not bak.exists():
        bak.write_bytes(path.read_bytes())
        print(f"backup {path} -> {bak}")

def patch(path, old, new, marker, label):
    text = path.read_text()
    if old in text:
        backup(path)
        path.write_text(text.replace(old, new, 1))
        print(f"patched {label}")
    elif marker in text:
        print(f"{label} already patched")
    else:
        raise SystemExit(f"{path}: expected block not found")
    return path.read_text()

reloc = Path("/unitree/module/graph_pid_ws/src/QT_Server/shell/re_location.sh")
launch_paths = [
    Path("/unitree/module/graph_pid_ws/src/task/launch/navigation_without_nav2_qt_test.launch.py"),
    Path("/unitree/module/graph_pid_ws/install/task/share/task/launch/navigation_without_nav2_qt_test.launch.py"),
]

patch(
    reloc,
    "{\nros2 run go2_control_by_sdk send_cmd\n}&\n",
    (
        "# FAR-only mode: legacy motion command bridge is intentionally disabled.\n"
        "# Re-enable only when running the original UnitreeSlam navigation stack.\n"
        "# {\n"
        "# ros2 run go2_control_by_sdk send_cmd\n"
        "# }&\n"
    ),
    "legacy motion command bridge is intentionally disabled",
    "re_location.sh: disabled go2_control_by_sdk send_cmd",
)

old_node = (
    "        Node(\n"
    "            package=\"dog_control\",\n"
    "            executable=\"dog_control_B1_one\",\n"
    "            output='screen'\n"
    "        ),\n"
)
new_node = (
    "        # FAR-only relocation: keep LIO/initial-pose nodes, but never start\n"
    "        # the legacy dog_control motion chain.\n"
    "        # Node(\n"
    "        #     package=\"dog_control\",\n"
    "        #     executable=\"dog_control_B1_one\",\n"
    "        #     output='screen'\n"
    "        # ),\n"
)
launch_texts = []
for launch in launch_paths:
    launch_texts.append(
        patch(
            launch,
            old_node,
            new_node,
            "legacy dog_control motion chain",
            f"{launch}: disabled dog_control_B1_one",
        )
    )

for launch, text in zip(launch_paths, launch_texts):
    compile(text, str(launch), "exec")
    print(f"launch syntax: OK {launch}")
PY

# Stop only the legacy motion-control processes. Keep the launch parent because
# it also owns LIO/relocalization, and do not touch Unitree low-level services.
pkill -TERM -f '[d]og_control_B1_one' 2>/dev/null || true
pkill -TERM -f '[g]o2_control_by_sdk.*send_cmd' 2>/dev/null || true
sleep 2

pkill -KILL -f '[d]og_control_B1_one' 2>/dev/null || true
pkill -KILL -f '[g]o2_control_by_sdk.*send_cmd' 2>/dev/null || true
sleep 1

echo "--- forbidden motion processes (must be empty) ---"
ps -eo pid,ppid,args |
  grep -E '[d]og_control_B1_one|[g]o2_control_by_sdk.*send_cmd' ||
  true

echo "--- required processes still alive ---"
ps -eo pid,ppid,args |
  grep -E '[U]nitreeSlam|[l]io_sam_ros2|[l]ivox_lidar_publisher|[l]ocalPlanner|[p]athFollower|[f]ar_viz_link|[f]ar_command_link|[i]nitialpose_link' |
  head -n 100 ||
  true
REMOTE
