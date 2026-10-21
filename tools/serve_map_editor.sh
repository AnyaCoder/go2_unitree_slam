#!/usr/bin/env bash
# Serve the vendored ROS SLAM Map Editor locally.
# usage: ./serve_map_editor.sh [port] [bind-address]
set -euo pipefail
PORT="${1:-${MAP_EDITOR_PORT:-8090}}"
HOST="${2:-127.0.0.1}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/ros_slam_map_editor" && pwd)"
echo "ROS SLAM Map Editor: http://${HOST}:${PORT}/editor.html"
cd "$DIR"
exec python3 -m http.server "$PORT" --bind "$HOST"
