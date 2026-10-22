#!/usr/bin/env bash
# Read-only dog-side frequency audit for the full navigation chain.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${FAR_VIZ_HOST:-go2-wifi}"
DURATION="${NAV_AUDIT_DURATION:-10}"
STRICT=0
JSON_OUT="/tmp/go2_nav_rate_audit.json"

while (( $# )); do
  case "$1" in
    --duration)
      DURATION="${2:?--duration needs seconds}"
      shift 2
      ;;
    --strict)
      STRICT=1
      shift
      ;;
    -h|--help)
      cat <<'EOF'
usage: ./nav/audit_real_rates.sh [--duration SEC] [--strict]

Read-only sampling on the Go2. --strict fails when a required topic is missing
or its arrival rate is more than 10% outside the configured window.
EOF
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "invalid duration: $DURATION" >&2
  exit 2
fi

ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" 'mkdir -p /home/unitree/go2_nav'
scp -q "${ROOT}/nav_dog/nav_rate_audit.py" \
  "$HOST:/home/unitree/go2_nav/nav_rate_audit.py"

strict_arg=''
if [[ "$STRICT" == "1" ]]; then
  strict_arg='--strict'
fi

echo "Sampling dog-side navigation topics for ${DURATION}s..."
ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" \
  "bash -lc 'set +u; source /opt/ros/foxy/setup.bash; source /unitree/module/graph_pid_ws/install/local_setup.bash; source /home/unitree/autonomy_stack_go2/install/setup.bash; export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_LOCALHOST_ONLY=0; cd /home/unitree/go2_nav; python3 nav_rate_audit.py --duration $DURATION --json-out $JSON_OUT $strict_arg'"
echo "JSON: $HOST:$JSON_OUT"
