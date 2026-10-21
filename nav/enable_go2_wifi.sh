#!/usr/bin/env bash
# Enable the Go2's saved Go2-Explore Wi-Fi hotspot over the wired SSH link.
# The sudo password is entered interactively on the Go2 and is never stored.
set -euo pipefail

HOST="${GO2_WIRED_HOST:-192.168.123.18}"
USER_NAME="${GO2_USER:-unitree}"
PROFILE="${GO2_WIFI_PROFILE:-go2-hotspot}"

echo "Go2 wired target: ${USER_NAME}@${HOST}"
echo "Activating Wi-Fi profile: ${PROFILE}"
echo "The next prompt is the Go2 user's sudo password."

ssh -o ConnectTimeout=6 -o ServerAliveInterval=5 -t "${USER_NAME}@${HOST}" "
  set -e
  sudo nmcli connection up '${PROFILE}'
  sleep 2
  echo
  echo '--- wlan0 ---'
  ip -brief addr show wlan0
  nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status | grep '^wlan0:' || true
  echo
  echo 'Expected hotspot address: 192.168.76.1'
"
