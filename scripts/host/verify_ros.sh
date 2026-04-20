#!/usr/bin/env bash
# Host-side ROS 2 verification. Run this in a separate terminal after
# `./run.sh sim` (freerun) or `./run.sh sim` with SYNC_MODE=handshake.
set -eu

log() { printf '\033[1;34m[verify-ros]\033[0m %s\n' "$*"; }

# FastDDS must match the container transport.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    cat >&2 <<'EOF'
[verify-ros] /opt/ros/jazzy/setup.bash missing — ROS 2 Jazzy is not installed on the host.
  install it with:
    ./scripts/host/install.sh --with-ros
  (installs ros-jazzy-desktop + ros-jazzy-ur-description, idempotent)
EOF
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[verify-ros] 'ros2' CLI not on PATH after sourcing setup.bash — ROS install looks broken" >&2
    exit 1
fi

log "ros2 topic list (expect /clock, /joint_states, /joint_command)"
ros2 topic list

log "ros2 topic hz /clock (2s sample)"
timeout 3 ros2 topic hz /clock || true

log "ros2 topic hz /joint_states (2s sample — freerun only)"
timeout 3 ros2 topic hz /joint_states || true

log "ros2 topic echo /joint_states -n 1"
timeout 3 ros2 topic echo --once /joint_states || true

MODE="${SYNC_MODE:-freerun}"
if [[ "${MODE}" == "handshake" ]]; then
    log "handshake: call /sim/step 5x"
    for i in 1 2 3 4 5; do
        ros2 service call /sim/step std_srvs/srv/Trigger "{}" | grep -E "success|message"
    done
    log "handshake: /sim/reset"
    ros2 service call /sim/reset std_srvs/srv/Trigger "{}" | grep -E "success|message"
fi

log "done."
