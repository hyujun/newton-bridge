#!/usr/bin/env bash
# Launch / manage the newton-bridge container.
#
# Usage:
#   ./scripts/host/run.sh                              # up (foreground, robot=ur5e, freerun)
#   ROBOT=franka ./scripts/host/run.sh                 # pick pack
#   SYNC_MODE=sync ./scripts/host/run.sh               # pick sync mode
#   ./scripts/host/run.sh sim                          # same as default up
#   ./scripts/host/run.sh shell                        # interactive bash
#   ./scripts/host/run.sh example basic_pendulum       # run a Newton example (viewer gl)
#   ./scripts/host/run.sh jupyter                      # start Jupyter on host:8888
#   ./scripts/host/run.sh verify                       # in-container smoke test
#   ./scripts/host/run.sh down                         # stop + remove container
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

COMPOSE=(docker compose -f docker/compose.yml)

# X11 passthrough (safe no-op if already granted).
if command -v xhost >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

# Robot pack selector — picks which robots/<name>/ the sim loads.
#
# Two modes:
#   1. Built-in pack: ROBOT=<name> (default ur5e) → /workspace/robots/<name>
#   2. External pack: EXTERNAL_PACK_HOST=/abs/host/path → /workspace/external_pack
#      The host path is bind-mounted ro into the container and ROBOT_PACK is
#      auto-pointed at it. The folder must contain robot.yaml + a flat URDF
#      (xacro is not supported on this path — pre-process to URDF on the host).
: "${ROBOT:=ur5e}"
if [[ -n "${EXTERNAL_PACK_HOST:-}" ]]; then
    if [[ ! -d "${EXTERNAL_PACK_HOST}" ]]; then
        echo "[run.sh] EXTERNAL_PACK_HOST is not a directory: ${EXTERNAL_PACK_HOST}" >&2
        exit 2
    fi
    if [[ ! -f "${EXTERNAL_PACK_HOST}/robot.yaml" ]]; then
        echo "[run.sh] ${EXTERNAL_PACK_HOST}/robot.yaml not found — see docs/ROBOTS.md (Path D)" >&2
        exit 2
    fi
    EXTERNAL_PACK_HOST="$(cd "${EXTERNAL_PACK_HOST}" && pwd)"  # absolutize
    : "${ROBOT_PACK:=/workspace/external_pack}"
    export EXTERNAL_PACK_HOST
fi
: "${ROBOT_PACK:=/workspace/robots/${ROBOT}}"
: "${SYNC_MODE:=freerun}"
: "${FREERUN_RATE:=realtime}"
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
: "${VIEWER:=rerun}"
export ROBOT_PACK SYNC_MODE FREERUN_RATE ROS_DOMAIN_ID RMW_IMPLEMENTATION VIEWER

# FastDDS only: force UDPv4 because shared-memory transport breaks at the
# container/host UID boundary. Cyclone DDS has no equivalent knob and doesn't
# need one.
if [[ "${RMW_IMPLEMENTATION}" == "rmw_fastrtps_cpp" ]]; then
    : "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
    export FASTDDS_BUILTIN_TRANSPORTS
fi

# Deprecation shim: ENABLE_VIEWER=1 was replaced by VIEWER=gl in Phase 7.
if [[ "${ENABLE_VIEWER:-0}" != "0" ]]; then
    echo "[run.sh] ERROR: ENABLE_VIEWER is deprecated. Use VIEWER=gl (or rerun/usd/file/null/none)." >&2
    exit 2
fi
# UID/GID are readonly in bash; use HOST_UID/HOST_GID for docker-compose.
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

MODE="${1:-sim}"
shift || true

case "${MODE}" in
    sim|up)
        exec "${COMPOSE[@]}" run --rm --service-ports newton-bridge \
            python3 -m newton_bridge "$@"
        ;;
    upd)
        exec "${COMPOSE[@]}" up -d
        ;;
    shell)
        # Start the service if not already running, then drop into bash.
        "${COMPOSE[@]}" up -d
        exec "${COMPOSE[@]}" exec newton-bridge bash
        ;;
    example)
        if [[ $# -lt 1 ]]; then
            echo "usage: $0 example <name> [--viewer gl|usd] [extra args...]" >&2
            echo "  e.g. $0 example basic_pendulum --viewer gl" >&2
            exit 1
        fi
        exec "${COMPOSE[@]}" run --rm --service-ports newton-bridge \
            python3 -m newton.examples "$@"
        ;;
    jupyter)
        exec "${COMPOSE[@]}" run --rm --service-ports newton-bridge \
            jupyter notebook \
                --ip=0.0.0.0 --port=8888 --no-browser --allow-root \
                --ServerApp.token="${JUPYTER_TOKEN:-newton}" \
                --notebook-dir=/workspace/workspace/notebooks
        ;;
    verify)
        exec "${COMPOSE[@]}" run --rm newton-bridge \
            bash /workspace/scripts/verify.sh
        ;;
    logs)
        exec "${COMPOSE[@]}" logs -f
        ;;
    down)
        exec "${COMPOSE[@]}" down
        ;;
    *)
        cat >&2 <<EOF
usage: $0 [sim|shell|example <name>|jupyter|verify|upd|logs|down]

env overrides:
  ROBOT=<name>          robots/<name>/ pack to load (default: ur5e)
  EXTERNAL_PACK_HOST=<path>  host folder to use as the pack (URDF only).
                             The folder must contain robot.yaml; meshes resolve
                             relative to the URDF. See docs/ROBOTS.md (Path D).
  SYNC_MODE=freerun|sync
  FREERUN_RATE=realtime|max
  ROS_DOMAIN_ID=<n>     match this to your host
  VIEWER=rerun|gl|usd|file|null|none  (default: rerun — web viewer at http://localhost:9090)
EOF
        exit 1
        ;;
esac
