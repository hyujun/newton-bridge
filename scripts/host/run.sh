#!/usr/bin/env bash
# Launch / manage the newton-bridge container.
#
# Usage:
#   ./scripts/host/run.sh                              # up (foreground, robot=ur5e, freerun)
#   ROBOT=franka ./scripts/host/run.sh                 # pick pack
#   SYNC_MODE=handshake ./scripts/host/run.sh          # pick sync mode
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
: "${ROBOT:=ur5e}"
: "${ROBOT_PACK:=/workspace/robots/${ROBOT}}"
: "${SYNC_MODE:=freerun}"
: "${FREERUN_RATE:=realtime}"
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
: "${VIEWER:=rerun}"
export ROBOT_PACK SYNC_MODE FREERUN_RATE ROS_DOMAIN_ID RMW_IMPLEMENTATION VIEWER

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
  SYNC_MODE=freerun|handshake
  FREERUN_RATE=realtime|max
  ROS_DOMAIN_ID=<n>     match this to your host
  VIEWER=rerun|gl|usd|file|null|none  (default: rerun — web viewer at http://localhost:9090)
EOF
        exit 1
        ;;
esac
