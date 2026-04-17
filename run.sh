#!/usr/bin/env bash
# Launch / manage the newton-bridge container.
#
# Usage:
#   ./run.sh                              # up (foreground, default robot=ur5e, freerun)
#   ROBOT=franka ./run.sh                 # pick pack
#   SYNC_MODE=handshake ./run.sh          # pick sync mode
#   ./run.sh sim                          # same as default up
#   ./run.sh shell                        # interactive bash
#   ./run.sh example basic_pendulum       # run a Newton example (viewer gl)
#   ./run.sh jupyter                      # start Jupyter on host:8888
#   ./run.sh verify                       # in-container smoke test
#   ./run.sh down                         # stop + remove container
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

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
export ROBOT_PACK SYNC_MODE FREERUN_RATE ROS_DOMAIN_ID RMW_IMPLEMENTATION
export UID="${UID:-$(id -u)}"
export GID="${GID:-$(id -g)}"

MODE="${1:-sim}"
shift || true

case "${MODE}" in
    sim|up)
        exec docker compose run --rm --service-ports newton-bridge \
            python3 /workspace/sim_node.py "$@"
        ;;
    upd)
        exec docker compose up -d
        ;;
    shell)
        # Start the service if not already running, then drop into bash.
        docker compose up -d
        exec docker compose exec newton-bridge bash
        ;;
    example)
        if [[ $# -lt 1 ]]; then
            echo "usage: $0 example <name> [--viewer gl|usd] [extra args...]" >&2
            echo "  e.g. $0 example basic_pendulum --viewer gl" >&2
            exit 1
        fi
        exec docker compose run --rm --service-ports newton-bridge \
            python3 -m newton.examples "$@"
        ;;
    jupyter)
        exec docker compose run --rm --service-ports newton-bridge \
            jupyter notebook \
                --ip=0.0.0.0 --port=8888 --no-browser --allow-root \
                --ServerApp.token="${JUPYTER_TOKEN:-newton}" \
                --notebook-dir=/workspace/workspace/notebooks
        ;;
    verify)
        exec docker compose run --rm newton-bridge \
            bash /workspace/scripts/verify.sh
        ;;
    logs)
        exec docker compose logs -f
        ;;
    down)
        exec docker compose down
        ;;
    *)
        cat >&2 <<EOF
usage: $0 [sim|shell|example <name>|jupyter|verify|upd|logs|down]

env overrides:
  ROBOT=<name>          robots/<name>/ pack to load (default: ur5e)
  SYNC_MODE=freerun|handshake
  FREERUN_RATE=realtime|max
  ROS_DOMAIN_ID=<n>     match this to your host
EOF
        exit 1
        ;;
esac
