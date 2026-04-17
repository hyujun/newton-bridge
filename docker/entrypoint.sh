#!/usr/bin/env bash
# Container entrypoint: layer ROS 2 env on top of the Newton venv, then exec.
set -euo pipefail

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

exec "$@"
