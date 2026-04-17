#!/usr/bin/env bash
# Container entrypoint: layer ROS 2 env on top of the Newton venv, then exec.
set -euo pipefail

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # ROS setup scripts reference AMENT_TRACE_SETUP_FILES etc. without defaults,
    # which trips `set -u`. Disable nounset just around the source.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
fi

exec "$@"
