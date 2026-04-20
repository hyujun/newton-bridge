#!/usr/bin/env bash
# Pull robot assets into robots/<name>/models/ from upstream sources.
#   - UR5e URDF:        ur_description apt pkg (ros-jazzy-ur-description)
#   - franka MJCF:      mujoco_menagerie (shallow clone, pinned main)
#   - kuka_iiwa_14 MJCF: mujoco_menagerie (shared clone)
#
# Safe to re-run. Destination dirs are gitignored; only robot.yaml is tracked.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

log()  { printf '\033[1;34m[fetch]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[fetch]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[fetch]\033[0m %s\n' "$*" >&2; exit 1; }

# -- prereq check -------------------------------------------------------------
missing=()
for cmd in git rsync; do
    command -v "${cmd}" >/dev/null 2>&1 || missing+=("${cmd}")
done
if [[ ${#missing[@]} -gt 0 ]]; then
    die "missing host utilities: ${missing[*]}
  run ./scripts/host/install.sh first (it installs git, rsync, etc.)"
fi

CACHE="${REPO_ROOT}/assets/_cache"
mkdir -p "${CACHE}"

# -- 1) mujoco_menagerie (shared for franka + kuka_iiwa_14) -------------------
MENAGERIE="${CACHE}/mujoco_menagerie"
if [[ ! -d "${MENAGERIE}/.git" ]]; then
    log "cloning mujoco_menagerie (shallow)"
    git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git \
        "${MENAGERIE}"
else
    log "mujoco_menagerie cache present — skipping clone"
fi

# -- 2) franka ----------------------------------------------------------------
log "populating robots/franka/models"
mkdir -p robots/franka/models
rm -rf robots/franka/models/*
cp -r "${MENAGERIE}/franka_emika_panda/"*.xml robots/franka/models/
cp -r "${MENAGERIE}/franka_emika_panda/assets" robots/franka/models/ 2>/dev/null || true

# -- 3) kuka_iiwa_14 ----------------------------------------------------------
log "populating robots/kuka_iiwa_14/models"
mkdir -p robots/kuka_iiwa_14/models
rm -rf robots/kuka_iiwa_14/models/*
cp -r "${MENAGERIE}/kuka_iiwa_14/"*.xml robots/kuka_iiwa_14/models/
cp -r "${MENAGERIE}/kuka_iiwa_14/assets" robots/kuka_iiwa_14/models/ 2>/dev/null || true

# -- 4) ur5e ------------------------------------------------------------------
log "populating robots/ur5e/models"
mkdir -p robots/ur5e/models
if ! dpkg -s ros-jazzy-ur-description >/dev/null 2>&1; then
    die "ros-jazzy-ur-description not installed.
  install it with:
    ./scripts/host/install.sh --with-ros"
fi
UR_SHARE="/opt/ros/jazzy/share/ur_description"
if command -v ros2 >/dev/null 2>&1; then
    UR_SHARE="$(ros2 pkg prefix ur_description 2>/dev/null || echo /opt/ros/jazzy)/share/ur_description"
fi
if [[ ! -d "${UR_SHARE}" ]]; then
    die "ur_description installed but share path not found at ${UR_SHARE}
  copy urdf/ + meshes/ manually into robots/ur5e/models/"
fi
log "  source: apt ros-jazzy-ur-description (${UR_SHARE})"
rsync -a --delete "${UR_SHARE}/urdf/" robots/ur5e/models/
rsync -a --delete "${UR_SHARE}/meshes/ur5e/" robots/ur5e/models/meshes/ur5e/ 2>/dev/null || true

log "done. Verify tree:"
log "  robots/ur5e/models/*.urdf"
log "  robots/franka/models/panda.xml"
log "  robots/kuka_iiwa_14/models/iiwa14.xml"
