#!/usr/bin/env bash
# Pull robot assets into robots/<name>/models/ from upstream sources.
#   - UR5e URDF:        sibling sim-bridge repo, then fallback to ur_description apt pkg
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
SIMBRIDGE_UR5E="${REPO_ROOT}/../sim-bridge/robots/ur5e"
if [[ -d "${SIMBRIDGE_UR5E}/urdf" ]]; then
    log "  source: sibling sim-bridge repo"
    rsync -a --delete "${SIMBRIDGE_UR5E}/urdf/" robots/ur5e/models/
    if [[ -d "${SIMBRIDGE_UR5E}/meshes" ]]; then
        rsync -a --delete "${SIMBRIDGE_UR5E}/meshes/" robots/ur5e/models/meshes/
    fi
else
    warn "sim-bridge/robots/ur5e not found — trying ur_description from apt"
    if dpkg -s ros-jazzy-ur-description >/dev/null 2>&1; then
        UR_SHARE="$(ros2 pkg prefix ur_description 2>/dev/null || echo "")/share/ur_description"
        if [[ -d "${UR_SHARE}" ]]; then
            rsync -a "${UR_SHARE}/urdf/" robots/ur5e/models/
            rsync -a "${UR_SHARE}/meshes/ur5e/" robots/ur5e/models/meshes/ur5e/ 2>/dev/null || true
        else
            die "ur_description not on share path — copy urdf/ + meshes/ manually"
        fi
    else
        die "no UR5e source: neither ../sim-bridge nor apt ros-jazzy-ur-description"
    fi
fi

log "done. Verify tree:"
log "  robots/ur5e/models/*.urdf"
log "  robots/franka/models/panda.xml"
log "  robots/kuka_iiwa_14/models/iiwa14.xml"
