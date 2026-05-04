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

# -- pack manifest check ------------------------------------------------------
# robot.yaml is the only git-tracked file inside each pack — if it's missing,
# the pack is broken and this script can't fix it (we only populate models/).
# Fail-fast with a precise message so users don't waste time blaming this
# script for a deletion it didn't cause.
for pack in franka kuka_iiwa_14 ur5e; do
    if [[ ! -f "robots/${pack}/robot.yaml" ]]; then
        die "robots/${pack}/robot.yaml is missing.
  This file is git-tracked and was NOT touched by this script.
  Likely cause: an earlier checkout / restore / manual edit removed it.
  Recover with:  git restore robots/${pack}/robot.yaml
  (or re-clone the repo to start fresh)"
    fi
done

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
# Goal: pack must be self-contained so the runtime never touches /opt/ros.
# We copy urdf/ + meshes/ur5e/{visual,collision} + config/ur5e from the host's
# ur_description share, then patch every `$(find ur_description)/...` reference
# in the xacro files to a robot-pack-relative path. xacro_loader.py later
# rewrites the `file://<ROS-share>/meshes/...` URIs that `force_abs_paths=true`
# produces (see robots/ur5e/robot.yaml) to point at this same pack tree.
log "populating robots/ur5e/models"
mkdir -p robots/ur5e/models
if ! dpkg -s ros-jazzy-ur-description >/dev/null 2>&1; then
    die "ros-jazzy-ur-description not installed.
  install it with:
    ./scripts/host/install.sh"
fi
UR_SHARE="/opt/ros/jazzy/share/ur_description"
if command -v ros2 >/dev/null 2>&1; then
    UR_SHARE="$(ros2 pkg prefix ur_description 2>/dev/null || echo /opt/ros/jazzy)/share/ur_description"
fi
if [[ ! -d "${UR_SHARE}" ]]; then
    die "ur_description installed but share path not found at ${UR_SHARE}"
fi
log "  source: apt ros-jazzy-ur-description (${UR_SHARE})"

# urdf/ (xacro files) — these will get patched below
rsync -a --delete "${UR_SHARE}/urdf/" robots/ur5e/models/

# meshes/ur5e/{visual,collision} — fail loud, runtime mesh loader needs these
[[ -d "${UR_SHARE}/meshes/ur5e/visual" ]] \
    || die "missing ${UR_SHARE}/meshes/ur5e/visual — reinstall ros-jazzy-ur-description"
[[ -d "${UR_SHARE}/meshes/ur5e/collision" ]] \
    || die "missing ${UR_SHARE}/meshes/ur5e/collision — reinstall ros-jazzy-ur-description"
mkdir -p robots/ur5e/models/meshes/ur5e
rsync -a --delete "${UR_SHARE}/meshes/ur5e/visual/"    robots/ur5e/models/meshes/ur5e/visual/
rsync -a --delete "${UR_SHARE}/meshes/ur5e/collision/" robots/ur5e/models/meshes/ur5e/collision/

# config/ur5e/*.yaml + config/initial_positions.yaml — read by the xacro at
# process time (joint_limits, kinematics, physical, visual). Must live inside
# the pack so the patched xacro can resolve them without ROS.
[[ -d "${UR_SHARE}/config/ur5e" ]] \
    || die "missing ${UR_SHARE}/config/ur5e — reinstall ros-jazzy-ur-description"
mkdir -p robots/ur5e/models/config
rsync -a --delete "${UR_SHARE}/config/ur5e/" robots/ur5e/models/config/ur5e/
[[ -f "${UR_SHARE}/config/initial_positions.yaml" ]] \
    && cp "${UR_SHARE}/config/initial_positions.yaml" robots/ur5e/models/config/initial_positions.yaml

# Replace every `$(find ur_description)` with a sentinel string. xacro_loader
# substitutes the sentinel for the current pack's models/ dir before invoking
# xacro, so includes, config-yaml refs, AND the `force_abs_paths`-generated
# `file://<...>/meshes/...` URIs all resolve to this pack — never /opt/ros.
# Why a sentinel and not a literal path? Host and container see the pack at
# different absolute paths (e.g. .../newton-bridge/robots vs /workspace/robots),
# so resolution has to happen at xacro process time, not now.
log "  patching \$(find ur_description) refs in pack xacro files"
for f in $(grep -rl 'find ur_description' robots/ur5e/models/ 2>/dev/null); do
    sed -i "s|\$(find ur_description)|@@NEWTON_BRIDGE_PACK_DIR@@|g" "${f}"
done

log "done. Verify tree:"
log "  robots/ur5e/models/*.urdf"
log "  robots/franka/models/panda.xml"
log "  robots/kuka_iiwa_14/models/iiwa14.xml"
