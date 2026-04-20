#!/usr/bin/env bash
# Install host prerequisites for running newton-bridge in Docker:
#   0. Base utilities   (git, rsync, curl, jq, x11-xserver-utils, ...)
#   1. Docker Engine    (apt repo: download.docker.com)
#   2. docker compose v2 (docker-compose-plugin)
#   3. NVIDIA Container Toolkit  (only if an NVIDIA GPU is detected)
#   4. Add current user to the 'docker' group
#   5. (optional) ROS 2 Jazzy Desktop + ur_description  (--with-ros)
#
# Detects — but does NOT install — the NVIDIA driver. On fresh 24.04 you must
# install a driver first (`sudo ubuntu-drivers autoinstall` then reboot) for
# nvidia-smi / GPU passthrough to work.
#
# Does NOT fetch assets, build the image, or run smoke tests — those are
# separate steps (scripts/host/fetch_assets.sh, scripts/host/build.sh,
# scripts/host/run.sh verify).
#
# Targets Ubuntu 22.04 / 24.04. Idempotent: re-running skips already-installed
# components. Requires sudo.
#
# Usage:
#   ./install.sh                   # docker + compose (+ nvidia toolkit if GPU)
#   ./install.sh --with-ros        # also install ROS 2 Jazzy Desktop on host
#   ./install.sh --no-nvidia       # skip nvidia-container-toolkit even if GPU
#   ./install.sh --only-check      # report what is installed, no changes
set -euo pipefail

log()  { printf '\033[1;34m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

# -- flags --------------------------------------------------------------------
NO_NVIDIA=0
ONLY_CHECK=0
WITH_ROS=0

for arg in "$@"; do
    case "${arg}" in
        --no-nvidia)    NO_NVIDIA=1 ;;
        --only-check)   ONLY_CHECK=1 ;;
        --with-ros)     WITH_ROS=1 ;;
        -h|--help)      sed -n '2,25p' "$0"; exit 0 ;;
        *) die "unknown arg: ${arg}" ;;
    esac
done

[[ "$(uname -s)" == "Linux" ]] || die "this installer only supports Linux"

if [[ ! -r /etc/os-release ]]; then
    die "/etc/os-release missing — unsupported distro"
fi
# shellcheck disable=SC1091
. /etc/os-release
if [[ "${ID:-}" != "ubuntu" ]]; then
    warn "untested distro '${ID:-unknown}' — proceeding, but apt repo URLs assume Ubuntu"
fi
UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-noble}}"

SUDO="sudo"
[[ $EUID -eq 0 ]] && SUDO=""

BASE_PKGS=(git rsync ca-certificates curl gnupg lsb-release x11-xserver-utils jq)

has_docker()        { command -v docker >/dev/null 2>&1; }
has_compose_v2()    { docker compose version >/dev/null 2>&1; }
has_nvidia_gpu()    { command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; }
has_nvidia_runtime(){ docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; }
has_ros_jazzy()     { [[ -f /opt/ros/jazzy/setup.bash ]] && dpkg -s ros-jazzy-desktop >/dev/null 2>&1; }
needs_base_utils()  {
    local p
    for p in "${BASE_PKGS[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || return 0
    done
    return 1
}

# -- status summary -----------------------------------------------------------
log "current state:"
log "  base utilities:          $(needs_base_utils && echo missing || echo ok)"
log "  docker:                  $(has_docker        && echo ok || echo missing)"
log "  docker compose v2:       $(has_compose_v2    && echo ok || echo missing)"
log "  nvidia GPU (nvidia-smi): $(has_nvidia_gpu    && echo present || echo none)"
if has_docker; then
    log "  nvidia docker runtime:   $(has_nvidia_runtime && echo ok || echo missing)"
fi
log "  ROS 2 Jazzy Desktop:     $(has_ros_jazzy && echo ok || echo missing)"

if [[ "${ONLY_CHECK}" -eq 1 ]]; then
    log "check only — exiting"
    exit 0
fi

# -- 0) base utilities --------------------------------------------------------
# Needed by fetch_assets.sh (git, rsync), run.sh X11 passthrough (xhost), and
# the docker/ROS apt repo setup below (curl, gnupg, ca-certificates, lsb-release).
if needs_base_utils; then
    log "installing base utilities: ${BASE_PKGS[*]}"
    ${SUDO} apt-get update
    ${SUDO} apt-get install -y --no-install-recommends "${BASE_PKGS[@]}"
else
    log "base utilities already installed — skipping"
fi

# -- 1) Docker Engine + compose plugin ---------------------------------------
if has_docker && has_compose_v2; then
    log "docker + compose v2 already installed — skipping"
else
    log "installing docker engine + compose v2 plugin"

    ${SUDO} install -m 0755 -d /etc/apt/keyrings
    if [[ ! -f /etc/apt/keyrings/docker.gpg ]]; then
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
            | ${SUDO} gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        ${SUDO} chmod a+r /etc/apt/keyrings/docker.gpg
    fi

    if [[ ! -f /etc/apt/sources.list.d/docker.list ]]; then
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME} stable" \
            | ${SUDO} tee /etc/apt/sources.list.d/docker.list > /dev/null
    fi

    ${SUDO} apt-get update
    ${SUDO} apt-get install -y --no-install-recommends \
        docker-ce docker-ce-cli containerd.io \
        docker-buildx-plugin docker-compose-plugin
fi

# -- 2) docker group membership ----------------------------------------------
if getent group docker >/dev/null 2>&1; then
    if id -nG "${USER}" | tr ' ' '\n' | grep -qx docker; then
        log "user '${USER}' already in docker group"
    else
        log "adding user '${USER}' to docker group (re-login required to take effect)"
        ${SUDO} usermod -aG docker "${USER}"
    fi
fi

# -- 3) NVIDIA Container Toolkit ---------------------------------------------
if [[ "${NO_NVIDIA}" -eq 1 ]]; then
    log "skipping nvidia-container-toolkit (--no-nvidia)"
elif ! has_nvidia_gpu; then
    warn "no NVIDIA driver detected (nvidia-smi missing or failing)."
    warn "  This script does NOT install the driver. For GPU passthrough:"
    warn "    sudo ubuntu-drivers autoinstall   # recommended on Ubuntu 24.04"
    warn "    sudo reboot"
    warn "  Then re-run this script to install nvidia-container-toolkit."
elif has_nvidia_runtime; then
    log "nvidia docker runtime already configured — skipping toolkit install"
else
    DRIVER_VER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
    log "NVIDIA driver detected (${DRIVER_VER:-?}) — installing nvidia-container-toolkit"

    if [[ ! -f /etc/apt/keyrings/nvidia-container-toolkit.gpg ]]; then
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
            | ${SUDO} gpg --dearmor -o /etc/apt/keyrings/nvidia-container-toolkit.gpg
    fi
    if [[ ! -f /etc/apt/sources.list.d/nvidia-container-toolkit.list ]]; then
        curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
            | sed 's#deb https://#deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://#g' \
            | ${SUDO} tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
    fi

    ${SUDO} apt-get update
    ${SUDO} apt-get install -y --no-install-recommends nvidia-container-toolkit

    log "registering nvidia runtime with docker daemon"
    ${SUDO} nvidia-ctk runtime configure --runtime=docker
    ${SUDO} systemctl restart docker
fi

# -- 4) (optional) ROS 2 Jazzy Desktop ---------------------------------------
if [[ "${WITH_ROS}" -eq 1 ]]; then
    if has_ros_jazzy && dpkg -s ros-jazzy-ur-description >/dev/null 2>&1; then
        log "ROS 2 Jazzy Desktop + ur_description already installed — skipping"
    else
        log "installing ROS 2 Jazzy Desktop + ur_description"

        if ! locale -a 2>/dev/null | grep -qi 'en_US\.utf8'; then
            ${SUDO} apt-get install -y --no-install-recommends locales
            ${SUDO} locale-gen en_US en_US.UTF-8
            ${SUDO} update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        fi

        ${SUDO} apt-get install -y --no-install-recommends software-properties-common
        ${SUDO} add-apt-repository -y universe

        if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
            ${SUDO} curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
                https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
        fi
        if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
                | ${SUDO} tee /etc/apt/sources.list.d/ros2.list > /dev/null
        fi

        ${SUDO} apt-get update
        ${SUDO} apt-get install -y --no-install-recommends \
            ros-jazzy-desktop ros-jazzy-ur-description
    fi
fi

# -- done ---------------------------------------------------------------------
cat <<'EOF'

[install] host prerequisites installed. Next:
  # if you were just added to the 'docker' group, log out and back in first
  # if nvidia-smi is missing, install the driver then reboot:
  #     sudo ubuntu-drivers autoinstall && sudo reboot
  #     (then re-run this script to pick up nvidia-container-toolkit)

  ./scripts/host/fetch_assets.sh   # download URDF / MJCF robot assets
  ./scripts/host/build.sh          # build the newton-bridge image (5-15 min)
  ./scripts/host/run.sh verify     # in-container smoke test
  ./scripts/host/run.sh sim        # start the sim (ROBOT=ur5e, freerun)
EOF
