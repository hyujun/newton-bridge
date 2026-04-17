#!/usr/bin/env bash
# Install host prerequisites for running newton-bridge in Docker:
#   1. Docker Engine      (apt repo: download.docker.com)
#   2. docker compose v2  (docker-compose-plugin)
#   3. NVIDIA Container Toolkit  (only if an NVIDIA GPU is detected)
#   4. Add current user to the 'docker' group
#
# Does NOT fetch assets, build the image, or run smoke tests — those are
# separate steps (./scripts/fetch_assets.sh, ./build.sh, ./run.sh verify).
#
# Targets Ubuntu 22.04 / 24.04. Idempotent: re-running skips already-installed
# components. Requires sudo.
#
# Usage:
#   ./install.sh                   # full install (docker + compose + nvidia toolkit if GPU)
#   ./install.sh --no-nvidia       # skip nvidia-container-toolkit even if GPU present
#   ./install.sh --only-check      # check what is installed, no changes
set -euo pipefail

log()  { printf '\033[1;34m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

# -- flags --------------------------------------------------------------------
NO_NVIDIA=0
ONLY_CHECK=0

for arg in "$@"; do
    case "${arg}" in
        --no-nvidia)    NO_NVIDIA=1 ;;
        --only-check)   ONLY_CHECK=1 ;;
        -h|--help)      sed -n '2,17p' "$0"; exit 0 ;;
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

has_docker()        { command -v docker >/dev/null 2>&1; }
has_compose_v2()    { docker compose version >/dev/null 2>&1; }
has_nvidia_gpu()    { command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; }
has_nvidia_runtime(){ docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; }

# -- status summary -----------------------------------------------------------
log "current state:"
log "  docker:                  $(has_docker        && echo ok || echo missing)"
log "  docker compose v2:       $(has_compose_v2    && echo ok || echo missing)"
log "  nvidia GPU (nvidia-smi): $(has_nvidia_gpu    && echo present || echo none)"
if has_docker; then
    log "  nvidia docker runtime:   $(has_nvidia_runtime && echo ok || echo missing)"
fi

if [[ "${ONLY_CHECK}" -eq 1 ]]; then
    log "check only — exiting"
    exit 0
fi

# -- 1) Docker Engine + compose plugin ---------------------------------------
if has_docker && has_compose_v2; then
    log "docker + compose v2 already installed — skipping"
else
    log "installing docker engine + compose v2 plugin"
    ${SUDO} apt-get update
    ${SUDO} apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release

    ${SUDO} install -m 0755 -d /etc/apt/keyrings
    if [[ ! -f /etc/apt/keyrings/docker.gpg ]]; then
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
            | ${SUDO} gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        ${SUDO} chmod a+r /etc/apt/keyrings/docker.gpg
    fi

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME} stable" \
        | ${SUDO} tee /etc/apt/sources.list.d/docker.list > /dev/null

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
    warn "no NVIDIA GPU detected — skipping nvidia-container-toolkit"
    warn "  (install the NVIDIA driver first if you want GPU access inside the container)"
elif has_nvidia_runtime; then
    log "nvidia docker runtime already configured — skipping toolkit install"
else
    log "installing nvidia-container-toolkit"
    if [[ ! -f /etc/apt/keyrings/nvidia-container-toolkit.gpg ]]; then
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
            | ${SUDO} gpg --dearmor -o /etc/apt/keyrings/nvidia-container-toolkit.gpg
    fi
    curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://#g' \
        | ${SUDO} tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

    ${SUDO} apt-get update
    ${SUDO} apt-get install -y --no-install-recommends nvidia-container-toolkit

    log "registering nvidia runtime with docker daemon"
    ${SUDO} nvidia-ctk runtime configure --runtime=docker
    ${SUDO} systemctl restart docker
fi

# -- done ---------------------------------------------------------------------
cat <<'EOF'

[install] host prerequisites installed. Next:
  # if you were just added to the 'docker' group, log out and back in first

  ./scripts/fetch_assets.sh   # download URDF / MJCF robot assets
  ./build.sh                  # build the newton-bridge image (5-15 min)
  ./run.sh verify             # in-container smoke test
  ./run.sh sim                # start the sim (ROBOT=ur5e, freerun)
EOF
