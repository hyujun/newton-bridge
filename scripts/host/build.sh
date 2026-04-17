#!/usr/bin/env bash
# Build the newton-bridge Docker image.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

NO_CACHE=""
if [[ "${1:-}" == "--no-cache" ]]; then
    NO_CACHE="--no-cache"
fi

# UID/GID are readonly in bash; use HOST_UID/HOST_GID for docker-compose.
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

docker compose -f docker/compose.yml build ${NO_CACHE}

echo
echo "[build] done. Next:"
echo "  cp .env.example .env                 # (optional) tune defaults"
echo "  ./scripts/host/fetch_assets.sh       # download robot meshes/MJCF"
echo "  ./scripts/host/run.sh verify         # smoke-test the image"
