#!/usr/bin/env bash
# Build the newton-bridge Docker image.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

NO_CACHE=""
if [[ "${1:-}" == "--no-cache" ]]; then
    NO_CACHE="--no-cache"
fi

export UID="${UID:-$(id -u)}"
export GID="${GID:-$(id -g)}"

docker compose build ${NO_CACHE}

echo
echo "[build] done. Next:"
echo "  cp .env.example .env   # (optional) tune defaults"
echo "  ./scripts/fetch_assets.sh   # download robot meshes/MJCF"
echo "  ./run.sh verify        # smoke-test the image"
