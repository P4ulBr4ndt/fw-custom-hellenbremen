#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE_NAME="${IMAGE_NAME:-rusefi-fw}"
DOCKERFILE="${DOCKERFILE:-${ROOT_DIR}/Dockerfile.rusefi}"
PLATFORM="${PLATFORM:-}"
EXTRA_ARGS=("$@")

BUILD_ARGS=("-f" "${DOCKERFILE}" "-t" "${IMAGE_NAME}" "${ROOT_DIR}")
RUN_ARGS=("--rm" "-it" "-v" "${ROOT_DIR}:/work" "-w" "/work" "${IMAGE_NAME}" "./scripts/build_firmware_in_container.sh")

if [ -n "${PLATFORM}" ]; then
  BUILD_ARGS=("--platform" "${PLATFORM}" "${BUILD_ARGS[@]}")
  RUN_ARGS=("--platform" "${PLATFORM}" "${RUN_ARGS[@]}")
fi

docker build "${BUILD_ARGS[@]}"
docker run "${RUN_ARGS[@]}" "${EXTRA_ARGS[@]}"
