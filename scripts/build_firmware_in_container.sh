#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUSEFI_DIR="${ROOT_DIR}/ext/rusefi"
META_INFO="${ROOT_DIR}/meta-info.env"

MODE="full"
DFU_MODE="auto"

while [ $# -gt 0 ]; do
  case "$1" in
    --bin-only) MODE="bin-only" ;;
    --bin-hex) MODE="bin-hex" ;;
    --no-dfu) DFU_MODE="skip" ;;
    --with-dfu) DFU_MODE="force" ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

if [ "${DFU_MODE}" = "auto" ]; then
  ARCH="$(uname -m)"
  case "${ARCH}" in
    arm64|aarch64) DFU_MODE="skip" ;;
    *) DFU_MODE="force" ;;
  esac
fi

ARTIFACTS=()
case "${MODE}" in
  full)
    ARTIFACTS=("hex" "list" "map" "elf")
    if [ "${DFU_MODE}" = "force" ]; then
      ARTIFACTS+=("bin" "srec" "bundle" "autoupdate")
    fi
    ;;
  bin-only)
    if [ "${DFU_MODE}" = "force" ]; then
      ARTIFACTS=("bin")
    else
      ARTIFACTS=("elf")
    fi
    ;;
  bin-hex)
    if [ "${DFU_MODE}" = "force" ]; then
      ARTIFACTS=("bin" "hex")
    else
      ARTIFACTS=("hex")
    fi
    ;;
esac

cd "${RUSEFI_DIR}"
misc/git_scripts/common_submodule_init.sh

cd "${RUSEFI_DIR}/firmware"
bash bin/compile.sh "${META_INFO}" config
bash bin/compile.sh "${META_INFO}" --output-sync=recurse "${ARTIFACTS[@]}"

if [ "${DFU_MODE}" = "skip" ]; then
  ELF_PATH="${RUSEFI_DIR}/firmware/build/rusefi.elf"
  BIN_PATH="${RUSEFI_DIR}/firmware/build/rusefi.bin"
  if [ -f "${ELF_PATH}" ] && [ ! -f "${BIN_PATH}" ]; then
    arm-none-eabi-objcopy -O binary "${ELF_PATH}" "${BIN_PATH}"
  fi
  if [ -f "${BIN_PATH}" ]; then
    mkdir -p "${RUSEFI_DIR}/firmware/deliver"
    cp -f "${BIN_PATH}" "${RUSEFI_DIR}/firmware/deliver/rusefi.bin"
  fi
fi

SHORT_BOARD_NAME="$(awk -F= '$1=="SHORT_BOARD_NAME"{print $2; exit}' "${META_INFO}" | tr -d '\r')"
if [ -n "${SHORT_BOARD_NAME}" ]; then
  INI_SRC="${RUSEFI_DIR}/firmware/tunerstudio/generated/rusefi_${SHORT_BOARD_NAME}.ini"
  if [ -f "${INI_SRC}" ]; then
    cp -f "${INI_SRC}" "${RUSEFI_DIR}/firmware/build/"
  fi
fi
