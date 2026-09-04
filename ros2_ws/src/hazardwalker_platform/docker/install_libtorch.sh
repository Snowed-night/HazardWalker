#!/usr/bin/env bash
# Install LibTorch (cxx11 ABI) at the path expected by unitree_guide CMakeLists.txt.
set -euo pipefail

LIBTORCH_VERSION="${LIBTORCH_VERSION:-2.0.1}"
# cpu | cu118 (CUDA 11.8 — matches platform cuda-11.6 builds; driver 535+ on hxbl)
LIBTORCH_CUDA="${LIBTORCH_CUDA:-cu118}"
LIBTORCH_DIR="${LIBTORCH_DIR:-/home/ros/Guoyulun/Download/libtorch}"
VARIANT_MARKER="${LIBTORCH_DIR}/.libtorch_variant"

if [[ "${LIBTORCH_CUDA}" == "cpu" ]]; then
  LIBTORCH_URL="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcpu.zip"
  variant_label="cpu"
else
  LIBTORCH_URL="https://download.pytorch.org/libtorch/${LIBTORCH_CUDA}/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2B${LIBTORCH_CUDA}.zip"
  variant_label="${LIBTORCH_CUDA}"
fi

if [[ -f "${LIBTORCH_DIR}/share/cmake/Torch/TorchConfig.cmake" ]] \
    && [[ -f "${VARIANT_MARKER}" ]] \
    && [[ "$(cat "${VARIANT_MARKER}")" == "${variant_label}" ]]; then
  echo "LibTorch ${variant_label} already installed at ${LIBTORCH_DIR}"
  exit 0
fi

if [[ -d "${LIBTORCH_DIR}" ]]; then
  echo "Replacing existing LibTorch at ${LIBTORCH_DIR}"
  rm -rf "${LIBTORCH_DIR}"
fi

apt-get update -qq
apt-get install -y -qq --no-install-recommends wget ca-certificates curl unzip
rm -rf /var/lib/apt/lists/*

parent_dir="$(dirname "${LIBTORCH_DIR}")"
mkdir -p "${parent_dir}"

tmp_zip="$(mktemp /tmp/libtorch.XXXXXX.zip)"
trap 'rm -f "${tmp_zip}"' EXIT

echo "Downloading LibTorch ${LIBTORCH_VERSION}+${variant_label} -> ${LIBTORCH_DIR}"
if command -v curl >/dev/null 2>&1; then
  curl -fsSL --retry 3 --retry-delay 5 -o "${tmp_zip}" "${LIBTORCH_URL}"
elif command -v wget >/dev/null 2>&1; then
  wget -O "${tmp_zip}" "${LIBTORCH_URL}"
else
  echo "ERROR: need curl or wget" >&2
  exit 1
fi
unzip -q "${tmp_zip}" -d "${parent_dir}"
rm -f "${tmp_zip}"
trap - EXIT

if [[ ! -f "${LIBTORCH_DIR}/share/cmake/Torch/TorchConfig.cmake" ]]; then
  echo "ERROR: TorchConfig.cmake missing after extract" >&2
  exit 1
fi

echo "${variant_label}" > "${VARIANT_MARKER}"
echo "LibTorch OK: ${LIBTORCH_DIR} (${variant_label})"
