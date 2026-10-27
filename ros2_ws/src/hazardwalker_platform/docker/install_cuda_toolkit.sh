#!/usr/bin/env bash
# CUDA 11.8 toolkit headers/nvcc — required for LibTorch cu118 find_package(Torch) at compile time.
# GPU passthrough alone does not install nvcc inside the container.
set -euo pipefail

CUDA_MAJOR="${CUDA_MAJOR:-11}"
CUDA_MINOR="${CUDA_MINOR:-8}"
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-${CUDA_MAJOR}.${CUDA_MINOR}}"

if [[ -x "${CUDA_HOME}/bin/nvcc" ]]; then
  echo "CUDA toolkit already at ${CUDA_HOME}"
  exit 0
fi

apt-get update -qq
apt-get install -y -qq --no-install-recommends wget ca-certificates gnupg
wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i cuda-keyring_1.1-1_all.deb
apt-get update -qq
apt-get install -y -qq --no-install-recommends "cuda-toolkit-${CUDA_MAJOR}-${CUDA_MINOR}"
rm -f cuda-keyring_1.1-1_all.deb
rm -rf /var/lib/apt/lists/*

apt-get install -y -qq --no-install-recommends "cuda-toolkit-${CUDA_MAJOR}-${CUDA_MINOR}"
rm -f cuda-keyring_1.1-1_all.deb
rm -rf /var/lib/apt/lists/*

CUDA_HOME="/usr/local/cuda-${CUDA_MAJOR}.${CUDA_MINOR}"
if [[ ! -x "${CUDA_HOME}/bin/nvcc" ]]; then
  echo "ERROR: nvcc not found at ${CUDA_HOME}/bin/nvcc" >&2
  find /usr/local -maxdepth 4 -name nvcc 2>/dev/null >&2 || true
  exit 1
fi
# cuda-toolkit deb already manages /usr/local/cuda -> cuda-11.8; do not re-link (causes symlink loop).
"${CUDA_HOME}/bin/nvcc" --version | tail -1
echo "CUDA toolkit OK: ${CUDA_HOME}"
