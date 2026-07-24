#!/bin/bash
# 3D Gaussian Splatting demo runner for HSLAM (self-contained).
#
# Usage:
#   ./run_gs.sh --files=/path/to/replica/dso/room0/images \
#     --calib=configs/replica_calib/camera.txt \
#     --vocab=misc/orbvoc.dbow3 --colour --loopclosure \
#     --gauss=configs/gaussian_mapper/replica_mono.yaml --mode=1

cd "$(dirname "$0")" || exit 1
HSLAM_DIR="$(pwd)"

# CUDA runtime (override CUDA_HOME if your toolkit lives elsewhere)
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-11.6}"

# libstdc++ preload: needed when the binary was built inside a conda env and the system
# libstdc++ is older. Auto-detected from an active conda env; override with STDCXX_LIB.
STDCXX_LIB="${STDCXX_LIB:-${CONDA_PREFIX:+$CONDA_PREFIX/lib/libstdc++.so.6}}"

# All runtime libraries live under Thirdparty/ (self-contained)
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:$HSLAM_DIR/Thirdparty/CompiledLibs/lib:$HSLAM_DIR/Thirdparty/libtorch/lib:$HSLAM_DIR/Thirdparty/onnxruntime/lib:$LD_LIBRARY_PATH"
[ -n "$STDCXX_LIB" ] && export LD_PRELOAD="$STDCXX_LIB"

exec ./build/bin/HSLAM "$@"
