#!/bin/bash

# Absolute path to this script's directory, captured BEFORE any cd (sourcing hslam_env.sh
# later with a relative path breaks once the script changes directory).
HSLAM_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

########################
# HSLAM ML Depth Mode — ICL-NUIM Dataset
# Sprint 0d follow-up: HSLAM² depth-only baseline for the 3-way ablation
# Uses --associations pipeline (TUM-compatible format) for correct
# numeric frame ordering (ICL-NUIM image names are not zero-padded).
# Depth source = ML (Metric3D) — matches production HSLAM² configuration.
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions="${REPS:-1}"

# Default sequence
DEFAULT_SEQ="living_room_traj0"
SEQ_NAME="${1:-$DEFAULT_SEQ}"

# Dataset paths
DATASET_BASE="$HOME/datasets/ICL_NUIM"
dataset_path="$DATASET_BASE/$SEQ_NAME"
calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/run_scripts/camera_icl_nuim.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"
assoc_path="$dataset_path/associations.txt"

# ML model configuration
ml_model_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/models/metric3d-vit-small/onnx/model.onnx"
ml_strategy="keyframe_only"

# GPU configuration
ml_gpu_enabled="true"
ml_gpu_device="0"
ml_gpu_memory="2048"

# End index (override with ENDINDEX env var, e.g. for smoke test)
ENDINDEX="${ENDINDEX:-100000}"

print_info()    { echo -e "\033[1;34m[INFO]\033[0m $1"; }
print_success() { echo -e "\033[1;32m[SUCCESS]\033[0m $1"; }
print_error()   { echo -e "\033[1;31m[ERROR]\033[0m $1"; }
print_ml_info() { echo -e "\033[1;36m[ML]\033[0m $1"; }

main() {
    print_info "HSLAM ML Depth Mode — ICL-NUIM $SEQ_NAME"
    print_info "Repetitions: $repetitions, endindex: $ENDINDEX"
    print_ml_info "Model: $(basename $ml_model_path), strategy: $ml_strategy, GPU: $ml_gpu_enabled"

    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"; exit 1
    fi
    if [ ! -f "$assoc_path" ]; then
        print_error "Associations file not found: $assoc_path"; exit 1
    fi
    if [ ! -f "$ml_model_path" ]; then
        print_error "ML model not found: $ml_model_path"; exit 1
    fi

    enable_loop_closure=false
    if [ -f "$vocab_path" ]; then enable_loop_closure=true; fi

    # ONNX Runtime needs the GPU lib path
    # Shared env: ONNX Runtime + SONAME compat shims (see run_scripts/hslam_env.sh)
    source "$HSLAM_SCRIPT_DIR/hslam_env.sh"
    mkdir -p "$results_directory"
    timestamp=$(date +"%Y%m%d_%H%M%S")

    cd "$build_directory_path"

    for ((i = 0; i < repetitions; i++)); do
        print_info "Run $((i+1))/$repetitions"

        # ML-depth pipeline: --depth-source=ml + --associations for ICL-NUIM frame ordering.
        # The associations file provides ordering only (depth column ignored when
        # depth-source=ml); ML inference produces metric depth from RGB.
        cmd="\"${build_directory_path}HSLAM\" -f \"$dataset_path\" -c \"$calib_path\""
        cmd="$cmd --associations=\"$assoc_path\""
        cmd="$cmd --depth-source=ml"
        cmd="$cmd --ml-depth=true --ml-model \"$ml_model_path\" --ml-strategy $ml_strategy --ml-init=true"
        if [ "$ml_gpu_enabled" = "true" ]; then
            cmd="$cmd --ml-gpu --ml-gpu-device $ml_gpu_device --ml-gpu-memory $ml_gpu_memory"
        fi
        if [ "$enable_loop_closure" = true ]; then
            cmd="$cmd -v \"$vocab_path\" --loopclosure=true"
        fi
        cmd="$cmd --preset=0 --mode=1 --quiet --nogui=true --nolog --endindex=$ENDINDEX"
        cmd="$cmd --export-map-ply=true --map-ply-out=result.ply"

        print_info "Command: $cmd"
        eval $cmd
        exit_code=$?

        dest="$results_directory/hslam-iclnuim-ml-depth-$SEQ_NAME-$timestamp"
        mkdir -p "$dest"

        [ -f "result.txt" ] && mv "result.txt" "$dest/trajectory_$i.txt" && print_success "Saved trajectory"
        [ -f "result.ply" ] && mv "result.ply"  "$dest/map_$i.ply"       && print_success "Saved PLY map"
        [ -f "PC.PCD" ]     && mv "PC.PCD"      "$dest/PC_$i.PCD"

        ln -sfn "$dest" "$results_directory/latest-iclnuim-ml-depth"
        print_info "Results -> $dest"

        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed (exit $exit_code)"; exit 1
        fi
    done

    print_success "Done. Latest results: $results_directory/latest-iclnuim-ml-depth"
}

main
