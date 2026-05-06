#!/bin/bash

########################
# HSLAM Monocular Mode — ICL-NUIM Dataset
# Sprint 0d: generates trajectory + PLY map for map-quality evaluation
# Uses --associations pipeline (TUM-compatible format) to ensure correct
# numeric frame ordering (ICL-NUIM image names are not zero-padded).
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

# End index (override with ENDINDEX env var, e.g. for smoke test)
ENDINDEX="${ENDINDEX:-100000}"

print_info()    { echo -e "\033[1;34m[INFO]\033[0m $1"; }
print_success() { echo -e "\033[1;32m[SUCCESS]\033[0m $1"; }
print_error()   { echo -e "\033[1;31m[ERROR]\033[0m $1"; }

main() {
    print_info "HSLAM Monocular Mode — ICL-NUIM $SEQ_NAME"
    print_info "Repetitions: $repetitions, endindex: $ENDINDEX"
    print_info "Note: using associations.txt pipeline for correct frame ordering"

    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"; exit 1
    fi
    if [ ! -f "$assoc_path" ]; then
        print_error "Associations file not found: $assoc_path"; exit 1
    fi

    enable_loop_closure=false
    if [ -f "$vocab_path" ]; then enable_loop_closure=true; fi

    mkdir -p "$results_directory"
    timestamp=$(date +"%Y%m%d_%H%M%S")

    cd "$build_directory_path"

    for ((i = 0; i < repetitions; i++)); do
        print_info "Run $((i+1))/$repetitions"

        # Use associations.txt pipeline (--depth-source=none: associations only used for ordering,
        # no actual depth loaded into the SLAM system — pure monocular tracking)
        cmd="\"${build_directory_path}HSLAM\" -f \"$dataset_path\" -c \"$calib_path\""
        cmd="$cmd --associations=\"$assoc_path\""
        cmd="$cmd --depth-source=none"
        if [ "$enable_loop_closure" = true ]; then
            cmd="$cmd -v \"$vocab_path\" --loopclosure=true"
        fi
        cmd="$cmd --preset=0 --mode=1 --quiet --nogui=true --nolog --endindex=$ENDINDEX"
        cmd="$cmd --export-map-ply=true --map-ply-out=result.ply"

        print_info "Command: $cmd"
        eval $cmd
        exit_code=$?

        dest="$results_directory/hslam-iclnuim-mono-$SEQ_NAME-$timestamp"
        mkdir -p "$dest"

        [ -f "result.txt" ] && mv "result.txt" "$dest/trajectory_$i.txt" && print_success "Saved trajectory"
        [ -f "result.ply" ] && mv "result.ply"  "$dest/map_$i.ply"       && print_success "Saved PLY map"
        [ -f "PC.PCD" ]     && mv "PC.PCD"      "$dest/PC_$i.PCD"

        ln -sfn "$dest" "$results_directory/latest-iclnuim-mono"
        print_info "Results -> $dest"

        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed (exit $exit_code)"; exit 1
        fi
    done

    print_success "Done. Latest results: $results_directory/latest-iclnuim-mono"
}

main
