#!/bin/bash

########################
# HSLAM TUM VO Mode Script
# Generates trajectory for TUM VO dataset
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Dataset name (extract from path or set default)
DATASET_NAME="sequence_01"

# Function to print colored output
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

# Main execution
main() {
    print_info "HSLAM TUM VO Mode (TUM Visual Odometry Dataset)"
    print_info "Dataset: $DATASET_NAME"
    print_info "=========================================="

    # Check build directory
    if [ ! -d "$build_directory_path" ]; then
        print_error "Build directory not found: $build_directory_path"
        exit 1
    fi

    mkdir -p "$results_directory"
    timestamp=$(date +"%Y%m%d_%H%M%S")
    cd "$build_directory_path"

    for ((i = 0; i < repetitions; i++)); do
        print_info "Starting HSLAM TUM VO run $((i+1))/$repetitions"

        cmd="./HSLAM \
            --files=$HOME/datasets/sequence_01/images.zip \
            --calib=$HOME/datasets/sequence_01/camera.txt \
            --vocab=$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3 \
            --gamma=$HOME/datasets/sequence_01/pcalib.txt \
            --vignette=$HOME/datasets/sequence_01/vignette.png \
            --loopclosure=True \
            --preset=0 \
            --mode=2 \
            --nogui=true \
            --nolog"

        print_info "Executing: $cmd"
        eval $cmd
        exit_code=$?
        print_info "HSLAM completed with exit code: $exit_code"

        destination_directory="$results_directory/hslam-tumVO-$DATASET_NAME-$timestamp"
        mkdir -p "$destination_directory"

        if [ -f "result.txt" ]; then
            mv "result.txt" "$destination_directory/trajectory_tumvo_$i.txt"
            print_success "Saved trajectory: $destination_directory/trajectory_tumvo_$i.txt"
        else
            print_warning "result.txt not found"
        fi

        if [ -f "map.pcd" ]; then
            mv "map.pcd" "$destination_directory/map_tumvo_$i.pcd"
        fi

        if [ -d "logs" ]; then
            mv "logs" "$destination_directory/logs_tumvo_$i"
        fi

        if [ -d "mats" ]; then
            rm -rf "mats"
        fi

        # Create run summary
        cat > "$destination_directory/run_summary.txt" << EOF
HSLAM TUM VO Trajectory Generation Summary
==========================================

Mode: TUM Visual Odometry
Dataset: $DATASET_NAME
Timestamp: $timestamp
Dataset Path: $HOME/datasets/sequence_01
Calibration: $HOME/datasets/sequence_01/camera.txt
Loop Closure: true

Generated Files:
- trajectory_tumvo_$i.txt  (TUM VO trajectory)
- Point clouds (*.pcd)

Exit Code: $exit_code
EOF

        # Create symlink to latest results
        latest_link="$results_directory/latest-tumvo"
        ln -sfn "$destination_directory" "$latest_link"

        print_success "Results saved to: $destination_directory"

        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed with exit code: $exit_code"
            exit 1
        fi
    done

    print_info "=========================================="
    print_success "TUM VO trajectory generation completed!"
    print_info "Results directory: $destination_directory"
    print_info "Trajectory file: trajectory_tumvo_0.txt"
    print_info "Latest results: $latest_link"
    print_info ""
    print_info "Ready for evo evaluation!"
}

main "$@"