#!/bin/bash

########################
# HSLAM KITTI Monocular Mode Script
# Generates trajectory WITHOUT ML depth integration (for comparison)
# Based on existing KITTI monocular script
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Default dataset (can be overridden via command line)
DEFAULT_DATASET="00"
DATASET_NUM="${1:-$DEFAULT_DATASET}"

# Dataset paths
DATASET_BASE="$HOME/datasets/KITTI"
dataset_path="$DATASET_BASE/$DATASET_NUM"
image_path="$dataset_path/image_1"  # RGB images from KITTI
# Select calibration file based on sequence number
case "$DATASET_NUM" in
    00|01|02) calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/Kitti/Kitti00-02.txt" ;;
    03)       calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/Kitti/Kitti03.txt" ;;
    *)        calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/Kitti/Kitti04-12.txt" ;;
esac
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"

# Processing configuration
end_index="" # Process limited frames for testing
quiet_mode=""
output_log="hslam_kitti_monocular_run.log"

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

# Help function
show_help() {
    echo "Usage: $0 [DATASET_NUM] [OPTIONS]"
    echo ""
    echo "DATASET_NUM: KITTI sequence number (default: 07)"
    echo ""
    echo "Options:"
    echo "  --quiet              Enable quiet mode"
    echo "  --endindex N         Process only first N frames"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "This script generates trajectory WITHOUT ML depth integration (monocular SLAM)"
    echo "Use for comparison against ML depth enhanced trajectories."
    echo "Results saved in: $results_directory/hslam-kitti-monocular-{dataset}-{timestamp}/"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --quiet)
            quiet_mode="--quiet"
            shift
            ;;
        --endindex)
            end_index="$2"
            shift 2
            ;;
        -*)
            echo "Unknown option: $1"
            exit 1
            ;;
        *)
            if [ -z "$DATASET_NUM_SET" ]; then
                DATASET_NUM="$1"
                DATASET_NUM_SET=1
                dataset_path="$DATASET_BASE/$DATASET_NUM"
                image_path="$dataset_path/image_2"
            fi
            shift
            ;;
    esac
done

# Main execution
main() {
    print_info "HSLAM KITTI Monocular Mode (WITHOUT ML depth)"
    print_info "Dataset: KITTI $DATASET_NUM"
    print_info "==============================================="
    
    # Check prerequisites (same as ML version)
    if [ ! -d "$build_directory_path" ]; then
        print_error "Build directory not found: $build_directory_path"
        exit 1
    fi
    
    if [ ! -d "$image_path" ]; then
        print_error "RGB images directory not found: $image_path"
        exit 1
    fi
    
    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"
        exit 1
    fi
    
    # Check for timestamps file (original or converted)
    times_file="$image_path/times.txt"
    original_times="$dataset_path/times.txt"
    
    if [ ! -f "$times_file" ] && [ ! -f "$original_times" ]; then
        print_error "No times file found. Expected either:"
        print_error "  $times_file (converted)"
        print_error "  $original_times (original KITTI)"
        exit 1
    elif [ ! -f "$times_file" ] && [ -f "$original_times" ]; then
        print_warning "Using original KITTI times.txt format"
        print_info "Creating symlink from original times.txt"
        ln -sf "$original_times" "$times_file"
    fi
    
    image_count=$(find "$image_path" -name "*.png" | wc -l)
    print_info "Found $image_count RGB images"
    
    mkdir -p "$results_directory"
    timestamp=$(date +"%Y%m%d_%H%M%S")
    
    cd "$build_directory_path"
    
    # Run HSLAM WITHOUT ML depth
    print_info "Running HSLAM in monocular mode (no ML depth)..."
    
    cmd="$build_directory_path/HSLAM \
        --files $image_path \
        --calib $calib_path \
        --vocab $vocab_path \
        --colour \
        --loopclosure \
        --nogui=false"
    
    [ -n "$quiet_mode" ] && cmd="$cmd $quiet_mode"
    [ -n "$end_index" ] && cmd="$cmd --endindex $end_index"
    cmd="$cmd --nolog"
    
    eval "$cmd 2>&1" | tee $output_log
    exit_code=${PIPESTATUS[0]}
    
    # Organize results
    destination_directory="$results_directory/hslam-kitti-monocular-$DATASET_NUM-$timestamp"
    mkdir -p "$destination_directory"
    
    # Move files
    [ -f "result.txt" ] && mv "result.txt" "$destination_directory/trajectory_monocular_0.txt"
    [ -f "PC.PCD" ] && mv "PC.PCD" "$destination_directory/pointcloud_monocular_0.pcd"
    [ -f "map.pcd" ] && mv "map.pcd" "$destination_directory/map_monocular_0.pcd"
    [ -d "logs" ] && mv "logs" "$destination_directory/logs_monocular_0"
    [ -d "mats" ] && mv "mats" "$destination_directory/mats_monocular_0"
    [ -f "$output_log" ] && mv "$output_log" "$destination_directory/run_log_monocular_0.txt"
    
    # Create summary
    cat > "$destination_directory/run_summary.txt" << EOF
HSLAM KITTI Monocular Trajectory Generation Summary
===================================================

Mode: Monocular SLAM (WITHOUT ML depth integration)
Dataset: KITTI $DATASET_NUM
Timestamp: $timestamp
Total Images: $image_count
Exit Code: $exit_code

Generated Files:
- trajectory_monocular_0.txt (Monocular trajectory for comparison)
- Complete logs and debug data

Use this trajectory as baseline for comparison against ML depth enhanced results.
EOF
    
    # Create symlink to latest results
    latest_link="$results_directory/latest-kitti-monocular"
    ln -sfn "$destination_directory" "$latest_link"
    
    print_success "Monocular results saved to: $destination_directory"
    
    if [ $exit_code -eq 0 ]; then
        print_success "KITTI monocular SLAM completed successfully!"
    else
        print_warning "HSLAM completed with exit code: $exit_code"
    fi
}

main