#!/bin/bash

########################
# HSLAM Monocular Mode Script
# Generates trajectory WITHOUT depth integration
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Default dataset (can be overridden via command line)
DEFAULT_DATASET="freiburg1_room"
DATASET_NAME="${1:-$DEFAULT_DATASET}"

# Dataset paths
DATASET_BASE="$HOME/datasets/TUM_RGBD"
dataset_path="$DATASET_BASE/rgbd_dataset_$DATASET_NAME"
calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/run_scripts/camera_freiburg1.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"

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
    echo "Usage: $0 [DATASET_NAME]"
    echo ""
    echo "DATASET_NAME: TUM RGB-D dataset name (default: freiburg1_desk)"
    echo "Available datasets:"
    ls -1 "$DATASET_BASE" 2>/dev/null | grep "rgbd_dataset_" | sed 's/rgbd_dataset_/  - /' || echo "  (Dataset directory not found)"
    echo ""
    echo "Examples:"
    echo "  $0                    # Use default dataset (freiburg1_desk)"
    echo "  $0 freiburg1_xyz      # Use freiburg1_xyz dataset"
    echo "  $0 freiburg2_desk     # Use freiburg2_desk dataset"
    echo ""
    echo "This script generates trajectory WITHOUT depth integration (monocular SLAM only)"
    echo "Results saved in: $results_directory/hslam-monocular-{dataset}-{timestamp}/"
}

# Parse command line arguments
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

# Main execution
main() {
    print_info "HSLAM Monocular Mode (WITHOUT Depth Integration)"
    print_info "Dataset: $DATASET_NAME"
    print_info "=========================================="
    
    # Check prerequisites
    print_info "Checking prerequisites..."
    
    # Check build directory
    if [ ! -d "$build_directory_path" ]; then
        print_error "Build directory not found: $build_directory_path"
        exit 1
    fi
    
    # Check dataset
    if [ ! -d "$dataset_path" ]; then
        print_error "Dataset directory not found: $dataset_path"
        print_info "Available datasets:"
        ls -1 "$DATASET_BASE" | grep "rgbd_dataset_" | sed 's/rgbd_dataset_/  - /'
        exit 1
    fi
    
    # Check calibration file
    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"
        exit 1
    fi
    
    # Check vocabulary file
    enable_loop_closure=false
    if [ -f "$vocab_path" ]; then
        print_info "Vocabulary file found, enabling loop closure"
        enable_loop_closure=true
    else
        print_warning "Vocabulary file not found, disabling loop closure"
    fi
    
    # Create results directory
    mkdir -p "$results_directory"
    
    # Generate timestamp for this run
    timestamp=$(date +"%Y%m%d_%H%M%S")
    
    cd "$build_directory_path"
    
    # Run HSLAM in monocular mode
    for ((i = 0; i < repetitions; i++)); do
        print_info "Starting HSLAM monocular run $((i+1))/$repetitions"
        
        # Build command - use RGB subdirectory for pure monocular mode
        local cmd="./HSLAM -f \"$dataset_path/rgb\" -c \"$calib_path\""
        
        if [ "$enable_loop_closure" = true ]; then
            cmd="$cmd -v \"$vocab_path\" --loopclosure=true"
        fi
        
        cmd="$cmd --preset=0 --mode=1 --quiet --nogui=false --outPC=true"
        
        print_info "Executing: $cmd"
        eval $cmd
        
        exit_code=$?
        print_info "HSLAM completed with exit code: $exit_code"
        
        # Organize results
        destination_directory="$results_directory/hslam-monocular-$DATASET_NAME-$timestamp"
        mkdir -p "$destination_directory"
        
        if [ -f "result.txt" ]; then
            mv "result.txt" "$destination_directory/trajectory_monocular_$i.txt"
            print_success "Saved trajectory: $destination_directory/trajectory_monocular_$i.txt"
        else
            print_warning "result.txt not found"
        fi
        
        if [ -f "PC.PCD" ]; then
            mv "PC.PCD" "$destination_directory/pointcloud_monocular_$i.pcd"
        fi
        
        if [ -f "map.pcd" ]; then
            mv "map.pcd" "$destination_directory/map_monocular_$i.pcd"
        fi
        
        if [ -d "logs" ]; then
            mv "logs" "$destination_directory/logs_monocular_$i"
        fi
        
        if [ -d "mats" ]; then
            mv "mats" "$destination_directory/mats_monocular_$i"
        fi
        
        # Move debug images if they exist (when --save flag is used)
        # Check both build directory and main HSLAM directory for images_out
        images_out_path=""
        if [ -d "images_out" ]; then
            images_out_path="images_out"
        elif [ -d "../images_out" ]; then
            images_out_path="../images_out"
        fi
        
        if [ -n "$images_out_path" ]; then
            mv "$images_out_path" "$destination_directory/debug_images"
            debug_count=$(find "$destination_directory/debug_images" -name "*.png" 2>/dev/null | wc -l)
            if [ "$debug_count" -gt 0 ]; then
                print_success "Saved $debug_count SLAM debug images to: $destination_directory/debug_images/"
            fi
        fi
        
        # Create run summary
        cat > "$destination_directory/run_summary.txt" << EOF
HSLAM Monocular Trajectory Generation Summary
=============================================

Mode: Monocular SLAM (WITHOUT depth integration)
Dataset: $DATASET_NAME
Timestamp: $timestamp
Dataset Path: $dataset_path
Ground Truth: $dataset_path/groundtruth.txt
Calibration: $calib_path
Loop Closure: $enable_loop_closure

Generated Files:
- trajectory_monocular_$i.txt  (monocular trajectory)
- debug_images/             (SLAM debug images, when --save used)
- Point clouds (*.pcd)
- System logs (logs_*)
- Internal matrices (mats_*)

Exit Code: $exit_code
EOF
        
        # Create symlink to latest results
        latest_link="$results_directory/latest-monocular"
        ln -sfn "$destination_directory" "$latest_link"
        
        print_success "Results saved to: $destination_directory"
        
        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed with exit code: $exit_code"
            exit 1
        fi
    done
    
    # Summary
    print_info "=========================================="
    print_success "Monocular trajectory generation completed!"
    print_info "Results directory: $destination_directory"
    print_info "Trajectory file: trajectory_monocular_0.txt"
    print_info "Ground truth file: $dataset_path/groundtruth.txt"
    print_info "Latest results: $latest_link"
    print_info ""
    print_info "Ready for evo evaluation!"
}

# Run main function
main 