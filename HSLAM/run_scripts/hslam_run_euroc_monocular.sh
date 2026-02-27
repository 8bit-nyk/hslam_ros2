#!/bin/bash

########################
# HSLAM EuRoC Monocular Mode Script
# Generates trajectory WITHOUT depth integration
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Default dataset (can be overridden via command line)
DEFAULT_DATASET="MH_01_easy"
DATASET_NAME="${1:-$DEFAULT_DATASET}"

# Dataset paths
DATASET_BASE="$HOME/datasets/EuRoC"
dataset_path="$DATASET_BASE/$DATASET_NAME/mav0"
calib_path="$dataset_path/cam0/camera.txt"
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
    echo "Usage: $0 [DATASET_NAME] [OPTIONS]"
    echo ""
    echo "DATASET_NAME: EuRoC MAV dataset name (default: MH_01_easy)"
    echo ""
    echo "Options:"
    echo "  --endindex N         Process only first N frames"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Available datasets:"
    ls -1 "$DATASET_BASE" 2>/dev/null | grep -E "^(MH|V)_[0-9]+_" | sed 's/^/  - /' || echo "  (Dataset directory not found)"
    echo ""
    echo "Examples:"
    echo "  $0                    # Use default dataset (MH_01_easy)"
    echo "  $0 MH_02_easy         # Use MH_02_easy dataset"
    echo "  $0 V1_01_easy         # Use V1_01_easy dataset"
    echo "  $0 MH_01_easy --endindex 20  # Process first 20 frames only"
    echo ""
    echo "This script generates trajectory WITHOUT depth integration (monocular SLAM only)"
    echo "Results saved in: $results_directory/hslam-euroc-monocular-{dataset}-{timestamp}/"
    echo ""
    echo "EuRoC Dataset Structure:"
    echo "  Images: {dataset}/mav0/cam0/data/*.png"
    echo "  Timestamps: {dataset}/mav0/cam0/times.txt"
    echo "  Calibration: {dataset}/mav0/cam0/camera.txt"
    echo "  Ground Truth: {dataset}/mav0/state_groundtruth_estimate0/data.csv"
}

# Processing configuration
end_index="" # Process limited frames for testing (set to "" for full trajectory)

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --endindex)
            end_index="$2"
            shift 2
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
        *)
            # First non-option argument is dataset name
            if [ -z "$DATASET_NAME_SET" ]; then
                DATASET_NAME="$1"
                DATASET_NAME_SET=1
                # Update paths with new dataset name
                dataset_path="$DATASET_BASE/$DATASET_NAME/mav0"
                calib_path="$dataset_path/cam0/camera.txt"
            else
                echo "Error: Multiple dataset names specified"
                exit 1
            fi
            shift
            ;;
    esac
done

# Main execution
main() {
    print_info "HSLAM EuRoC Monocular Mode (WITHOUT Depth Integration)"
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
        ls -1 "$DATASET_BASE" | grep -E "^(MH|V)_[0-9]+_" | sed 's/^/  - /'
        exit 1
    fi
    
    # Check camera data directory
    if [ ! -d "$dataset_path/cam0/data" ]; then
        print_error "Camera data directory not found: $dataset_path/cam0/data"
        exit 1
    fi
    
    # Check calibration file
    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"
        exit 1
    fi
    
    # Check times.txt file
    times_file="$dataset_path/cam0/times.txt"
    if [ ! -f "$times_file" ]; then
        print_error "Times file not found: $times_file"
        exit 1
    fi
    
    # Verify dataset structure
    image_count=$(find "$dataset_path/cam0/data" -name "*.png" | wc -l)
    print_info "Found $image_count images in dataset"
    
    if [ "$image_count" -eq 0 ]; then
        print_error "No PNG images found in $dataset_path/cam0/data"
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
        print_info "Starting HSLAM EuRoC monocular run $((i+1))/$repetitions"
        
        # Build command - use cam0/data subdirectory for monocular mode
        local cmd="./HSLAM -f \"$dataset_path/cam0/data\" -c \"$calib_path\""
        
        if [ "$enable_loop_closure" = true ]; then
            cmd="$cmd -v \"$vocab_path\" --loopclosure=true"
        fi
        
        cmd="$cmd --preset=0 --mode=1 --quiet --nogui=false --outPC=true --nolog"
        
        # Add endindex if specified
        [ -n "$end_index" ] && cmd="$cmd --endindex $end_index"
        
        print_info "Executing: $cmd"
        eval $cmd
        
        exit_code=$?
        print_info "HSLAM completed with exit code: $exit_code"
        
        # Organize results
        destination_directory="$results_directory/hslam-euroc-monocular-$DATASET_NAME-$timestamp"
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
HSLAM EuRoC Monocular Trajectory Generation Summary
==================================================

Mode: Monocular SLAM (WITHOUT depth integration)
Dataset: $DATASET_NAME
Timestamp: $timestamp
Dataset Path: $dataset_path
Images: $dataset_path/cam0/data/
Timestamps: $dataset_path/cam0/times.txt
Calibration: $calib_path
Ground Truth: $dataset_path/state_groundtruth_estimate0/data.csv
Loop Closure: $enable_loop_closure

EuRoC Dataset Info:
- Total Images: $image_count
- Camera: VI-Sensor cam0 (MT9M034)
- Resolution: 752x480
- Rate: 20 Hz

Generated Files:
- trajectory_monocular_$i.txt  (monocular trajectory in TUM format)
- debug_images/             (SLAM debug images, when --save used)
- Point clouds (*.pcd)
- System logs (logs_*)
- Internal matrices (mats_*)

Exit Code: $exit_code

Evaluation:
To evaluate trajectory against ground truth, use:
  evo_ape euroc $dataset_path/state_groundtruth_estimate0/data.csv trajectory_monocular_$i.txt --plot
EOF
        
        # Create symlink to latest results
        latest_link="$results_directory/latest-euroc-monocular"
        ln -sfn "$destination_directory" "$latest_link"
        
        print_success "Results saved to: $destination_directory"
        
        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed with exit code: $exit_code"
            exit 1
        fi
    done
    
    # Summary
    print_info "=========================================="
    print_success "EuRoC monocular trajectory generation completed!"
    print_info "Results directory: $destination_directory"
    print_info "Trajectory file: trajectory_monocular_0.txt"
    print_info "Ground truth file: $dataset_path/state_groundtruth_estimate0/data.csv"
    print_info "Latest results: $latest_link"
    print_info ""
    print_info "Ready for evo evaluation!"
    print_info "  evo_ape euroc $dataset_path/state_groundtruth_estimate0/data.csv trajectory_monocular_0.txt --plot"
}

# Run main function
main