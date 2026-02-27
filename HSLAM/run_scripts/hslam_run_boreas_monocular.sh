#!/bin/bash

########################
# HSLAM Monocular Mode Script for Boreas Dataset
# Generates trajectory WITHOUT depth integration
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Default dataset sequence
DEFAULT_SEQUENCE="boreas-2021-01-26-10-59"
SEQUENCE_NAME="${1:-$DEFAULT_SEQUENCE}"

# Dataset paths
DATASET_BASE="$HOME/datasets/boreas"
dataset_path="$DATASET_BASE/$SEQUENCE_NAME"
image_path="$dataset_path/cropped_images"
calib_path="$dataset_path/calib/camera_cropped.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"

# Processing configuration
end_index="" # Process limited frames for testing (set to "" for full trajectory)
quiet_mode=""  # Set to "--quiet" to reduce HSLAM console output for performance
output_log="hslam_boreas_monocular_run.log"

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
    echo "Usage: $0 [SEQUENCE_NAME] [OPTIONS]"
    echo ""
    echo "SEQUENCE_NAME: Boreas dataset sequence name (default: boreas-2021-01-26-10-59)"
    echo ""
    echo "Options:"
    echo "  --quiet              Enable quiet mode (minimal console output for performance)"
    echo "  --endindex N         Process only first N frames"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Available sequences:"
    ls -1 "$DATASET_BASE" 2>/dev/null | sed 's/^/  - /' || echo "  (Dataset directory not found)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use default sequence"
    echo "  $0 boreas-2021-01-26-10-59          # Use specific sequence"
    echo "  $0 boreas-2021-01-26-10-59 --quiet  # Quiet mode (minimal output)"
    echo "  $0 boreas-2021-01-26-10-59 --endindex 100  # Process first 100 frames only"
    echo ""
    echo "This script generates trajectory WITHOUT depth integration (monocular SLAM only)"
    echo "Results saved in: $results_directory/hslam-boreas-monocular-{sequence}-{timestamp}/"
    echo ""
    echo "Boreas Dataset Structure:"
    echo "  Images: {sequence}/cropped_images/*.png (640x480 RGB images)"
    echo "  Timestamps: {sequence}/cropped_images/times.txt (converted from Boreas format)"
    echo "  Calibration: {sequence}/calib/camera_cropped.txt"
    echo ""
    echo "Prerequisites:"
    echo "  1. Convert timestamps: python3 scripts/convert_boreas_times.py \\"
    echo "                         $dataset_path/times.txt"
    echo "  2. Verify RGB images exist in $image_path/"
    echo "  3. Ensure calibration file exists: $calib_path"
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
            echo "Use --help for usage information"
            exit 1
            ;;
        *)
            # First non-option argument is sequence name
            if [ -z "$SEQUENCE_NAME_SET" ]; then
                SEQUENCE_NAME="$1"
                SEQUENCE_NAME_SET=1
                # Update paths with new sequence name
                dataset_path="$DATASET_BASE/$SEQUENCE_NAME"
                image_path="$dataset_path/cropped_images"
                calib_path="$dataset_path/calib/camera_cropped.txt"
            else
                echo "Error: Multiple sequence names specified"
                exit 1
            fi
            shift
            ;;
    esac
done

# Main execution
main() {
    print_info "HSLAM Monocular Mode for Boreas Dataset (WITHOUT Depth Integration)"
    print_info "Sequence: $SEQUENCE_NAME"
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
        print_info "Available sequences:"
        ls -1 "$DATASET_BASE" | sed 's/^/  - /'
        exit 1
    fi
    
    # Check image directory
    if [ ! -d "$image_path" ]; then
        print_error "Image directory not found: $image_path"
        exit 1
    fi
    
    # Check calibration file
    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"
        exit 1
    fi
    
    # Check timestamps file
    timestamp_file="$image_path/times.txt"
    if [ ! -f "$timestamp_file" ]; then
        print_error "Timestamp file not found: $timestamp_file"
        print_info "Run timestamp conversion first:"
        print_info "  python3 scripts/convert_boreas_times.py $dataset_path/times.txt"
        exit 1
    fi
    
    # Verify RGB images
    image_count=$(find "$image_path" -name "*.png" | wc -l)
    print_info "Found $image_count RGB images in dataset"
    
    if [ "$image_count" -eq 0 ]; then
        print_error "No PNG images found in $image_path"
        exit 1
    fi
    
    # Validate first image is RGB
    first_image=$(find "$image_path" -name "*.png" | sort | head -1)
    if [ -n "$first_image" ]; then
        image_info=$(file "$first_image")
        if echo "$image_info" | grep -q "PNG image"; then
            print_success "✓ Image validation: PNG format detected"
        else
            print_warning "⚠ Image validation: Unexpected format - $image_info"
        fi
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
        
        # Build command - use cropped_images directory for pure monocular mode
        local cmd="./HSLAM -f \"$image_path\" -c \"$calib_path\""
        
        if [ "$enable_loop_closure" = true ]; then
            cmd="$cmd -v \"$vocab_path\" --loopclosure=true"
        fi
        
        cmd="$cmd --preset=0 --mode=1 --nogui=false --outPC=true --nolog"
        
        # Add optional flags
        [ -n "$quiet_mode" ] && cmd="$cmd $quiet_mode"
        [ -n "$end_index" ] && cmd="$cmd --endindex $end_index"
        
        print_info "Executing: $cmd"
        
        # Execute and log
        eval "$cmd 2>&1" | tee "$output_log"
        
        exit_code=${PIPESTATUS[0]}
        print_info "HSLAM completed with exit code: $exit_code"
        
        # Organize results
        destination_directory="$results_directory/hslam-boreas-monocular-$SEQUENCE_NAME-$timestamp"
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
        
        # Move log file
        if [ -f "$output_log" ]; then
            mv "$output_log" "$destination_directory/run_log_monocular_$i.txt"
        fi
        
        # Move debug images if they exist (when --save flag is used)
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
HSLAM Boreas Monocular Trajectory Generation Summary
==================================================

Mode: Monocular SLAM (WITHOUT depth integration)
Sequence: $SEQUENCE_NAME
Timestamp: $timestamp
Dataset Path: $dataset_path
Image Path: $image_path
Calibration: $calib_path
Loop Closure: $enable_loop_closure

Dataset Statistics:
- Total Images: $image_count
- Image Format: PNG (640x480)
- Timestamps: Converted from Boreas microseconds to TUM format

Generated Files:
- trajectory_monocular_$i.txt  (monocular trajectory)
- run_log_monocular_$i.txt    (Complete execution log)
- debug_images/             (SLAM debug images, when --save used)
- Point clouds (*.pcd)
- System logs (logs_*)
- Internal matrices (mats_*)

Exit Code: $exit_code

Notes:
- Pure monocular SLAM without any depth information
- Scale is arbitrary and determined by initialization
- Loop closure helps with drift correction if vocabulary is available
EOF
        
        # Create symlink to latest results
        latest_link="$results_directory/latest-boreas-monocular"
        ln -sfn "$destination_directory" "$latest_link"
        
        print_success "Results saved to: $destination_directory"
        
        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed with exit code: $exit_code"
            exit 1
        fi
    done
    
    # Summary
    print_info "=========================================="
    print_success "Boreas monocular trajectory generation completed!"
    print_info "Results directory: $destination_directory"
    print_info "Trajectory file: trajectory_monocular_0.txt"
    print_info "Latest results: $latest_link"
    print_info ""
    print_info "Ready for evaluation!"
}

# Run main function
main