#!/bin/bash

########################
# HSLAM Monocular Mode Script - HKU Airport Dataset
# Generates trajectory WITHOUT depth integration
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Dataset paths
dataset_path="/home/aub/datasets/HKU_AIRPORT/images"
calib_path="/home/aub/datasets/HKU_AIRPORT/camera_hku.txt"
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
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "HKU Airport Dataset - Monocular Mode"
    echo ""
    echo "Options:"
    echo "  --endindex N         Process only first N frames"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Run full HKU Airport sequence (monocular)"
    echo "  $0 --endindex 100     # Process first 100 frames only"
    echo ""
    echo "This script generates trajectory WITHOUT depth integration (monocular SLAM only)"
    echo "Dataset: HKU Airport (drone footage)"
    echo "Results saved in: $results_directory/hslam-monocular-hku-airport-{timestamp}/"
}

# Parse command line arguments
end_index=""
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
            echo "Error: Unexpected argument: $1"
            exit 1
            ;;
    esac
done

# Main execution
main() {
    print_info "HSLAM Monocular Mode - HKU Airport Dataset"
    print_info "Dataset: HKU Airport (drone footage)"
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
        print_info "Please extract images from HKU Airport bag first"
        exit 1
    fi
    
    # Check for images in dataset
    image_count=$(find "$dataset_path" -name "*.png" -o -name "*.jpg" | wc -l)
    if [ "$image_count" -eq 0 ]; then
        print_error "No images found in dataset directory: $dataset_path"
        print_info "Please extract images from /home/aub/datasets/HKUAIRPORT.bag first"
        exit 1
    fi
    print_info "Found $image_count images in dataset"
    
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
        
        # Build command for HKU Airport dataset
        local cmd="./HSLAM -f \"$dataset_path\" -c \"$calib_path\""
        
        if [ "$enable_loop_closure" = true ]; then
            cmd="$cmd -v \"$vocab_path\" --loopclosure=true"
        fi
        
        cmd="$cmd --preset=0 --mode=1 --quiet --nogui=false --outPC=true"
        
        # Add end index if specified
        if [ -n "$end_index" ]; then
            cmd="$cmd --endindex $end_index"
        fi
        
        print_info "Executing: $cmd"
        eval $cmd
        
        exit_code=$?
        print_info "HSLAM completed with exit code: $exit_code"
        
        # Organize results
        destination_directory="$results_directory/hslam-monocular-hku-airport-$timestamp"
        mkdir -p "$destination_directory"
        
        if [ -f "result.txt" ]; then
            mv "result.txt" "$destination_directory/trajectory_hku_monocular_$i.txt"
            print_success "Saved trajectory: $destination_directory/trajectory_hku_monocular_$i.txt"
        else
            print_warning "result.txt not found"
        fi
        
        if [ -f "PC.PCD" ]; then
            mv "PC.PCD" "$destination_directory/pointcloud_hku_monocular_$i.pcd"
        fi
        
        if [ -f "map.pcd" ]; then
            mv "map.pcd" "$destination_directory/map_hku_monocular_$i.pcd"
        fi
        
        if [ -d "logs" ]; then
            mv "logs" "$destination_directory/logs_hku_monocular_$i"
        fi
        
        if [ -d "mats" ]; then
            mv "mats" "$destination_directory/mats_hku_monocular_$i"
        fi
        
        # Move debug images if they exist
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
HSLAM Monocular Trajectory Generation Summary - HKU Airport
==========================================================

Mode: Monocular SLAM (WITHOUT depth integration)
Dataset: HKU Airport (drone footage)
Timestamp: $timestamp
Dataset Path: $dataset_path
Calibration: $calib_path
Loop Closure: $enable_loop_closure

Dataset Information:
- Images: $image_count frames
- Camera: DJI drone camera
- Environment: Airport outdoor scene
- Motion: Aerial drone flight

Generated Files:
- trajectory_hku_monocular_$i.txt  (monocular trajectory)
- debug_images/                   (SLAM debug images, when --save used)
- Point clouds (*.pcd)
- System logs (logs_*)
- Internal matrices (mats_*)

Exit Code: $exit_code

Notes:
- HKU Airport dataset consists of drone camera footage
- Camera intrinsics from provided calibration YAML
- No ground truth available for quantitative evaluation
- Focus on qualitative trajectory smoothness and loop closure detection
EOF
        
        # Create symlink to latest results
        latest_link="$results_directory/latest-hku-monocular"
        ln -sfn "$destination_directory" "$latest_link"
        
        print_success "Results saved to: $destination_directory"
        
        if [ $exit_code -ne 0 ]; then
            print_error "HSLAM failed with exit code: $exit_code"
            exit 1
        fi
    done
    
    # Summary
    print_info "=========================================="
    print_success "HKU Airport monocular trajectory generation completed!"
    print_info "Results directory: $destination_directory"
    print_info "Trajectory file: trajectory_hku_monocular_0.txt"
    print_info "Latest results: $latest_link"
    print_info ""
    print_info "Ready for trajectory analysis!"
}

# Run main function
main