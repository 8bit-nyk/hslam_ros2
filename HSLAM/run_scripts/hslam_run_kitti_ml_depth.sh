#!/bin/bash

########################
# HSLAM KITTI ML Depth Mode Script
# Generates trajectory WITH ML depth integration on KITTI dataset
# Based on proven EuRoC ML depth integration script
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Default dataset (can be overridden via command line)
DEFAULT_DATASET="07"
DATASET_NUM="${1:-$DEFAULT_DATASET}"

# Dataset paths
DATASET_BASE="$HOME/datasets/KITTI"
dataset_path="$DATASET_BASE/$DATASET_NUM"
image_path="$dataset_path/image_2"  # RGB images from KITTI
calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/Kitti/Kitti04-12.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"

# ML model configuration (same as EuRoC/TUM)
ml_model_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/models/metric3d-vit-small/onnx/model.onnx"
ml_strategy="keyframe_only"
ml_benchmark=""  # Set to non-empty to enable benchmarking

# GPU configuration (same as production EuRoC/TUM)
ml_gpu_enabled="true"      # Enable GPU acceleration (37ms inference vs 650ms CPU)
ml_fp16_enabled=""          # Enable FP16 optimization (leave empty to disable)
ml_gpu_device="0"           # GPU device ID
ml_gpu_memory="2048"        # GPU memory limit in MB (6GB for KITTI's larger images)

# Processing configuration
end_index="" # Process limited frames for testing (set to "" for full trajectory)
quiet_mode=""  # Set to "--quiet" to reduce HSLAM console output for performance
output_log="hslam_kitti_ml_depth_run.log"

# Function to print colored output (same as EuRoC script)
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

print_ml_info() {
    echo -e "\033[1;36m[ML]\033[0m $1"
}

# Help function
show_help() {
    echo "Usage: $0 [DATASET_NUM] [OPTIONS]"
    echo ""
    echo "DATASET_NUM: KITTI sequence number (default: 07)"
    echo ""
    echo "Options:"
    echo "  --quiet              Enable quiet mode (minimal console output for performance)"
    echo "  --endindex N         Process only first N frames"
    echo "  --benchmark          Enable ML benchmarking"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Available datasets:"
    ls -1 "$DATASET_BASE" 2>/dev/null | grep -E "^[0-9]+$" | sed 's/^/  - KITTI /' || echo "  (Dataset directory not found)"
    echo ""
    echo "Examples:"
    echo "  $0                           # Use default dataset (KITTI 07)"
    echo "  $0 05                        # Use KITTI 05 dataset"
    echo "  $0 07 --quiet                # Quiet mode (minimal output)"
    echo "  $0 07 --endindex 100         # Process first 100 frames only"
    echo "  $0 07 --benchmark            # Enable ML performance benchmarking"
    echo ""
    echo "This script generates trajectory WITH ML depth integration (Metric3D)"
    echo "Results saved in: $results_directory/hslam-kitti-ml-depth-{dataset}-{timestamp}/"
    echo ""
    echo "KITTI Dataset Structure:"
    echo "  Images: {dataset}/image_2/*.png (RGB color images 1226x370)"
    echo "  Timestamps: {dataset}/image_2/times.txt (converted from KITTI format)"
    echo "  Calibration: HSLAM/misc/Kitti/Kitti04-12.txt"
    echo "  Ground Truth: {dataset}/groundtruth.txt"
    echo ""
    echo "ML Configuration:"
    echo "  Model: $ml_model_path"
    echo "  Strategy: $ml_strategy"
    echo "  Input Size: 616x1064 (ViT model with aspect ratio preservation)"
    if [ -n "$ml_gpu_enabled" ]; then
        echo "  Device: GPU (CUDA) - 37ms inference"
    else
        echo "  Device: CPU - 650ms inference (slow)"
    fi
    echo ""
    echo "Prerequisites:"
    echo "  1. Convert timestamps: python3 scripts/convert_kitti_times.py \\"
    echo "                         $dataset_path/times.txt \\"
    echo "                         $image_path/times.txt"
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
        --benchmark)
            ml_benchmark="enabled"
            shift
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
        *)
            # First non-option argument is dataset number
            if [ -z "$DATASET_NUM_SET" ]; then
                DATASET_NUM="$1"
                DATASET_NUM_SET=1
                # Update paths with new dataset number
                dataset_path="$DATASET_BASE/$DATASET_NUM"
                image_path="$dataset_path/image_2"
            else
                echo "Error: Multiple dataset numbers specified"
                exit 1
            fi
            shift
            ;;
    esac
done

# Main execution function
main() {
    print_info "HSLAM KITTI ML Depth Mode (WITH Metric3D Integration)"
    print_info "Dataset: KITTI $DATASET_NUM"
    print_ml_info "Model: $(basename $ml_model_path)"
    print_ml_info "Strategy: $ml_strategy"
    print_info "==============================================="
    
    # Check prerequisites
    print_info "Checking prerequisites..."
    
    # Check build directory
    if [ ! -d "$build_directory_path" ]; then
        print_error "Build directory not found: $build_directory_path"
        exit 1
    fi
    
    # Check dataset directory
    if [ ! -d "$dataset_path" ]; then
        print_error "Dataset directory not found: $dataset_path"
        print_info "Available KITTI datasets:"
        ls -1 "$DATASET_BASE" | grep -E "^[0-9]+$" | sed 's/^/  - KITTI /'
        exit 1
    fi
    
    # Check RGB images directory
    if [ ! -d "$image_path" ]; then
        print_error "RGB images directory not found: $image_path"
        print_info "Expected structure: $dataset_path/image_2/ (RGB color images)"
        exit 1
    fi
    
    # Check calibration file
    if [ ! -f "$calib_path" ]; then
        print_error "Calibration file not found: $calib_path"
        exit 1
    fi
    
    # Check converted timestamps file (DISABLED for testing original KITTI format)
    times_file="$image_path/times.txt"
    if [ ! -f "$times_file" ]; then
        print_warning "Converted times file not found: $times_file"
        print_info "Will use original KITTI times.txt format: $dataset_path/times.txt"
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
        if echo "$image_info" | grep -q "RGB"; then
            print_success "✓ RGB images detected: $(echo "$image_info" | grep -o '[0-9]* x [0-9]*')"
        else
            print_warning "⚠ Images may not be RGB format: $image_info"
        fi
    fi
    
    # Check ML model (same as EuRoC script)
    if [ ! -f "$ml_model_path" ]; then
        print_error "ML model file not found: $ml_model_path"
        print_info "Checking parent directory:"
        ls -la "$(dirname "$ml_model_path")" 2>/dev/null || print_error "Parent directory doesn't exist"
        exit 1
    fi
    
    # Verify model file is readable and show size
    if [ ! -r "$ml_model_path" ]; then
        print_error "ML model not readable: $ml_model_path"
        ls -la "$ml_model_path"
        exit 1
    fi
    
    model_size=$(stat -c%s "$ml_model_path" 2>/dev/null)
    print_info "Model file verified: $(numfmt --to=iec-i --suffix=B $model_size 2>/dev/null || echo "$model_size bytes")"
    
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
    
    # Set proper library path for ONNX Runtime
    export LD_LIBRARY_PATH="$HOME/Dev/hslam_ros2_ws/src/HSLAM/Thirdparty/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
    print_info "Library path configured for ONNX Runtime"
    
    # Run HSLAM with ML depth
    print_info "Running HSLAM with ML depth integration on KITTI dataset..."
    if [ -n "$ml_gpu_enabled" ]; then
        print_ml_info "GPU acceleration ENABLED (37ms inference)"
    else
        print_warning "GPU acceleration DISABLED (650ms CPU inference)"
    fi
    
    # Build command (based on EuRoC template)
    cmd="$build_directory_path/HSLAM \
        --files $image_path \
        --calib $calib_path \
        --vocab $vocab_path \
        --save \
        --colour \
        --ml-depth \
        --ml-model $ml_model_path \
        --ml-strategy $ml_strategy"
    
    # Add GPU flags if enabled
    if [ -n "$ml_gpu_enabled" ]; then
        cmd="$cmd --ml-gpu"
        [ -n "$ml_fp16_enabled" ] && cmd="$cmd --ml-fp16"
        [ -n "$ml_gpu_device" ] && cmd="$cmd --ml-gpu-device $ml_gpu_device"
        [ -n "$ml_gpu_memory" ] && cmd="$cmd --ml-gpu-memory $ml_gpu_memory"
    fi
    
    # Add other flags
    [ -n "$ml_benchmark" ] && cmd="$cmd --ml-benchmark"
    [ -n "$quiet_mode" ] && cmd="$cmd $quiet_mode"
    cmd="$cmd --loopclosure --nogui=false"
    [ -n "$end_index" ] && cmd="$cmd --endindex $end_index"
    
    # Execute and log
    print_info "Executing: $(echo $cmd | tr -s ' ')"
    eval "$cmd 2>&1" | tee $output_log
    
    exit_code=${PIPESTATUS[0]}
    
    # Analyze results (same as EuRoC script)
    if [ -f "$output_log" ]; then
        print_ml_info "ML Performance Analysis:"
        
        # Check for ML initialization success
        if grep -q "Metric3D model validation: PASSED" "$output_log"; then
            print_success "✓ ML model validation: PASSED"
        else
            print_error "✗ ML model validation: FAILED"
        fi
        
        # Check for performance metrics
        if grep -q "Average inference time:" "$output_log"; then
            avg_time=$(grep "Average inference time:" "$output_log" | tail -1 | awk '{print $4}')
            print_ml_info "✓ Average inference time: ${avg_time}ms"
            
            if echo "$avg_time" | awk '{if($1 > 50) exit 1; else exit 0}'; then
                print_success "✓ Performance: ACCEPTABLE for real-time (< 50ms)"
            else
                print_warning "⚠ Performance: SLOW (may impact real-time operation)"
            fi
        fi
        
        # Check for ML depth utilization
        if grep -q "ML depth available" "$output_log"; then
            print_success "✓ ML depth integration: ACTIVE"
        else
            print_warning "⚠ ML depth integration: NOT DETECTED"
        fi
        
        # Check for system stability
        if grep -q "HSLAM finished" "$output_log" || [ $exit_code -eq 0 ]; then
            print_success "✓ System stability: COMPLETE PROCESSING"
        else
            print_warning "⚠ System may have terminated early"
        fi
    fi
    
    # Organize results (same structure as EuRoC)
    destination_directory="$results_directory/hslam-kitti-ml-depth-$DATASET_NUM-$timestamp"
    mkdir -p "$destination_directory"
    
    # Move trajectory file
    if [ -f "result.txt" ]; then
        mv "result.txt" "$destination_directory/trajectory_ml_depth_0.txt"
        print_success "Saved trajectory: $destination_directory/trajectory_ml_depth_0.txt"
    else
        print_warning "result.txt not found"
    fi
    
    # Move other output files
    [ -f "PC.PCD" ] && mv "PC.PCD" "$destination_directory/pointcloud_ml_depth_0.pcd"
    [ -f "map.pcd" ] && mv "map.pcd" "$destination_directory/map_ml_depth_0.pcd"
    [ -d "logs" ] && mv "logs" "$destination_directory/logs_ml_depth_0"
    [ -d "mats" ] && mv "mats" "$destination_directory/mats_ml_depth_0"
    
    # Move and analyze log file
    if [ -f "$output_log" ]; then
        mv "$output_log" "$destination_directory/run_log_ml_depth_0.txt"
        
        # Extract ML statistics
        echo "=== KITTI ML Depth Integration Statistics ===" > "$destination_directory/ml_stats_0.txt"
        grep -E "(ML depth|Metric3D|inference time|benchmark)" "$destination_directory/run_log_ml_depth_0.txt" >> "$destination_directory/ml_stats_0.txt"
    fi
    
    # Move ML debug images if they exist
    images_out_path=""
    if [ -d "images_out" ]; then
        images_out_path="images_out"
    elif [ -d "../images_out" ]; then
        images_out_path="../images_out"
    fi
    
    if [ -n "$images_out_path" ]; then
        mv "$images_out_path" "$destination_directory/debug_images"
        # Count ML depth samples saved
        ml_depth_count=$(find "$destination_directory/debug_images" -name "ml_depth_*.png" 2>/dev/null | wc -l)
        if [ "$ml_depth_count" -gt 0 ]; then
            print_success "Saved $ml_depth_count ML depth samples to: $destination_directory/debug_images/"
        fi
    fi
    
    # Create run summary
    cat > "$destination_directory/run_summary.txt" << EOF
HSLAM KITTI ML Depth Trajectory Generation Summary
==================================================

Mode: ML Depth SLAM (WITH Metric3D integration)
Dataset: KITTI $DATASET_NUM
Timestamp: $timestamp
Dataset Path: $dataset_path
RGB Images: $image_path/ (1226x370 color images)
Timestamps: $times_file (converted from KITTI format)
Calibration: $calib_path
Ground Truth: $dataset_path/groundtruth.txt (if available)
Loop Closure: $enable_loop_closure

KITTI Dataset Info:
- Total Images: $image_count
- Camera: Color camera (RGB)
- Original Resolution: 1226x370
- Undistorted Resolution: 1216x368 (per calibration)
- Rate: ~10 Hz

ML Configuration:
- Model: $(basename $ml_model_path)
- Strategy: $ml_strategy
- Input Size: 616x1064 (ViT model with aspect ratio preservation)
- Device: $([ -n "$ml_gpu_enabled" ] && echo "GPU (CUDA) - 37ms inference" || echo "CPU - 650ms inference")
- GPU Enabled: $([ -n "$ml_gpu_enabled" ] && echo "Yes" || echo "No")
- FP16 Mode: $([ -n "$ml_fp16_enabled" ] && echo "Yes" || echo "No")

Generated Files:
- trajectory_ml_depth_0.txt  (ML depth trajectory in TUM format)
- run_log_ml_depth_0.txt    (Complete execution log)
- ml_stats_0.txt            (ML performance statistics)
- debug_images/             (ML depth maps & SLAM debug images)
- Point clouds (*.pcd)
- System logs (logs_*)
- Internal matrices (mats_*)

Exit Code: $exit_code

Validation Checklist:
- [ ] ML model loads successfully
- [ ] Model validation passes
- [ ] Performance benchmark completes
- [ ] ML depth integration active
- [ ] Trajectory file generated
- [ ] Performance acceptable for real-time
- [ ] KITTI RGB images processed correctly
- [ ] Aspect ratio preservation working

Evaluation:
To evaluate trajectory against ground truth (if available), use:
  evo_ape kitti $dataset_path/groundtruth.txt trajectory_ml_depth_0.txt --plot

Notes:
- KITTI dataset uses wider aspect ratio (1226x370) than TUM/EuRoC
- ML inference time should be < 50ms for real-time performance
- Metric3D preprocessing handles aspect ratio preservation automatically
- KITTI timestamps converted from relative to TUM format
- RGB images provide richer information for ML depth estimation
EOF
    
    # Create symlink to latest results
    latest_link="$results_directory/latest-kitti-ml-depth"
    ln -sfn "$destination_directory" "$latest_link"
    
    print_success "Results saved to: $destination_directory"
    
    if [ $exit_code -ne 0 ]; then
        print_warning "HSLAM completed with non-zero exit code: $exit_code"
        print_info "Check run_log_ml_depth_0.txt for details"
    else
        print_success "HSLAM KITTI ML depth integration completed successfully!"
        print_info "Ready for evaluation (if ground truth available):"
        print_info "  evo_ape kitti $dataset_path/groundtruth.txt trajectory_ml_depth_0.txt --plot"
    fi
}

# Run main function
main