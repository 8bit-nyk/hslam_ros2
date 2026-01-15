#!/bin/bash

########################
# HSLAM ML Depth Mode Script - HKU Airport Dataset
# Generates trajectory WITH ML depth integration
########################

# Configuration
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
repetitions=1

# Dataset paths
dataset_path="/home/aub/datasets/HKU_AIRPORT/images"
calib_path="/home/aub/datasets/HKU_AIRPORT/camera_hku.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"

# ML model configuration
ml_model_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/models/metric3d-vit-small/onnx/model.onnx"
ml_strategy="keyframe_only"
ml_benchmark=""  # Set to non-empty to enable benchmarking

# GPU configuration
ml_gpu_enabled="true"      # Enable GPU acceleration (44ms inference vs 650ms CPU)
ml_fp16_enabled=""          # Enable FP16 optimization (leave empty to disable)
ml_gpu_device="0"           # GPU device ID
ml_gpu_memory="2048"        # GPU memory limit in MB

# Processing configuration
end_index="" # Process limited frames for testing (set to "" for full trajectory)
quiet_mode=""  # Set to "--quiet" to reduce HSLAM console output for performance
ml_init_enabled="true"  # Enable ML metric scale initialization by default
output_log="hslam_hku_ml_depth_run.log"

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

print_ml_info() {
    echo -e "\033[1;36m[ML]\033[0m $1"
}

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "HKU Airport Dataset - ML Depth Mode"
    echo ""
    echo "Options:"
    echo "  --quiet              Enable quiet mode (minimal console output for performance)"
    echo "  --endindex N         Process only first N frames"
    echo "  --no-ml-init         Disable ML metric scale initialization"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                           # Run full HKU Airport sequence with ML depth"
    echo "  $0 --quiet                   # Quiet mode (minimal output)"
    echo "  $0 --endindex 100            # Process first 100 frames only"
    echo "  $0 --no-ml-init              # Test without metric scale initialization"
    echo ""
    echo "This script generates trajectory WITH ML depth integration (Metric3D)"
    echo "Dataset: HKU Airport (drone footage)"
    echo "Results saved in: $results_directory/hslam-ml-depth-hku-airport-{timestamp}/"
    echo ""
    echo "ML Configuration:"
    echo "  Model: $ml_model_path"
    echo "  Strategy: $ml_strategy"
    echo "  Input Size: 518x518"
    if [ -n "$ml_gpu_enabled" ]; then
        echo "  Device: GPU (CUDA) - 44ms inference"
    else
        echo "  Device: CPU - 650ms inference (slow)"
    fi
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
        --no-ml-init)
            ml_init_enabled="false"
            shift
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
    print_info "HSLAM ML Depth Mode - HKU Airport Dataset"
    print_info "Dataset: HKU Airport (drone footage)"
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
    
    # Check ML model
    if [ ! -f "$ml_model_path" ]; then
        print_error "ML model file not found: $ml_model_path"
        exit 1
    fi
    
    # Verify model file is readable and show size
    if [ ! -r "$ml_model_path" ]; then
        print_error "ML model not readable: $ml_model_path"
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
    print_info "Updated LD_LIBRARY_PATH for ONNX Runtime"
    
    # Run HSLAM with ML depth
    echo "Running HSLAM with ML depth integration on HKU Airport dataset..."
    if [ -n "$ml_gpu_enabled" ]; then
        print_ml_info "GPU acceleration ENABLED (40ms inference)"
    else
        print_warning "GPU acceleration DISABLED (650ms CPU inference)"
    fi
    
    # Build command with conditional GPU flags
    cmd="$build_directory_path/HSLAM \
        --files $dataset_path \
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
    
    # Add ML init flag based on configuration
    if [ "$ml_init_enabled" = "true" ]; then
        cmd="$cmd --ml-init=true"
    else
        cmd="$cmd --ml-init=false"
    fi
    
    # Show final command for debugging
    print_info "Executing command:"
    echo "  $cmd"
    echo ""
    
    # Execute and log
    eval "$cmd 2>&1" | tee $output_log
    
    exit_code=${PIPESTATUS[0]}
    
    # Analyze ML performance from log
    if [ -f "$output_log" ]; then
        print_ml_info "ML Performance Analysis:"
        
        # Check for ML initialization success
        if grep -q "Metric3D model validation: PASSED" "$output_log"; then
            print_success "✓ ML model validation: PASSED"
        else
            print_error "✗ ML model validation: FAILED"
        fi
        
        # Check for performance benchmark
        if grep -q "Average inference time:" "$output_log"; then
            avg_time=$(grep "Average inference time:" "$output_log" | awk '{print $4}')
            print_ml_info "✓ Average inference time: ${avg_time}ms"
            
            if echo "$avg_time" | awk '{if($1 > 50) exit 1; else exit 0}'; then
                print_success "✓ Performance: ACCEPTABLE for real-time"
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
    fi
    
    # Organize results
    destination_directory="$results_directory/hslam-ml-depth-hku-airport-$timestamp"
    mkdir -p "$destination_directory"
    
    # Move trajectory file
    if [ -f "result.txt" ]; then
        mv "result.txt" "$destination_directory/trajectory_hku_ml_depth_0.txt"
        print_success "Saved trajectory: $destination_directory/trajectory_hku_ml_depth_0.txt"
    else
        print_warning "result.txt not found"
    fi
    
    # Move other output files
    if [ -f "PC.PCD" ]; then
        mv "PC.PCD" "$destination_directory/pointcloud_hku_ml_depth_0.pcd"
    fi
    
    if [ -f "map.pcd" ]; then
        mv "map.pcd" "$destination_directory/map_hku_ml_depth_0.pcd"
    fi
    
    if [ -d "logs" ]; then
        mv "logs" "$destination_directory/logs_hku_ml_depth_0"
    fi
    
    if [ -d "mats" ]; then
        mv "mats" "$destination_directory/mats_hku_ml_depth_0"
    fi
    
    # Move and analyze log file
    if [ -f "$output_log" ]; then
        mv "$output_log" "$destination_directory/run_log_hku_ml_depth_0.txt"
        
        # Extract ML statistics
        echo "=== ML Depth Integration Statistics ===" > "$destination_directory/ml_stats_0.txt"
        grep -E "(ML depth|Metric3D|inference time|benchmark)" "$destination_directory/run_log_hku_ml_depth_0.txt" >> "$destination_directory/ml_stats_0.txt"
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
        ml_depth_count=$(find "$destination_directory/debug_images" -name "ml_depth_*.png" 2>/dev/null | wc -l)
        if [ "$ml_depth_count" -gt 0 ]; then
            print_success "Saved $ml_depth_count ML depth samples to: $destination_directory/debug_images/"
        fi
    fi
    
    # Create run summary
    cat > "$destination_directory/run_summary.txt" << EOF
HSLAM ML Depth Trajectory Generation Summary - HKU Airport
==========================================================

Mode: ML Depth SLAM (WITH Metric3D integration)
Dataset: HKU Airport (drone footage)
Timestamp: $timestamp
Dataset Path: $dataset_path
Calibration: $calib_path
Loop Closure: $enable_loop_closure

ML Configuration:
- Model: $(basename $ml_model_path)
- Strategy: $ml_strategy
- Input Size: 518x518
- Device: $([ -n "$ml_gpu_enabled" ] && echo "GPU (CUDA) - 44ms inference" || echo "CPU - 650ms inference")
- GPU Enabled: $([ -n "$ml_gpu_enabled" ] && echo "Yes" || echo "No")
- FP16 Mode: $([ -n "$ml_fp16_enabled" ] && echo "Yes" || echo "No")

Generated Files:
- trajectory_hku_ml_depth_0.txt  (ML depth trajectory)
- run_log_hku_ml_depth_0.txt    (Complete execution log)
- ml_stats_0.txt               (ML performance statistics)
- debug_images/                (ML depth maps & SLAM debug images)
- Point clouds (*.pcd)
- System logs (logs_*)
- Internal matrices (mats_*)

Exit Code: $exit_code

Notes:
- HKU Airport dataset consists of drone camera footage
- Camera intrinsics estimated from calibration YAML
- No ground truth available for quantitative evaluation
- Focus on qualitative trajectory smoothness and ML integration
EOF
    
    # Create symlink to latest results
    latest_link="$results_directory/latest-hku-ml-depth"
    ln -sfn "$destination_directory" "$latest_link"
    
    print_success "Results saved to: $destination_directory"
    
    if [ $exit_code -ne 0 ]; then
        print_warning "HSLAM completed with non-zero exit code: $exit_code"
        print_info "Check run_log_hku_ml_depth_0.txt for details"
    fi
}

# Run main function
main