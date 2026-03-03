#!/bin/bash

########################
# HSLAM EuRoC ML Depth Mode Script
# Generates trajectory WITH ML depth integration
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

# ML model configuration
ml_model_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/models/metric3d-vit-small/onnx/model.onnx"
ml_strategy="keyframe_only"
ml_benchmark=""  # Set to non-empty to enable benchmarking

# GPU configuration
ml_gpu_enabled="true"      # Enable GPU acceleration (44ms inference vs 650ms CPU)
ml_fp16_enabled=""          # Enable FP16 optimization (leave empty to disable)
ml_gpu_device="0"           # GPU device ID
ml_gpu_memory="2048"        # GPU memory limit in MB (increased from 2GB to 6GB for better performance)

# Processing configuration
end_index="" # Process limited frames for testing (set to "" for full trajectory)
quiet_mode=""  # Set to "--quiet" to reduce HSLAM console output for performance
output_log="hslam_euroc_ml_depth_run.log"

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
    echo "Usage: $0 [DATASET_NAME] [OPTIONS]"
    echo ""
    echo "DATASET_NAME: EuRoC MAV dataset name (default: MH_01_easy)"
    echo ""
    echo "Options:"
    echo "  --quiet              Enable quiet mode (minimal console output for performance)"
    echo "  --endindex N         Process only first N frames"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Available datasets:"
    ls -1 "$DATASET_BASE" 2>/dev/null | grep -E "^(MH|V)_[0-9]+_" | sed 's/^/  - /' || echo "  (Dataset directory not found)"
    echo ""
    echo "Examples:"
    echo "  $0                           # Use default dataset (MH_01_easy)"
    echo "  $0 MH_02_easy                # Use MH_02_easy dataset"
    echo "  $0 V1_01_easy --quiet        # Quiet mode (minimal output)"
    echo "  $0 MH_01_easy --endindex 50  # Process first 50 frames only"
    echo ""
    echo "This script generates trajectory WITH ML depth integration (Metric3D)"
    echo "Results saved in: $results_directory/hslam-euroc-ml-depth-{dataset}-{timestamp}/"
    echo ""
    echo "EuRoC Dataset Structure:"
    echo "  Images: {dataset}/mav0/cam0/data/*.png"
    echo "  Timestamps: {dataset}/mav0/cam0/times.txt"
    echo "  Calibration: {dataset}/mav0/cam0/camera.txt"
    echo "  Ground Truth: {dataset}/mav0/state_groundtruth_estimate0/data.csv"
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
    print_info "HSLAM EuRoC ML Depth Mode (WITH Metric3D Integration)"
    print_info "Dataset: $DATASET_NAME"
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
    
    # Check ML model (enhanced debugging)
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
    
    # Debug library paths and ONNX Runtime linkage
    print_info "Library Path Debug:"
    echo "  Current LD_LIBRARY_PATH: ${LD_LIBRARY_PATH:-"(not set)"}"
    echo "  ONNX Runtime library check:"
    if ldd ./HSLAM | grep -q onnxruntime; then
        ldd ./HSLAM | grep onnx | head -5
    else
        print_warning "  No ONNX Runtime libraries detected in HSLAM linkage"
    fi
    
    # Set proper library path if needed
    export LD_LIBRARY_PATH="$HOME/Dev/hslam_ros2_ws/src/HSLAM/Thirdparty/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
    print_info "  Updated LD_LIBRARY_PATH for ONNX Runtime"
    
    # Run HSLAM with ML depth
    echo "Running HSLAM with ML depth integration on EuRoC dataset..."
    if [ -n "$ml_gpu_enabled" ]; then
        print_ml_info "GPU acceleration ENABLED (40ms inference)"
    else
        print_warning "GPU acceleration DISABLED (650ms CPU inference)"
    fi
    
    # Build command with conditional GPU flags
    cmd="$build_directory_path/HSLAM \
        --files $dataset_path/cam0/data \
        --calib $calib_path \
        --vocab $vocab_path \
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
    cmd="$cmd --loopclosure --nogui=true --nolog"  # Back to headless mode due to X11 error
    [ -n "$end_index" ] && cmd="$cmd --endindex $end_index"
    
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
        else
            print_warning "⚠ Performance benchmark not found in log"
        fi
        
        # Check for ML depth utilization
        if grep -q "ML depth available" "$output_log"; then
            print_success "✓ ML depth integration: ACTIVE"
        else
            print_warning "⚠ ML depth integration: NOT DETECTED"
        fi
    fi
    
    # Organize results
    destination_directory="$results_directory/hslam-euroc-ml-depth-$DATASET_NAME-$timestamp"
    mkdir -p "$destination_directory"
    
    # Move trajectory file
    if [ -f "result.txt" ]; then
        mv "result.txt" "$destination_directory/trajectory_ml_depth_0.txt"
        print_success "Saved trajectory: $destination_directory/trajectory_ml_depth_0.txt"
    else
        print_warning "result.txt not found"
    fi
    
    # Move other output files
    if [ -f "PC.PCD" ]; then
        mv "PC.PCD" "$destination_directory/pointcloud_ml_depth_0.pcd"
    fi
    
    if [ -f "map.pcd" ]; then
        mv "map.pcd" "$destination_directory/map_ml_depth_0.pcd"
    fi
    
    if [ -d "logs" ]; then
        mv "logs" "$destination_directory/logs_ml_depth_0"
    fi
    
    if [ -d "mats" ]; then
        mv "mats" "$destination_directory/mats_ml_depth_0"
    fi
    
    # Move and analyze log file
    if [ -f "$output_log" ]; then
        mv "$output_log" "$destination_directory/run_log_ml_depth_0.txt"
    fi
    
    # Move ML benchmark images if they exist
    if [ -f "ml_benchmark_input.png" ]; then
        mkdir -p "$destination_directory/ml_benchmark"
        mv ml_benchmark_*.png "$destination_directory/ml_benchmark/"
        print_success "Saved ML benchmark images to: $destination_directory/ml_benchmark/"
    fi
    
    # Move debug images (including ML depth maps) if they exist
    # Check both build directory and main HSLAM directory for images_out
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
            print_info "  • Colorized visualizations: ml_depth_*.png"
            print_info "  • Raw depth data: ml_depth_raw_*.exr"
        fi
        # Count other debug images
        other_debug_count=$(find "$destination_directory/debug_images" -name "predicted_*.png" 2>/dev/null | wc -l)
        if [ "$other_debug_count" -gt 0 ]; then
            print_info "  • SLAM debug images: $other_debug_count predicted frames"
        fi
    fi
    
    # Create symlink to latest results
    latest_link="$results_directory/latest-euroc-ml-depth"
    ln -sfn "$destination_directory" "$latest_link"
    
    print_success "Results saved to: $destination_directory"
    
    if [ $exit_code -ne 0 ]; then
        print_warning "HSLAM completed with non-zero exit code: $exit_code"
        print_info "Check run_log_ml_depth_0.txt for details"
    else
        print_success "HSLAM EuRoC ML depth integration completed successfully!"
        print_info "Ready for evo evaluation:"
        print_info "  evo_ape euroc $dataset_path/state_groundtruth_estimate0/data.csv trajectory_ml_depth_0.txt --plot"
    fi
}

# Run main function
main