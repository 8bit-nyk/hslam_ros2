#!/bin/bash

########################
# ML Inference Frequency Ablation Study Script
# Tests different ML inference rates and their impact on SLAM performance
########################

# Configuration
DATASET="${1:-07}"  # Default to KITTI 07
FRAME_LIMIT="${2:-200}"  # Default to 200 frames for comprehensive testing

# Ablation study parameters
SNAPSHOT_RATES=(1 2 5 10 20)  # Different rates to test
RESULTS_BASE="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results/ablation_study_$(date +%Y%m%d_%H%M%S)"
SCRIPT_DIR="$(dirname "$0")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  ML INFERENCE FREQUENCY ABLATION STUDY${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo "Dataset: KITTI $DATASET"
    echo "Frame Limit: $FRAME_LIMIT frames"
    echo "Snapshot Rates: ${SNAPSHOT_RATES[*]}"
    echo "Results Directory: $RESULTS_BASE"
    echo ""
}

run_experiment() {
    local rate=$1
    local experiment_name="rate_${rate}"
    local results_dir="$RESULTS_BASE/$experiment_name"
    
    echo -e "${YELLOW}--- Experiment: ML every $rate keyframes ---${NC}"
    
    # Calculate expected coverage
    local expected_coverage=$(echo "scale=1; 100.0 / $rate" | bc)
    echo "Expected ML coverage: ${expected_coverage}%"
    
    # Create results directory
    mkdir -p "$results_dir"
    
    # Set environment variables
    export ML_STRATEGY="snapshot_mode"
    export ML_SNAPSHOT_RATE="$rate"
    
    # Run the experiment
    echo "Running HSLAM with snapshot rate $rate..."
    start_time=$(date +%s)
    
    if [ "$rate" -eq 1 ]; then
        # Baseline: keyframe_only mode
        export ML_STRATEGY="keyframe_only"
        echo "  Using keyframe_only strategy (100% baseline)"
    fi
    
    # Execute the run script
    "$SCRIPT_DIR/hslam_run_kitti_ml_depth.sh" "$DATASET" --endindex "$FRAME_LIMIT" \
        > "$results_dir/execution.log" 2>&1
    
    local exit_code=$?
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    
    # Extract key metrics from log
    echo "Extracting metrics..."
    local ml_inferences=$(grep -c "ML_ABLATION.*RUNNING ML inference" "$results_dir/execution.log" || echo "0")
    local total_keyframes=$(grep "ML_ABLATION_STATS.*keyframes" "$results_dir/execution.log" | tail -1 | grep -o '[0-9]\+ keyframes' | grep -o '[0-9]\+' || echo "0")
    local actual_coverage=$(grep "ML_ABLATION_STATS.*coverage" "$results_dir/execution.log" | tail -1 | grep -o '[0-9.]\+% coverage' | grep -o '[0-9.]\+' || echo "0.0")
    
    # Create summary report
    cat > "$results_dir/summary.txt" << EOF
ML Inference Ablation Study Results
==================================
Experiment: $experiment_name
Dataset: KITTI $DATASET
Frame Limit: $FRAME_LIMIT
Snapshot Rate: Every $rate keyframes
Strategy: $ML_STRATEGY

TIMING:
- Execution Time: ${duration}s
- Exit Code: $exit_code

ML STATISTICS:
- Expected Coverage: ${expected_coverage}%
- Actual Coverage: ${actual_coverage}%
- Total Keyframes: $total_keyframes
- ML Inferences: $ml_inferences
- Inference Reduction: $(echo "scale=1; 100.0 - $actual_coverage" | bc)%

STATUS: $([ $exit_code -eq 0 ] && echo "SUCCESS" || echo "FAILED")
EOF
    
    # Report results
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ Experiment completed successfully${NC}"
        echo "  Duration: ${duration}s"
        echo "  ML Coverage: ${actual_coverage}% (expected: ${expected_coverage}%)"
        echo "  Results: $results_dir/summary.txt"
    else
        echo -e "${RED}✗ Experiment failed (exit code: $exit_code)${NC}"
        echo "  Log: $results_dir/execution.log"
    fi
    
    echo ""
}

analyze_results() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  ABLATION STUDY ANALYSIS${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # Create comprehensive analysis
    local analysis_file="$RESULTS_BASE/ablation_analysis.txt"
    
    echo "Ablation Study Analysis - $(date)" > "$analysis_file"
    echo "Dataset: KITTI $DATASET" >> "$analysis_file"
    echo "Frame Limit: $FRAME_LIMIT frames" >> "$analysis_file"
    echo "========================================" >> "$analysis_file"
    echo "" >> "$analysis_file"
    
    printf "%-12s %-15s %-15s %-15s %-12s %-10s\n" "Rate" "Strategy" "Expected%" "Actual%" "Duration(s)" "Status" | tee -a "$analysis_file"
    printf "%-12s %-15s %-15s %-15s %-12s %-10s\n" "----" "--------" "---------" "-------" "---------" "------" | tee -a "$analysis_file"
    
    for rate in "${SNAPSHOT_RATES[@]}"; do
        local experiment_name="rate_${rate}"
        local results_dir="$RESULTS_BASE/$experiment_name"
        local summary_file="$results_dir/summary.txt"
        
        if [ -f "$summary_file" ]; then
            local strategy=$(grep "Strategy:" "$summary_file" | cut -d' ' -f2)
            local expected=$(grep "Expected Coverage:" "$summary_file" | grep -o '[0-9.]\+')
            local actual=$(grep "Actual Coverage:" "$summary_file" | grep -o '[0-9.]\+')
            local duration=$(grep "Execution Time:" "$summary_file" | grep -o '[0-9]\+')
            local status=$(grep "STATUS:" "$summary_file" | cut -d' ' -f2)
            
            printf "%-12s %-15s %-15s %-15s %-12s %-10s\n" "$rate" "$strategy" "${expected}%" "${actual}%" "${duration}s" "$status" | tee -a "$analysis_file"
        else
            printf "%-12s %-15s %-15s %-15s %-12s %-10s\n" "$rate" "N/A" "N/A" "N/A" "N/A" "MISSING" | tee -a "$analysis_file"
        fi
    done
    
    echo "" | tee -a "$analysis_file"
    echo "Individual result files available in: $RESULTS_BASE/" | tee -a "$analysis_file"
    echo "Complete analysis saved to: $analysis_file"
}

# Main execution
main() {
    print_header
    
    # Validate prerequisites
    if [ ! -f "$SCRIPT_DIR/hslam_run_kitti_ml_depth.sh" ]; then
        echo -e "${RED}ERROR: KITTI run script not found${NC}"
        exit 1
    fi
    
    if ! command -v bc &> /dev/null; then
        echo -e "${RED}ERROR: 'bc' calculator not found. Please install: sudo apt install bc${NC}"
        exit 1
    fi
    
    # Create base results directory
    mkdir -p "$RESULTS_BASE"
    
    # Run all experiments
    for rate in "${SNAPSHOT_RATES[@]}"; do
        run_experiment "$rate"
        sleep 3  # Brief pause between experiments
    done
    
    # Analyze and summarize results
    analyze_results
    
    echo -e "${GREEN}Ablation study completed!${NC}"
    echo "Check results in: $RESULTS_BASE/"
}

# Run main function
main "$@"