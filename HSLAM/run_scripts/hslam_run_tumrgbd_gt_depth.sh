#!/bin/bash
########################
# HSLAM GT Depth Validation Mode (Phase B — research only)
# Uses Kinect GT depth from TUM-RGBD associations.txt in place of Metric3D.
# NOT a production mode. See docs/gt_depth_validation/PLAN.md.
########################

build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"

DEFAULT_DATASET="freiburg1_room"
DATASET_NAME="${1:-$DEFAULT_DATASET}"
shift 2>/dev/null

DATASET_BASE="$HOME/datasets/TUM_RGBD"
dataset_path="$DATASET_BASE/rgbd_dataset_$DATASET_NAME"
calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/run_scripts/camera_freiburg1.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"
associations_path="$dataset_path/associations.txt"

# Mode: ml | gt | none  (default gt for this script)
depth_source="gt"
end_index=""
quiet_mode=""
output_log="hslam_gt_depth_run.log"

# Parse options (positional dataset name already consumed)
while [[ $# -gt 0 ]]; do
    case $1 in
        --depth-source)
            depth_source="$2"
            shift 2
            ;;
        --endindex)
            end_index="$2"
            shift 2
            ;;
        --quiet)
            quiet_mode="--quiet"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [DATASET_NAME] [--depth-source ml|gt|none] [--endindex N] [--quiet]"
            echo "  Requires dataset with associations.txt (TUM-RGBD format)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"; exit 1
            ;;
    esac
done

if [ ! -d "$dataset_path" ]; then
    echo "ERROR: dataset not found: $dataset_path"; exit 1
fi
if [ ! -f "$associations_path" ]; then
    echo "ERROR: associations.txt not found: $associations_path"; exit 1
fi
if [ ! -f "$calib_path" ]; then
    echo "ERROR: calibration file not found: $calib_path"; exit 1
fi

mkdir -p "$results_directory"
timestamp=$(date +"%Y%m%d_%H%M%S")
cd "$build_directory_path"

export LD_LIBRARY_PATH="$HOME/Dev/hslam_ros2_ws/src/HSLAM/Thirdparty/onnxruntime/lib:${LD_LIBRARY_PATH:-}"

cmd="$build_directory_path/HSLAM \
    --files $dataset_path \
    --calib $calib_path \
    --vocab $vocab_path \
    --associations $associations_path \
    --colour \
    --depth-source $depth_source \
    --loopclosure \
    --nogui=true \
    --nolog"

# In GT/NONE mode, ML inference is fully skipped inside FullSystem (gated on depthSource).
# In ML mode via this script, still enable --ml-depth so the ML processor initializes.
if [ "$depth_source" = "ml" ]; then
    ml_model_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/models/metric3d-vit-small/onnx/model.onnx"
    cmd="$cmd --ml-depth --ml-model $ml_model_path --ml-strategy keyframe_only --ml-gpu --ml-gpu-memory 2048"
fi

[ -n "$quiet_mode" ]  && cmd="$cmd $quiet_mode"
[ -n "$end_index" ]   && cmd="$cmd --endindex $end_index"

echo "[HSLAM GT Depth Validation] depth-source=$depth_source  dataset=$DATASET_NAME  endindex=${end_index:-all}"
echo "Command: $cmd"
echo ""

eval "$cmd 2>&1" | tee "$output_log"
exit_code=${PIPESTATUS[0]}

dst="$results_directory/hslam-gt_${depth_source}-$DATASET_NAME-$timestamp"
mkdir -p "$dst"
[ -f "result.txt" ] && mv "result.txt" "$dst/trajectory_gt_${depth_source}_0.txt"
cp -f "$output_log" "$dst/run_log.txt"

echo ""
echo "Results saved to: $dst"

# Update latest symlink
latest="$results_directory/latest-tum-gt-${depth_source}"
rm -f "$latest"
ln -sf "$dst" "$latest"

exit $exit_code
