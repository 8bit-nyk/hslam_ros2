#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/.."

# Set paths
CALIB_PATH="run_scripts/camera_logi.txt"
VOCAB_PATH="misc/orbvoc.dbow3"

echo "Running HSLAM with Logitech C270 camera"
echo "Calibration file: $CALIB_PATH"
echo "Vocabulary file: $VOCAB_PATH"

# Run HSLAM with camera input
./build/bin/HSLAM \
    --camera \
    --camera-id 5 \
    --calib=$CALIB_PATH \
    --vocab=$VOCAB_PATH \
    --mode=1 \
    --preset=1 \
    --nogui=false \
    --loopclosure 