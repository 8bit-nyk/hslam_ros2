#!/bin/bash

# Define dataset path
DATASET_PATH="/home/aub/datasets/boreas/boreas-2021-01-26-10-59"
HSLAM_PATH="/home/aub/Dev/hslam_ros2_ws/src/HSLAM"
echo "DATASET_PATH: $DATASET_PATH"
# echo "./bin/HSLAM -files={$DATASET_PATH}/images.zip -calib=$DATASET_PATH/camera.txt -vocabPath=$HSLAM_PATH/misc/orbvoc.dbow3 -mode=1"
# Run HSLAM
./build/bin/HSLAM \
    --files=/home/aub/datasets/boreas/boreas-2021-01-26-10-59/images.zip \
    --calib=/home/aub/datasets/boreas/boreas-2021-01-26-10-59/calib/camera_cropped.txt \
    --vocab=/home/aub/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3 \
    --mode=1
