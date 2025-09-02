########################
# To run HSLAM multiple instances you can use [&\] operator between two of the below commands
#Example:
#  ./HSLAM
#  &\
#  ./HSLAM
######################

#Set the desired directory path
build_directory_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/build/bin/"
cd "$build_directory_path"

echo "Running HSLAM RGB-D Pipeline Test"
echo "Working directory: $(pwd)"
echo "Home directory: $HOME"

results_directory="$HOME/Dev/hslam_ros2_ws/src/HSLAM/results"
mkdir -p $results_directory
repetitions=1

# Check if required files exist
dataset_path="$HOME/datasets/TUM_RGBD/rgbd_dataset_freiburg1_desk"
calib_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/run_scripts/camera_freiburg1.txt"
associations_path="$dataset_path/associations.txt"
vocab_path="$HOME/Dev/hslam_ros2_ws/src/HSLAM/misc/orbvoc.dbow3"

echo "Checking required files..."
echo "Dataset path: $dataset_path"
echo "Calibration file: $calib_path"
echo "Associations file: $associations_path"
echo "Vocabulary file: $vocab_path"

if [ ! -d "$dataset_path" ]; then
    echo "ERROR: Dataset directory not found: $dataset_path"
    exit 1
fi

if [ ! -f "$calib_path" ]; then
    echo "ERROR: Calibration file not found: $calib_path"
    exit 1
fi

# if [ ! -f "$associations_path" ]; then
#     echo "ERROR: Associations file not found: $associations_path"
#     exit 1
# fi

# Check if vocabulary file exists, disable loop closure if not
enable_loop_closure=false
if [ -f "$vocab_path" ]; then
    echo "Vocabulary file found, enabling loop closure"
    enable_loop_closure=true
else
    echo "WARNING: Vocabulary file not found, disabling loop closure"
fi

#hslam RGB-D pipeline test
for ((i = 0; i < repetitions; i++)); do
    echo "Starting HSLAM run $((i+1))/$repetitions"
    
    if [ "$enable_loop_closure" = true ]; then
        ./HSLAM \
            -f "$dataset_path" \
            -c "$calib_path" \
            -v "$vocab_path" \
            -a "$associations_path" \
            --loopclosure=true \
            --preset=0 \
            --mode=1 \
            --quiet \
            --outPC=true
    else
        ./HSLAM \
            -f "$dataset_path" \
            -c "$calib_path" \
            -a "$associations_path" \
            --preset=0 \
            --mode=1 \
            --nogui \
            --quiet \
            --outPC=true
    fi
    
    exit_code=$?
    echo "HSLAM completed with exit code: $exit_code"

    timestamp=$(date +"%Y%m%d_%H%M%S")
    destination_directory="$results_directory/hslam-tumRGBD-$timestamp"
    mkdir -p "$destination_directory"

    if [ -f "result.txt" ]; then
        mv "result.txt" "$destination_directory/trajectory_$i.txt"
        echo "Moved result.txt to: $destination_directory/trajectory_$i.txt"
    else
        echo "WARNING: result.txt not found"
    fi

    if [ -f "PC.PCD" ]; then
        mv "PC.PCD" "$destination_directory/pointcloud_$i.pcd"
        echo "Moved PC.PCD to: $destination_directory/pointcloud_$i.pcd"
    fi
    
    if [ -f "map.pcd" ]; then
        mv "map.pcd" "$destination_directory/map_$i.pcd"
        echo "Moved map.pcd to: $destination_directory/map_$i.pcd"
    fi
    
    if [ -d "logs" ]; then
        mv "logs" "$destination_directory/logs_$i"
        echo "Moved logs directory to: $destination_directory/logs_$i"
    fi

    if [ -d "mats" ]; then
        mv "mats" "$destination_directory/mats_$i"
        echo "Moved mats directory to: $destination_directory/mats_$i"
    fi
    
    echo "Results saved to: $destination_directory"
done

echo "All runs completed!"