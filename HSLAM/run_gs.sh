# run using this script: 
# cd /home/ahmad/Desktop/GD/H-SLAM_GD/HSLAM
# ./run_gs.sh --files=/home/ahmad/datasets/replica/dso/room0/images \
#   --calib=/home/ahmad/Desktop/GD/MGSO_GD/configs/replica_calib/camera.txt \
#   --vocab=misc/orbvoc.dbow3 --colour --loopclosure \
#   --gauss=configs/gaussian_mapper/replica_mono.yaml --mode=1
# 
# 
# or simply run: 
# cd /home/ahmad/Desktop/GD/H-SLAM_GD/HSLAM
# LD_LIBRARY_PATH=/usr/local/cuda-11.6/lib64:/home/ahmad/Desktop/GD/MGSO_GD/thirdparty/opencv-4.9.0-cuda/lib:/home/ahmad/Desktop/GD/MGSO_GD/thirdparty/libtorch/lib:/home/ahmad/Desktop/GD/H-SLAM_GD/HSLAM/Thirdparty/onnxruntime/lib \
# LD_PRELOAD=/home/ahmad/anaconda3/lib/libstdc++.so.6 \
# ./build/bin/HSLAM \
#   --files=/home/ahmad/datasets/replica/dso/room0/images \
#   --calib=/home/ahmad/Desktop/GD/MGSO_GD/configs/replica_calib/camera.txt \
#   --vocab=misc/orbvoc.dbow3 --colour --loopclosure \
#   --gauss=configs/gaussian_mapper/replica_mono.yaml --mode=1

cd "$(dirname "$0")" || exit 1

LD_LIBRARY_PATH=/usr/local/cuda-11.6/lib64:/home/ahmad/Desktop/GD/MGSO_GD/thirdparty/opencv-4.9.0-cuda/lib:/home/ahmad/Desktop/GD/MGSO_GD/thirdparty/libtorch/lib:/home/ahmad/Desktop/GD/H-SLAM_GD/HSLAM/Thirdparty/onnxruntime/lib \
LD_PRELOAD=/home/ahmad/anaconda3/lib/libstdc++.so.6 \
exec ./build/bin/HSLAM "$@"

