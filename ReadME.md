# HSLAM2: Photogrammetric and Deep Learned Hybrid Monocular Visual SLAM

## Project Description

HSLAM is a C++ implementation of a monocular visual simultaneous localization and mapping (SLAM) algorithm that combines:

- **Direct Methods**: Photometric tracking from DSO (Direct Sparse Odometry)
- **Indirect Methods**: Feature-based loop closure using ORB features
- **ML Depth Integration**: Deep learning-based metric depth estimation for improved scale and accuracy

This hybrid approach leverages the strengths of both direct and indirect SLAM methods, enhanced with modern deep learning techniques for robust performance across diverse datasets.

### Key Features

- **Production-Ready Performance**: 12-20 FPS on TUM RGBD, EuRoC MAV, and KITTI datasets
- **GPU-Accelerated ML Depth**: 44ms inference time with CUDA support on NVIDIA RTX A4500 Laptop GPU (vs 650ms on Intel i9-12950HX CPU)
- **Metric Scale Estimation**: ML depth provides metric scale initialization
- **Loop Closure Detection**: DBoW3-based place recognition with 1000 ORB features
- **Real-time Visualization**: Pangolin-based 3D display with trajectory and point cloud

---

## Related Publications

Please cite these papers if used in academic context:

**[HSLAM2: Photogrammetric and Deep Learned Hybrid Monocular Visual SLAM]**

```bibtex

```
**[H-SLAM: Hybrid direct--indirect visual SLAM](https://doi.org/10.1016/j.robot.2024.104729)**

```bibtex
@article{younes2024h,
  title={H-SLAM: Hybrid direct--indirect visual SLAM},
  author={Younes, Georges and Khalil, Douaa and Zelek, John and Asmar, Daniel},
  journal={Robotics and Autonomous Systems},
  volume={179},
  pages={104729},
  year={2024},
  publisher={Elsevier}
}
```

**[Inline Photometrically Calibrated Hybrid Visual SLAM](https://doi.org/10.1109/IROS58592.2024.10802153)**

```bibtex
@inproceedings{abboud2024inline,
  title={Inline Photometrically Calibrated Hybrid Visual SLAM},
  author={Abboud, Nicolas and Sayour, Malak and Elhajj, Imad H and Zelek, John and Asmar, Daniel},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={10089--10096},
  year={2024},
  organization={IEEE}
}
```

---

## System Requirements

### Tested Environment
- **OS**: Ubuntu 22.04 LTS
- **Compiler**: GCC 11+ with C++17 support
- **CMake**: Version 3.0 or higher
- **GPU** (Optional): NVIDIA GPU with CUDA support for ML depth acceleration

### 3D Gaussian Splatting (3DGS) Requirements
The 3DGS integration adds these requirements on top of the base SLAM:
- **NVIDIA GPU + CUDA Toolkit ≥ 11** (validated on 11.6) — **required, not optional**. OpenCV is built with CUDA, and the Gaussian rasterizer runs on the GPU.
- **libtorch 2.0.0** (LibTorch C++), CUDA build matching your toolkit — downloaded in the build steps below.
- **Conda environment** (`environment.yml`: Python 3.8 + PyTorch 2.0.0) — provides the toolchain and the `libstdc++` the binary links against at runtime. Created in the install steps.

### Required System Dependencies
```bash
sudo apt update
sudo apt install -y \
    libgl1-mesa-dev libglew-dev libsuitesparse-dev libeigen3-dev \
    libboost-all-dev cmake build-essential git libzip-dev ccache \
    freeglut3-dev libgoogle-glog-dev libatlas-base-dev ninja-build \
    unzip wget
```

### Optional Dependencies (for Pangolin GUI recording)
```bash
sudo apt install -y ffmpeg libavcodec-dev libavutil-dev \
    libavformat-dev libswscale-dev libavdevice-dev
```

---

## Installation

### Step 1: Clone the Repository

```bash
# Create workspace directory
mkdir -p ~/hslam_ws
cd ~/hslam_ws

# Clone repository
git clone <repository-url> src
cd src/HSLAM
```

### Step 2: Create the Conda Environment (required for 3DGS)

The 3DGS build and run use a conda env (Python 3.8 + PyTorch 2.0.0). It also supplies the
`libstdc++` the HSLAM binary needs at runtime. **Create it once and keep it activated for every
step below and whenever you run the demo:**

```bash
conda env create -f environment.yml
conda activate surfel_splatting
```

### Step 3: Build Third-Party Dependencies

HSLAM requires several third-party libraries. The automated build script handles everything:

```bash
cd Thirdparty
chmod +x build.sh
./build.sh
```

**This script will:**
1. Install system dependencies (if not already installed)
2. Download and build Ceres Solver 1.14.0
3. Download and build OpenCV 4.9.0 with contrib modules
4. Build Pangolin (visualization library)
5. Build g2o (graph optimization)
6. Build DBoW3 (loop closure vocabulary)
7. Download ONNX Runtime 1.19.2 with GPU support
8. Configure environment variables in `~/.bashrc`

**Build time**: 30-60 minutes depending on your system (OpenCV is the longest step)

**Note**: After the script completes, reload your shell environment:
```bash
source ~/.bashrc
```

<details>
<summary><b>Manual Build Instructions (Alternative)</b></summary>

If you prefer manual control or encounter issues with the automated script:

#### 1. Install System Dependencies
```bash
sudo apt install -y libgl1-mesa-dev libglew-dev libsuitesparse-dev \
    libeigen3-dev libboost-all-dev cmake build-essential git libzip-dev \
    ccache freeglut3-dev libgoogle-glog-dev libatlas-base-dev ninja-build \
    unzip wget ffmpeg libavcodec-dev libavutil-dev libavformat-dev \
    libswscale-dev libavdevice-dev
```

#### 2. Build Ceres Solver
```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty
wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
tar -zxf ceres-solver-1.14.0.tar.gz
cd ceres-solver-1.14.0
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../CompiledLibs -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF -DCXX11=ON
make -j$(nproc) && make install
```

#### 3. Build OpenCV 4.9.0
```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.9.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.9.0.zip
unzip opencv.zip && unzip opencv_contrib.zip
mv opencv_contrib-4.9.0 opencv-4.9.0/

cd opencv-4.9.0
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX=../../CompiledLibs \
    -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_V4L=ON \
    -DWITH_CUDA=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.9.0/modules \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DCeres_DIR=../../CompiledLibs/lib/cmake/Ceres
make -j$(nproc) && make install
```

#### 4. Build Pangolin
```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty/Pangolin
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../CompiledLibs \
    -DBUILD_PANGOLIN_PYTHON=OFF -DDISPLAY_WAYLAND=OFF \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc) && make install
```

#### 5. Build g2o
```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty/g2o
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../CompiledLibs \
    -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
    -DBUILD_WITH_MARCH_NATIVE=ON -DG2O_USE_OPENMP=OFF
make -j$(nproc) && make install
```

#### 6. Build DBoW3
```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty/DBow3
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../CompiledLibs \
    -DBUILD_UTILS=OFF -DUSE_CONTRIB=true \
    -DOpenCV_DIR=../../CompiledLibs/share/OpenCV
make -j$(nproc) && make install
```

#### 7. Setup ONNX Runtime
```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-gpu-1.19.2.tgz
tar -xzf onnxruntime-linux-x64-gpu-1.19.2.tgz
mv onnxruntime-linux-x64-gpu-1.19.2 onnxruntime
```

#### 8. Set Environment Variables
Add to `~/.bashrc`:
```bash
export PATH=$PATH:~/hslam_ws/src/HSLAM/Thirdparty/CompiledLibs/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/hslam_ws/src/HSLAM/Thirdparty/CompiledLibs/lib
```
Then reload: `source ~/.bashrc`

</details>

### Step 4: Download ML Depth Model

HSLAM uses the Metric3D ViT-Small model for depth estimation:

```bash
cd ~/hslam_ws/src/HSLAM/models

# Clone the ONNX model from Hugging Face
git clone https://huggingface.co/onnx-community/metric3d-vit-small
```

**Model Details:**
- Size: ~144 MB (model.onnx)
- Inference time: 44ms (GPU) / 650ms (CPU)
- Input resolution: 518×518
- Output: Metric depth estimation with confidence values

**Verify installation:**
```bash
ls -lh metric3d-vit-small/onnx/model.onnx
# Should show: -rw-rw-r-- ... 144M ... model.onnx
```

### Step 5: Get libtorch (required for 3DGS)

Download **libtorch 2.0.0** with the CUDA tag matching your toolkit and unzip into `Thirdparty/libtorch`:

```bash
cd ~/hslam_ws/src/HSLAM/Thirdparty
# CUDA 11.x example (cu118):
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcu118.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.0+cu118.zip     # -> Thirdparty/libtorch/
```

### Step 6: Build HSLAM Application

With the conda env active, activate your CUDA toolkit in the terminal, then configure and build:

```bash
export CUDA_HOME=/usr/local/cuda-11.6        # your CUDA >= 11
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

cd ~/hslam_ws/src/HSLAM
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CUDA_COMPILER=$CUDA_HOME/bin/nvcc \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(nproc)
```
 

## Running the 3D Gaussian Splatting Demo

Use `run_gs.sh` — it sets the runtime library paths (CUDA, OpenCV, libtorch, ONNX Runtime) for you:

```bash
cd ~/hslam_ws/src/HSLAM
./run_gs.sh \
  --files=/path/to/replica/dso/room0/images \
  --calib=configs/replica_calib/camera.txt \
  --vocab=misc/orbvoc.dbow3 \
  --gauss=configs/gaussian_mapper/replica_mono.yaml \
  --colour --loopclosure --mode=1 --gsgui
```

**Arguments:**

| Flag | Meaning |
|---|---|
| `--files` | Image sequence folder (DSO format) |
| `--calib` | Camera calibration file |
| `--vocab` | ORB vocabulary for loop closure (`misc/orbvoc.dbow3`) |
| `--gauss` | Gaussian mapper config (see `configs/gaussian_mapper/`) |
| `--colour` | Enable colour rendering |
| `--loopclosure` | Enable loop closure |
| `--mode` | DSO processing mode (`1` = no photometric calibration) |

> Built inside a conda env? `run_gs.sh` auto-preloads that env's `libstdc++`. If you get a
> `GLIBCXX_... not found` error, activate your conda env or set `STDCXX_LIB=/path/to/libstdc++.so.6`.

### Dataset (Replica)

The example uses the **Replica** dataset (NICE-SLAM version — the same source used by Photo-SLAM):

```bash
wget https://cvg-data.inf.ethz.ch/nice-slam/data/Replica.zip
unzip Replica.zip
```

HSLAM reads each sequence in **DSO layout** — a flat folder of frames:

```
replica/dso/room0/
└── images/          # sequential RGB frames
```

> The NICE-SLAM download ships frames under `room0/results/`. If your copy isn't already in
> the `dso/room0/images/` layout above, convert it to a flat `images/` folder first.

Included Gaussian-mapper configs: `configs/gaussian_mapper/replica_mono.yaml`, `configs/gaussian_mapper/tumvi_mono.yaml`.

---

## Dataset Setup

HSLAM supports three benchmark datasets. Download the ones you need:

### TUM RGBD Dataset

**Download:**
```bash
mkdir -p ~/datasets/TUM_RGBD
cd ~/datasets/TUM_RGBD

# Example: Download freiburg1_room sequence
wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_room.tgz
tar -xzf rgbd_dataset_freiburg1_room.tgz
```

**Available sequences:**
- `freiburg1_desk`, `freiburg1_room`, `freiburg1_xyz` (indoor office)
- `freiburg2_desk`, `freiburg2_xyz` (office with fast motion)
- `freiburg3_long_office_household` (long sequence)

**More datasets**: https://vision.in.tum.de/data/datasets/rgbd-dataset/download

### EuRoC MAV Dataset

**Download:**
```bash
mkdir -p ~/datasets/EuRoC
cd ~/datasets/EuRoC

# Example: Download MH_01_easy sequence
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip MH_01_easy.zip
```

**Available sequences:**
- Machine Hall: `MH_01_easy` through `MH_05_difficult`
- Vicon Room: `V1_01_easy` through `V2_03_difficult`

**More datasets**: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### KITTI Dataset

**Download:**
```bash
mkdir -p ~/datasets/KITTI
cd ~/datasets/KITTI

# Example: Download sequence 00
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
unzip 2011_10_03_drive_0027_sync.zip
```

**Note**: KITTI requires timestamp conversion, which is handled automatically by the run scripts.

**More datasets**: http://www.cvlibs.net/datasets/kitti/eval_odometry.php

---

## Running HSLAM

HSLAM includes dataset-specific run scripts that handle all configuration automatically.

### Quick Start: TUM RGBD with ML Depth

```bash
cd ~/hslam_ws/src/HSLAM
./run_scripts/hslam_run_tumrgbd_ml_depth.sh freiburg1_room
```

This will:
- Run HSLAM with ML depth integration
- Use GPU acceleration (if available)
- Display real-time 3D visualization
- Save trajectory to `results/` directory

### Run Scripts Overview

#### TUM RGBD Dataset
```bash
# ML Depth mode (recommended)
./run_scripts/hslam_run_tumrgbd_ml_depth.sh <sequence_name>

# Monocular mode (without ML depth)
./run_scripts/hslam_run_tumrgbd_monocular.sh <sequence_name>

# Example:
./run_scripts/hslam_run_tumrgbd_ml_depth.sh freiburg1_desk
```

#### EuRoC MAV Dataset
```bash
# ML Depth mode
./run_scripts/hslam_run_euroc_ml_depth.sh <sequence_name>

# Monocular mode
./run_scripts/hslam_run_euroc_monocular.sh <sequence_name>

# Example:
./run_scripts/hslam_run_euroc_ml_depth.sh MH_02_easy
```

#### KITTI Dataset
```bash
# ML Depth mode
./run_scripts/hslam_run_kitti_ml_depth.sh <sequence_number>

# Monocular mode
./run_scripts/hslam_run_kitti_monocular.sh <sequence_number>

# Example:
./run_scripts/hslam_run_kitti_ml_depth.sh 07
```

### Run Script Configuration

All run scripts support the following configurations (edit the script to modify):

- **GPU Acceleration**: `ml_gpu_enabled="true"` (set to "" for CPU-only)
- **ML Strategy**: `ml_strategy="keyframe_only"` (integrates ML depth on keyframes)
- **Frame Limit**: `end_index=""` (set to number for testing, leave empty for full sequence)
- **Quiet Mode**: `quiet_mode=""` (set to "--quiet" to reduce console output)

### Output Files

Results are saved to `~/hslam_ws/src/HSLAM/results/` in timestamped directories.

**Example output directory**: `hslam-ml-depth-freiburg1_desk-20250915_152947/`

**Directory structure**:
```
hslam-<mode>-<sequence>-<timestamp>/
├── trajectory_ml_depth_0.txt    # Camera trajectory (TUM format: timestamp tx ty tz qx qy qz qw)
├── run_summary.txt               # Run configuration and dataset statistics
├── ml_stats_0.txt                # ML depth integration statistics and performance
├── run_log_ml_depth_0.txt        # Complete HSLAM console output
├── debug_images/                 # ML depth and confidence visualizations
│   ├── ml_confidence_*.png       # Confidence maps for each keyframe
│   └── ml_depth_*.png            # Depth maps for each keyframe (if enabled)
├── logs_ml_depth_0/              # Detailed system logs
│   ├── fpsLog.txt                # Frame processing times
│   ├── coarseTrackingLog.txt     # Tracking residuals and statistics
│   ├── calibLog.txt              # Photometric calibration logs
│   └── [other diagnostic logs]
└── mats_ml_depth_0/              # (Empty - reserved for future use)
```

**Key output files**:

- **`trajectory_ml_depth_0.txt`**: Camera poses in TUM format (used for evaluation)
  - Format: `timestamp tx ty tz qx qy qz qw` (7 values per pose)
  - Can be evaluated using [TUM evaluation tools](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)

- **`run_summary.txt`**: Human-readable summary with:
  - Dataset information (path, image count, format)
  - ML configuration (model, strategy, resolution)
  - Run metadata (mode, sequence, timestamp)

- **`ml_stats_0.txt`**: ML depth performance metrics
  - Keyframe ML depth integration success rate
  - Average inference time per frame
  - ML depth utilization percentage
  - Sample depth ranges for validation

- **`debug_images/`**: Visual debugging outputs (10 samples by default)
  - Confidence maps showing ML depth uncertainty
  - Optional depth visualizations (colorized depth maps)

### Performance Benchmarks

| Dataset | Mode | FPS | Status |
|---------|------|-----|--------|
| TUM RGBD | ML Depth | 12-15 | ✅ Production Ready |
| EuRoC MAV | ML Depth | 12-15 | ✅ Production Ready |
| KITTI (640×368) | ML Depth | 18.3 | ✅ Production Ready |
| KITTI (1216×368) | ML Depth | 3.9 | ✅ Functional |

---

## ROS2 Integration (Optional)

HSLAM can be integrated with ROS2 for real-time camera streams and robotic applications.

### Prerequisites

```bash
# Install ROS2 Humble (if not already installed)
sudo apt install ros-humble-desktop

# Source ROS2
source /opt/ros/humble/setup.bash
```

### Build ROS2 Wrapper

```bash
cd ~/hslam_ws
colcon build --packages-select hslam_ros2
source install/setup.bash
```

### Running with ROS2

**Terminal 1** - Start camera stream:
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

**Terminal 2** - Run HSLAM:
```bash
source ~/hslam_ws/install/setup.bash
ros2 launch hslam_ros2 hslam.launch.py
```

**Supported cameras:**
- Intel RealSense D435/D455
- Any camera publishing to ROS2 image topics

---

## Troubleshooting

### Build Issues

**Issue**: `Could not find OpenCV`
**Solution**: Ensure environment variables are set:
```bash
source ~/.bashrc
echo $LD_LIBRARY_PATH  # Should include CompiledLibs/lib
```

**Issue**: `ONNX Runtime not found`
**Solution**: Check that `Thirdparty/onnxruntime/` exists and contains `include/` and `lib/` directories.

### Runtime Issues

**Issue**: `ML model file not found`
**Solution**: Verify model exists:
```bash
ls ~/hslam_ws/src/HSLAM/models/metric3d-vit-small/onnx/model.onnx
```

**Issue**: `GPU not found, using CPU`
**Solution**: Install CUDA toolkit for GPU acceleration:
```bash
sudo apt install nvidia-cuda-toolkit
```

**Issue**: `GLX BadAccess error`
**Solution**: This is resolved in the current version (Pangolin runs in main thread for Intel Mesa compatibility).

### Performance Issues

**Slow ML inference (>500ms)**:
- Verify GPU is being used: Check console output for "GPU device 0"
- Install NVIDIA drivers: `nvidia-smi` should show GPU info
- Use smaller resolution for KITTI (640×368 vs 1216×368)

**Low FPS (<5)**:
- Check dataset path is correct and accessible
- Reduce point density: Edit `setting_desiredPointDensity` in source (not recommended)
- Use `end_index` in run script to test shorter sequences

---

## Advanced Usage

### Custom Camera Calibration

**TUM/KITTI Format** (simple pinhole model):
```
fx fy cx cy k
width height
distortion_model
output_width output_height
```

Example (`fx fy cx cy k`):
```
517.3 516.5 318.6 255.3 0.0
640 480
none
640 480
```

**EuRoC Format** (RadTan distortion model):
```
RadTan fx fy cx cy k1 k2 p1 p2
width height
crop_params
output_width output_height
```

Run with custom calibration:
```bash
./build/bin/HSLAM \
    --calib path/to/camera.txt \
    --files path/to/images \
    --vocab misc/orbvoc.dbow3 \
    --ml-model models/metric3d-vit-small/onnx/model.onnx \
    --ml-strategy keyframe_only
```

### ML Depth Configuration

Core ML parameters:
- `--ml-depth`: Enable ML depth estimation (boolean)
- `--ml-model`: Path to ONNX ML depth model (default: `models/metric3d-vit-small/onnx/model.onnx`)
- `--ml-init`: Enable ML depth for metric scale initialization (default: true)

Available ML strategies (set via `--ml-strategy` parameter):
- `keyframe_only`: Integrate ML depth on keyframes only (recommended, default)
- `snapshot_mode`: Integrate ML depth every N keyframes (configurable)
- `every_frame`: Integrate ML depth on all frames (not validated, experimental)

Snapshot mode configuration:
- `--ml-snapshot-interval`: Frames between ML inference in snapshot mode (default: 5)

GPU acceleration settings:
- `--ml-gpu`: Enable GPU acceleration (boolean)
- `--ml-gpu-device`: Select GPU device ID (default: 0)
- `--ml-gpu-memory`: GPU memory limit in MB (default: 2048)
- `--ml-fp16`: Enable FP16 optimization for GPU inference (boolean)

Performance monitoring:
- `--ml-benchmark`: Enable ML performance benchmarking (boolean)

---

## Development

### Project Structure

```
HSLAM/
├── src/                    # Source code
│   ├── FullSystem/        # Core SLAM system
│   ├── OptimizationBackend/ # Bundle adjustment
│   ├── IOWrapper/         # Visualization (Pangolin)
│   ├── ML/                # ML depth integration
│   ├── Indirect/          # Loop closure (ORB, DBoW3)
│   └── util/              # Utilities and settings
├── Thirdparty/            # Third-party dependencies
├── models/                # ML models
├── run_scripts/           # Dataset-specific run scripts
├── misc/                  # Vocabulary and calibration files
└── results/               # Output trajectories and logs
```

### Key Configuration Files

- `src/util/settings.h`: Global SLAM settings
- `src/ML/MLDepthProcessor.h`: ML depth integration parameters
- `CLAUDE.md`: Development guidelines and system status

### Building with Debug Symbols

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

### Enable ML Depth Debugging

```bash
cd build
cmake .. -DENABLE_DEPTH_DEBUG=ON
make -j$(nproc)
```

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make your changes and commit: `git commit -am 'Add new feature'`
4. Push to the branch: `git push origin feature/your-feature`
5. Submit a pull request

### Coding Guidelines

- Follow existing code style (C++17, 4-space indentation)
- Add comments for non-obvious functionality
- Test with TUM RGBD dataset before submitting
- Update documentation for new features

---

## License

This repository is licensed under the **GNU General Public License version 3 (GPLv3)**.

### Collaborative Effort

This work is a joint collaborative effort between:

- **Vision and Robotics Lab** at the American University of Beirut (AUB)
- **Vision and Image Processing Group** at the University of Waterloo (UW)

---

## Related Legacy Projects

- [HSLAM C++ original implementation](https://github.com/8bit-nyk/HSLAM)
- [ROS1 Wrapper for HSLAM](https://github.com/8bit-nyk/hslam_ros)
- [Dockerized version of HSLAM](https://github.com/8bit-nyk/hslam_ros_docker)

---

## Acknowledgments

HSLAM builds upon several excellent open-source projects:

- [DSO (Direct Sparse Odometry)](https://github.com/JakobEngel/dso) - Direct visual odometry
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) - Indirect feature-based SLAM
- [Metric3D](https://jugghm.github.io/Metric3Dv2/) - Metric depth estimation model
- [ONNX Runtime](https://onnxruntime.ai/) - ML inference framework

---

## Contact & Support

For questions, issues, or collaboration:

- **Issues**: Please use the [GitHub issue tracker](https://github.com/your-repo/issues)
- **Documentation**: See `docs/` directory for detailed technical documentation
- **Status**: See `CLAUDE.md` for current development status and known issues

**Current System Status** (August 2025):
- ✅ All datasets production-ready
- ✅ ML depth integration fully functional
- ✅ Hybrid SLAM architecture complete
- ✅ GPU acceleration working
