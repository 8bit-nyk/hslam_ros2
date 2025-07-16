# HSLAM Development Guidelines

## 1. Project Overview

### 🎯 **Project Context**
- **Core Application:** HSLAM C++ implementation.
- **Architecture:** Hybrid direct-indirect monocular visual SLAM algorithm.
- **Current Research Focus:** Improving depth estimation by integrating external data sources. The initial proof-of-concept will use ground truth depth from the TUM RGB-D dataset.
- **Performance Target:** Maintain real-time performance (20+ fps) for the core SLAM system.

### 🏗️ **Project Structure**
- **`HSLAM/`** - Core C++ SLAM implementation.
  - **`HSLAM/src/`** - Core SLAM algorithm implementation.
  - **`HSLAM/include/`** - Public headers for the algorithm.
  - **`HSLAM/docs/`** - Research papers, technical documentation, and experiment logs.
  - **`HSLAM/Thirdparty/`** - External dependencies (G2O, DBoW3, etc.).
  - **`HSLAM/build/`** - Build artifacts (not committed to version control).
  - **`HSLAM/run_scripts/`** - Scripts for running experiments and benchmarks.
  - **`HSLAM/models/`** - Location for ML models (for future use).

---

## 2. C++ Programming Guidelines

### 💻 **C++ Standards and Configuration**
- **C++ Standard:** C++14.
- **Build System:** CMake.
- **Compiler Flags:** `-Wall -Wextra -Wpedantic` must be used to ensure high code quality.
- **Dependencies:** OpenCV, Eigen3, Boost, Pangolin, G2O, DBoW3. ONNX Runtime is available for future deep learning integration.
- **Header Guards:** Use `#pragma once`.
- **Include Order:** Standard library → Third-party headers → Project headers.

### 📝 **Naming Conventions**
- **Classes:** `PascalCase` (e.g., `FullSystem`, `MapPoint`).
- **Functions:** `snake_case` (e.g., `track_new_frame()`).
- **Variables:** `snake_case` (e.g., `image_pyramid`, `world_pose`).
- **Constants:** `UPPER_SNAKE_CASE` (e.g., `MAX_FEATURES`, `PYRAMID_LEVELS`).
- **Namespaces:** `lowercase` (e.g., `hslam`, `hslam::util`).
- **Files:** `snake_case` (e.g., `coarse_tracker.cpp`, `map_point.hpp`).

### 🏗️ **Code Organization & Memory Management**
- **RAII Principles:** Resource acquisition is initialization. Use `std::unique_ptr` and `std::shared_ptr` for all resource management. Avoid raw pointers for owning resources.
- **Class Structure:** Public members, then protected, then private.
- **Header Files (.hpp):** Declarations only. Keep includes to a minimum.
- **Source Files (.cpp):** Definitions and implementations.

### 🧪 **Testing and Verification**
- **Unit Tests:** All new algorithmic components should be accompanied by unit tests.
- **Integration Tests:** Experiments and new features must be tested on full datasets (e.g., TUM RGB-D).
- **Verification Scripts:** Create scripts in `run_scripts/` to automate the process of running tests and evaluating results (e.g., calculating Absolute Trajectory Error).
- **Temporary Files:** All temporary files or scripts created for verification should be clearly named (e.g., `verify_depth_integration.sh`) and should not be committed unless they are part of the final, reusable verification toolkit.

### 📚 **Documentation Standards**
- **Code Comments:** Explain the "why," not the "what." Document complex algorithms, mathematical assumptions, and non-obvious logic.
- **API Documentation:** Use Doxygen-style comments for public-facing classes and functions.
- **Experiment Logs:** All research and experiments should be documented in markdown files within the `docs/` directory.

---

## 3. HSLAM-Specific Guidelines

### 🔬 **Current Research: Depth Estimation Experiments**
- **Objective:** Integrate external depth data to analyze its impact on system performance and to serve as a proof-of-concept for future DL-based depth estimators.
- **Dataset:** TUM RGB-D (`rgbd_dataset_freiburg1_xyz`). The dataset is located at `/home/aub/datasets/rgbd_dataset_freiburg1_xyz`.
- **Method (Revised Plan):**
    1.  **Milestone 2.0: Pre-computation and Library Integration**
        -   **Data Prep:** Use the `associate.py` script from TUM to generate `associations.txt` for the dataset.
        -   **Library Integration:** Download the header-only `tum_benchmark` library and integrate it into the project via `HSLAM/Thirdparty/` and the main `CMakeLists.txt`.
    2.  **Milestone 2.1: System Interface for External Depth**
        -   In `main.cpp`, use `tum_benchmark::FileReader` to parse `associations.txt`.
        -   Load RGB and Depth images (as `CV_32FC1`, scaled by `1/5000.0`) in the main loop.
        -   Create a new system entry point `FullSystem::TrackRGBD(...)`.
    3.  **Milestone 2.2: Depth Integration for New Points**
        -   When creating new feature points (`ImmaturePoint`), use the depth from the input depth map to calculate a precise initial `idepth`.
        -   This will bypass the standard monocular depth estimation for new points.
    4.  **Milestone 2.3: Final Evaluation**
        -   Run the full system on a TUM sequence.
        -   Save the estimated trajectory and evaluate the Absolute Trajectory Error (ATE) against the ground truth using a Python script.


#### **Technical Implementation**
- **Library**: `tum_benchmark` for RGB-D data handling

*For detailed implementation information, see [implementation_guide.md](./implementation_guide.md)*

### 🚀 **Performance Considerations**
- **Real-time Constraints:** All new developments should be mindful of the 20+ fps target.
- **Memory Efficiency:** Avoid unnecessary data copies. Use `const` references for passing large objects. Profile memory usage to stay within reasonable limits.
- **Profile First:** Do not optimize prematurely. Use profiling tools to identify bottlenecks before attempting to optimize.

