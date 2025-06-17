# HSLAM: Hybrid Direct-Indirect Monocular Visual SLAM

## Project Description

The HSLAM project is a C++ implementation of a monocular visual simultaneous localization and mapping algorithm that combines the strengths of both direct and indirect methods for improved performance in visual SLAM tasks.

In this repository we introduce an update ROS 2 version for better reproducibility. This documentation is tested using ROS2 humble on Ubuntu 22.04.

This is the current up to date development branch for the HSLAM project, legacy imlplementations can be found in the [legacy branch](#related-legacy-projects).




### Related Publications:
### Related Legacy projects:

Please cite the paper if used in an academic context.





## Installation
### Prepare project and colcon workspace
To build the HSLAM project, follow these steps:

1. Create the ros2 workspace directory. For example"
```bash
mkdir hslam_ws
```
2. Clone this repository inside the ros2 workspace you just created.
```bash
git clone 
```
3. rename to *src* , required later for the colcon build .

### Building *Thridparty* libraries 

1. In a terminal. Navigate to Thirdparty directory, For example:
```bash
    cd ~/hslam_ws/src/HSLAM/Thirdparty

```
2. Build Thirparty libraries using the provided shell script.
```bash
    sudo chmod +x build.sh && ./build.sh
```
### Building the main HSLAM application:

1. Navigate to the HSLAM directory.
```bash
    cd /hslam_ws/src/FSLAM
```
2. Build FSLAM.
```bash
    mkdir -p build && cd build && cmake .. && make -j 10
```
5. Navigate to the workspace directory.
```bash
    cd /hslam_ws
```
6. Build the workspace.
```bash
    colcon build --packages-select hslam_ros2
```

## Usage

To run the HSLAM run:
1. In the first terminal run the following command to open a camera stream through ROS and publish the camera's images unto a ROS topic:
``` bash
    ros2 launch realsense2_camera rs_launch.py
```

2. In the second terminal, execute this command to run the FSLAM algorithm on the image stream.
``` bash
    ros2 launch hslam_ros2 hslam.launch.py
```
Start moving the camera/computer around and perform SLAM.

## Features

- Utilizes the FSLAM algorithm for simultaneous localization and mapping.
- Integrates with ROS Foxy and utilizes various ROS functionalities.
- Supports camera integration, including Realsense cameras.
- Provides a wrapper for ROS integration and additional functionality.

## Contributing

Contributions to the FSLAM project are welcome. If you would like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Make the necessary changes and commit them.
4. Push your changes to your forked repository.
5. Submit a pull request detailing the changes you have made.

## License
This project is licensed under the [MIT License](LICENSE).
