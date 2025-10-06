#!/bin/bash 
# nyk 22/05/2023
# Run this script once as it will clean up after itself. Everytime you run it will recompile packages (except opencv)
BuildType="RelWithDebInfo" 

SCRIPTPATH=$(dirname $0)
if [ $SCRIPTPATH = '.' ]
then
SCRIPTPATH=$(pwd)
fi
# echo -e "Script path: $SCRIPTPATH\n"

mkdir -p $SCRIPTPATH/CompiledLibs
InstallDir=$SCRIPTPATH/CompiledLibs

#install system wide dependencies
#================================
export DEBIAN_FRONTEND=noninteractive
sudo apt -y install libgl1-mesa-dev libglew-dev libsuitesparse-dev libeigen3-dev libboost-all-dev cmake build-essential git libzip-dev ccache freeglut3-dev libgoogle-glog-dev libatlas-base-dev ninja-build unzip wget

#install libceres for compatibility with ubuntu 22:
echo -e "Setting up Ceres Solver\n"
cd $SCRIPTPATH
if [ ! -d "ceres-solver-1.14.0" ]; then
    echo "Downloading Ceres Solver..."
    wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
    tar -zxf ceres-solver-1.14.0.tar.gz
    echo "Ceres Solver downloaded"
else
    echo "Ceres Solver source already exists"
fi

cd ceres-solver-1.14.0 && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$InstallDir -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCXX11=ON && make -j $(nproc) && make install
cd ../.. && rm -rf ceres-solver-1.14.0/build
echo "Ceres Solver build complete"

#optional libs to record pangolin gui
sudo apt -y install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev

# OpenCV build with contrib modules
echo -e "Setting up OpenCV\n"
cvVersion=4.9.0
cd $SCRIPTPATH

if [ ! -d "opencv-${cvVersion}" ]; then
    echo "Downloading OpenCV ${cvVersion}..."
    DL_opencv="https://github.com/opencv/opencv/archive/${cvVersion}.zip"
    DL_contrib="https://github.com/opencv/opencv_contrib/archive/${cvVersion}.zip"

    wget -O opencv.zip "${DL_opencv}" && unzip opencv.zip && rm opencv.zip
    wget -O opencv_contrib.zip "${DL_contrib}" && unzip opencv_contrib.zip && rm opencv_contrib.zip

    # Move contrib into opencv directory
    mv opencv_contrib-${cvVersion} opencv-${cvVersion}/
    echo "OpenCV ${cvVersion} downloaded"
else
    echo "OpenCV ${cvVersion} already exists"
fi

echo -e "Compiling OpenCV ${cvVersion}\n"
cd $SCRIPTPATH/opencv-${cvVersion} && mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=$BuildType \
    -DCMAKE_INSTALL_PREFIX=$InstallDir \
    -DWITH_V4L=ON \
    -DWITH_CUDA=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DWITH_QT=ON \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DWITH_OPENGL=ON \
    -DOPENCV_EXTRA_MODULES_PATH=$SCRIPTPATH/opencv-${cvVersion}/opencv_contrib-${cvVersion}/modules \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DCeres_DIR=$InstallDir/lib/cmake/Ceres
make -j $(nproc) && make install
cd .. && rm -rf build
echo "OpenCV build complete"

#Build Thirdparty libs	
#=====================
echo -e "Compiling Pangolin\n"
cd $SCRIPTPATH/Pangolin
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BuildType -DCMAKE_INSTALL_PREFIX=$InstallDir -DBUILD_PANGOLIN_PYTHON=OFF -DDISPLAY_WAYLAND=OFF -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF && make -j $(nproc) && make install && cd .. && rm -r build

#echo -e "Compiling G2O\n"
cd $SCRIPTPATH/g2o
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BuildType -DCMAKE_INSTALL_PREFIX=$InstallDir -DCMAKE_RELWITHDEBINFO_POSTFIX="" -DCMAKE_MINSIZEREL_POSTFIX="" -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_WITH_MARCH_NATIVE=ON -DG2O_USE_OPENMP=OFF
make -j $(nproc) && make install && cd .. && rm -r build && rm -r bin && rm -r lib

echo -e "Compiling DBoW3\n"
cd $SCRIPTPATH/DBow3
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BuildType -DCMAKE_INSTALL_PREFIX=$InstallDir -DBUILD_UTILS=OFF -DCMAKE_CXX_FLAGS=-std=c++11 -DUSE_CONTRIB=true -DOpenCV_DIR=$InstallDir/share/OpenCV && make -j $(nproc) && make install && cd .. && rm -r build

# Download and setup ONNX Runtime GPU for deep learning integration
echo -e "Setting up ONNX Runtime GPU\n"
cd $SCRIPTPATH
if [ ! -d "onnxruntime" ]; then
    echo "Downloading ONNX Runtime GPU..."
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-gpu-1.19.2.tgz
    tar -xzf onnxruntime-linux-x64-gpu-1.19.2.tgz
    mv onnxruntime-linux-x64-gpu-1.19.2 onnxruntime
    # rm onnxruntime-linux-x64-gpu-1.19.2.tgz
    echo "ONNX Runtime GPU setup complete"
else
    echo "ONNX Runtime already exists"
fi

#set environment settings
#==========================
if grep -Fxq 'PATH=${PATH}'":${InstallDir}/bin" ~/.bashrc 
then :
else
  echo 'PATH=${PATH}'":${InstallDir}/bin:$InstallDir/share/OpenCV" >> ~/.bashrc 
  echo 'LD_LIBRARY_PATH=${LD_LIBRARY_PATH}'":${InstallDir}/lib" >> ~/.bashrc
  source ~/.bashrc 
fi


#build SLAM
#==========
#cmake_prefix=$InstallDir/lib/cmake
#cd $SCRIPTPATH && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BuildType && make -j $(nproc)



