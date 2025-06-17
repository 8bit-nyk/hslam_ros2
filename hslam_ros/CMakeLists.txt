cmake_minimum_required(VERSION 3.5)
project(hslam_ros)
set(HARDCODED_PROJECT_DIR /home/aub/Dev) # Hardcoded path to the project directory (replace later: pavan)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/hslam_ros/cmake)
set(CMAKE_PREFIX_PATH ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/cmake)
message("The CMAKE prefix path is: ${CMAKE_PREFIX_PATH}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.9.0 REQUIRED PATHS ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/cmake/opencv4)
find_package(Pangolin 0.2 REQUIRED PATHS ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/cmake)
find_package(DBoW3 REQUIRED)
find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

set(G2O_DIR ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/cmake/g2o)
set(G2O_INCLUDE_DIR ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/include/g2o)
set(G2O_LIB_DIR ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib)
message("The G2O directory is : ${G2O_DIR}" )
find_package(G2O REQUIRED)
message("\n\n---- FOUND G2O Headers at\n\"${G2O_INCLUDE_DIR}\"")
message("---- FOUND G2O Libs at\n\"${G2O_LIBS}\"")

# Manually set G2O libraries if find_package fails to populate G2O_LIBS
if(NOT G2O_LIBS)
    set(G2O_LIBS
        ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/libg2o_core.so
        ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/libg2o_stuff.so
        ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/libg2o_solver_csparse.so
        ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/libg2o_types_slam3d.so
        ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/libg2o_types_sba.so
        ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/Thirdparty/CompiledLibs/lib/libg2o_csparse_extension.so
        # Add other G2O libraries here as needed
    )
    message(STATUS "Manually set G2O_LIBS to: ${G2O_LIBS}")
endif()

# Set paths
set(FSLAM_PATH ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM)
set(FSLAM_LIBRARY_PATH ${HARDCODED_PROJECT_DIR}/hslam_ros2_ws/src/HSLAM/build/lib)

# Set values from environment variables if not already set
if (NOT FSLAM_PATH)
    set(FSLAM_PATH $ENV{FSLAM_PATH})
endif()
if (NOT FSLAM_LIBRARY_PATH)
    set(FSLAM_LIBRARY_PATH $ENV{FSLAM_LIBRARY_PATH})
    if (NOT FSLAM_LIBRARY_PATH AND FSLAM_PATH)
        set(FSLAM_LIBRARY_PATH "${FSLAM_PATH}/build/lib")
    endif()
endif()

# Find HSLAM library and headers
if (FSLAM_PATH)
    # Manually set path to HSLAM source folder
    message("FSLAM_PATH set, trying to find library.")
    message("---- EXPECTING HSLAM sources at\n\"${FSLAM_PATH}\"")
    set(FSLAM_INCLUDE_DIRS "${FSLAM_PATH}/src" "${FSLAM_PATH}/Thirdparty/Sophus" "${FSLAM_PATH}/Thirdparty/g2o" "${FSLAM_PATH}/Thirdparty/DBow3" "${FSLAM_PATH}/Thirdparty/opencv-4.9.0")
    message("---- LOOKING FOR HSLAM library at\n\"${FSLAM_LIBRARY_PATH}\"")
    find_library(FSLAM_LIBRARY HSLAM_lib ${FSLAM_LIBRARY_PATH})
else()
    message("FSLAM_PATH not set yet, trying to find installed hslam headers and library.")
    find_path(FSLAM_INCLUDE_DIRS HSLAM)
    if (FSLAM_INCLUDE_DIRS)
        set(FSLAM_INCLUDE_DIRS "${FSLAM_INCLUDE_DIRS}/HSLAM")
        message("---- FOUND HSLAM headers at \"${FSLAM_INCLUDE_DIRS}\"")
    endif()
    find_library(FSLAM_LIBRARY hslam)
endif()

if (NOT FSLAM_INCLUDE_DIRS)
    message(FATAL_ERROR "HSLAM headers not found and/or FSLAM_PATH not set.")
endif()
if (NOT FSLAM_LIBRARY)
    message(FATAL_ERROR "HSLAM library not found and/or FSLAM_LIBRARY_PATH not set.")
endif()

message("---- Found HSLAM library at \"${FSLAM_LIBRARY}\"")

set(LIBS
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
  ${Pangolin_LIBRARIES}
  ${DBoW3_LIB_DIR}/libDBoW3.so
  ${BOOST_THREAD_LIBRARY}
  boost_system
  cxsparse
  cholmod
  fmt
)
# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${FSLAM_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
)

link_directories(
    ${G2O_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
)
# Declare a C++ executable
add_executable(fslam_live src/main.cpp)
set_target_properties(fslam_live PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${FSLAM_LIBRARY_PATH}:${CMAKE_INSTALL_RPATH}"
)
# Link libraries
ament_target_dependencies(fslam_live
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
)

target_link_libraries(fslam_live
    ${FSLAM_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${G2O_LIBS}
    ${DBoW3_LIB_DIR}/libDBoW3.so
    ${CSPARSE_LIBRARY}
    ${CHOLMOD_LIBRARIES}
)

# Install
install(TARGETS fslam_live
  DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
)

if(EXISTS "${CMAKE_SOURCE_DIR}/launch")
    install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/launch
    )
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/config")
    install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/config
    )
endif()

ament_package()