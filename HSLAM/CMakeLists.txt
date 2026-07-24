SET(PROJECT_NAME HSLAM)
CMAKE_MINIMUM_REQUIRED(VERSION 3.18) # gd -dev -16june2026; was set to 2.9

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/CompiledLibs/lib/cmake/opencv4)
# set(OpenCV_DIR /home/ahmad/Desktop/GD/MGSO_GD/thirdparty/opencv-4.9.0-cuda/lib/cmake/opencv4)
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/CompiledLibs/lib/cmake)
#set(G2O_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/CompiledLibs/lib/cmake/g2o)
Set(G2O_INCLUDE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/CompiledLibs/include/g2o)
# Ensure these paths are set correctly before calling project()
message ("OPENCV Directory: ${OpenCV_DIR}")
message("The CMAKE prefix path is: ${CMAKE_PREFIX_PATH}")

PROJECT(${PROJECT_NAME} LANGUAGES CXX CUDA) # gd -dev -16june2026
#set(BUILD_TYPE Release)
set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# required libraries
find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
find_package(OpenCV 4.9.0 REQUIRED)
# find_package(OpenCV 4.9.0 REQUIRED PATHS ${HARDCODED_PROJECT_DIR}/colcon_ws/src/HSLAM/Thirdparty/CompiledLibs NO_DEFAULT_PATH)
find_package(G2O REQUIRED)
find_package(DBoW3 REQUIRED) #PATHS ~/colcon_ws/src/HSLAM/Thirdparty/CompiledLibs)
# gd -dev -16june2026
find_package(glm REQUIRED)
find_package(CUDAToolkit REQUIRED) 
set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/libtorch/share/cmake/Torch)
# set(Torch_DIR /home/ahmad/Desktop/GD/MGSO_GD/thirdparty/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -pthread")
# gd -dev -16june2016

# optional libraries
find_package(LibZip QUIET)
find_package(Pangolin 0.2 QUIET)# PATHS ~/colcon_ws/src/HSLAM/Thirdparty/CompiledLibs)

# ONNX Runtime configuration
set(ONNXRUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/onnxruntime)
if(EXISTS ${ONNXRUNTIME_DIR})
    message("--- found ONNX Runtime, compiling with ONNX Runtime library.")
    set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_DIR}/include)
    set(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_DIR}/lib)
    set(HAS_ONNXRUNTIME 1)
    add_definitions(-DHAS_ONNXRUNTIME=1)
else()
    message("--- could not find ONNX Runtime, not compiling with ONNX Runtime library.")
    message("    this means there will be no deep learning model inference capabilities.")
    set(HAS_ONNXRUNTIME 0)
    add_definitions(-DHAS_ONNXRUNTIME=0)
endif()

# CUDA configuration for GPU acceleration
find_package(CUDA QUIET)
if(CUDA_FOUND AND EXISTS ${ONNXRUNTIME_LIB_DIR}/libonnxruntime_providers_cuda.so)
    message("--- found CUDA and ONNX Runtime GPU provider, enabling GPU acceleration.")
    set(HAS_CUDA 1)
    add_definitions(-DHAS_CUDA=1)
    message("    CUDA Version: ${CUDA_VERSION}")
    message("    CUDA Libraries: ${CUDA_LIBRARIES}")
else()
    message("--- CUDA not found or ONNX Runtime GPU provider missing, GPU acceleration disabled.")
    set(HAS_CUDA 0)
    add_definitions(-DHAS_CUDA=0)
endif()

# flags
add_definitions("-DENABLE_SSE")
#detect and number of processors
include(ProcessorCount)
ProcessorCount(NUMTHREADS)
add_definitions("-DThreadCount=${NUMTHREADS}")

# DEPTH_DEBUG: Optional debugging flag for ML depth integration (TEMPORARY)
option(ENABLE_DEPTH_DEBUG "Enable comprehensive ML depth debugging (temporary)" OFF)
if(ENABLE_DEPTH_DEBUG)
    add_definitions(-DENABLE_DEPTH_DEBUG=1)
    message(STATUS "DEPTH_DEBUG: ML depth debugging enabled (temporary)")
endif()

# set(CMAKE_GENERATOR "Unix Makefiles")
# set(CMAKE_CXX_FLAGS
#    "${SSE_FLAGS} -O3 -g -std=c++14 -march=native -Wno-deprecated -Wno-deprecated-declarations" #-fopenmp is added if g2o is compiled with openmp capabilities
# #   "${SSE_FLAGS} -O3 -g -std=c++14 -fno-omit-frame-pointer"
# )
# #SET CXX FLAGS to use march antive and gcc
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -std=gnu++14")

# gd -dev -16june2026
set(CMAKE_GENERATOR "Unix Makefiles")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -march=native -Wno-deprecated -Wno-deprecated-declarations" #-fopenmp is added if g2o is compiled with openmp capabilities
)
#SET CXX FLAGS to use march antive and gcc
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# gd -dev -16june2026
if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

# Sources files
set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptimize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemDebugStuff.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemMarginalize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/Residuals.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseTracker.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseInitializer.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/ImmaturePoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/HessianBlocks.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/PixelSelector2.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp
  ${PROJECT_SOURCE_DIR}/src/util/DepthLogger.cpp
  ${PROJECT_SOURCE_DIR}/src/util/FPSLogger.cpp
  ${PROJECT_SOURCE_DIR}/src/ML/MLInference.cpp
  ${PROJECT_SOURCE_DIR}/src/ML/MLDepthProcessor.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/Detector.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/MapPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/Matcher.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/Map.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/IndirectTracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/Optimizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/Sim3Solver.cpp
  ${PROJECT_SOURCE_DIR}/src/Indirect/LoopCloser.cpp
# gd -dev -16june2026
  ${PROJECT_SOURCE_DIR}/src/GSIntegration/gaussian_mapper.cpp
  ${PROJECT_SOURCE_DIR}/src/GSIntegration/gaussian_mapper_utils.cpp
  ${PROJECT_SOURCE_DIR}/src/GSIntegration/gaussian_mapper_IO.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_rasterizer.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_scene.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_model_utils.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_parameters.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_keyframe.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_model.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_renderer.cpp
  ${PROJECT_SOURCE_DIR}/3DGS/src/gaussian_trainer.cpp
  ${PROJECT_SOURCE_DIR}/Thirdparty/tinyply/tinyply.cpp   # gd -dev -17june2026
# gd -dev -16june2026
)



include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/Thirdparty/sse2neon
  ${PROJECT_SOURCE_DIR}/Thirdparty/tum_benchmark
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${ONNXRUNTIME_INCLUDE_DIR}
# gd -dev -16june2026
  ${PROJECT_SOURCE_DIR}
  ${TORCH_INCLUDE_DIRS}
# gd -dev -16june2026
) 


# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling pangolin library.")
	include_directories( ${Pangolin_INCLUDE_DIRS} ) 
	set(pangolin_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp)
	set(HAS_PANGOLIN 1)
else ()
	message("--- could not find PANGOLIN, not compiling pangolin library.")
	message("    this means there will be no 3D display / GUI available.")
	set(pangolin_SOURCE_FILES )
	set(HAS_PANGOLIN 0)
endif ()

# decide if we have openCV
if (OpenCV_FOUND)
	message("--- found OpenCV, compiling with opencv library.")
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	set(opencv_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp)
	set(HAS_OPENCV 1)
else ()
	message("--- could not find OpenCV, not compiling opencv library.")
	message("    this means there will be no image display, and image read / load functionality.")
	set(opencv_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ImageDisplay_dummy.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ImageRW_dummy.cpp)
	set(HAS_OPENCV 0)
endif ()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
	message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
	add_definitions(-DHAS_ZIPLIB=1)
	include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} ) 
else()
	message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
	set(LIBZIP_LIBRARY "")
endif()

# gd -dev -16june2026
# Compiled GS libraries
add_library(simple_knn SHARED
    ${PROJECT_SOURCE_DIR}/Thirdparty/simple-knn/simple_knn.cu
    ${PROJECT_SOURCE_DIR}/Thirdparty/simple-knn/simple_knn.h
    ${PROJECT_SOURCE_DIR}/Thirdparty/simple-knn/spatial.cu
    ${PROJECT_SOURCE_DIR}/Thirdparty/simple-knn/spatial.h)
target_link_libraries(simple_knn "${TORCH_LIBRARIES}")

add_library(cuda_rasterizer SHARED
    ${PROJECT_SOURCE_DIR}/3DGS/include/operate_points.h
    ${PROJECT_SOURCE_DIR}/3DGS/src/operate_points.cu
    ${PROJECT_SOURCE_DIR}/3DGS/include/rasterize_points.h
    ${PROJECT_SOURCE_DIR}/3DGS/src/rasterize_points.cu
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/auxiliary.h
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/backward.cu
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/backward.h
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/config.h
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/forward.cu
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/forward.h
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/operate_points.h
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/rasterizer.h
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/rasterizer_impl.cu
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/rasterizer_impl.h)
set_target_properties(cuda_rasterizer PROPERTIES CUDA_ARCHITECTURES "61;75;86")
target_include_directories(cuda_rasterizer PRIVATE
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
target_link_libraries(cuda_rasterizer
    glm::glm
    "${TORCH_LIBRARIES}")
# gd -dev -16june2026

# compile main library.
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}) 
add_library(HSLAM_lib ${SOURCE_FILES} ${opencv_SOURCE_FILES} ${pangolin_SOURCE_FILES})



if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()


set(LIBS
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
  ${LIBZIP_LIBRARY} 
  ${Pangolin_LIBRARIES}
  ${DBoW3_LIB_DIR}/libDBoW3.so
  ${BOOST_THREAD_LIBRARY}
# gd -dev -16junee2026
  ${TORCH_LIBRARIES}
  glm::glm
  simple_knn
  cuda_rasterizer
# gd -dev -16june2026
  boost_system
  cxsparse
  cholmod
)

# Add ONNX Runtime library if available
if(HAS_ONNXRUNTIME)
    list(APPEND LIBS ${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so)
endif()

# build main executable (only if we have both OpenCV and Pangolin)
if (OpenCV_FOUND AND Pangolin_FOUND)
	message("--- compiling executable.")
	add_executable(HSLAM ${PROJECT_SOURCE_DIR}/src/main.cpp )
    target_link_libraries(HSLAM HSLAM_lib ${LIBS} )
else()
	message("--- not building executable, since either don't have openCV or Pangolin.")
endif()

# GPU functionality tests (if ONNX Runtime is available)
option(BUILD_GPU_TESTS "Build GPU functionality tests" ON)
if(BUILD_GPU_TESTS AND HAS_ONNXRUNTIME)
    # Try to find Google Test
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        message("--- Google Test not found, attempting to download...")
        include(FetchContent)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        )
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()
    
    # GPU functionality tests disabled due to linking issues with ncurses/curl
    # if(GTest_FOUND OR TARGET gtest)
    #     message("--- Building GPU functionality tests")
    #     add_executable(gpu_functionality_tests ${PROJECT_SOURCE_DIR}/tests/gpu_functionality_tests.cpp)
    #     target_link_libraries(gpu_functionality_tests 
    #         HSLAM_lib 
    #         ${LIBS} 
    #         GTest::gtest 
    #         GTest::gtest_main
    #     )
    #     target_include_directories(gpu_functionality_tests PRIVATE
    #         ${PROJECT_SOURCE_DIR}/src
    #         ${EIGEN3_INCLUDE_DIR}
    #         ${ONNXRUNTIME_INCLUDE_DIR}
    #     )
        
        # Add test to CTest
        # enable_testing()
        # add_test(NAME gpu_functionality COMMAND gpu_functionality_tests)
        
        # Set test properties for CI
        # set_tests_properties(gpu_functionality PROPERTIES
        #     TIMEOUT 300  # 5 minute timeout
        #     ENVIRONMENT "CUDA_VISIBLE_DEVICES=0"
        # )
    # else()
    #     message("--- Could not configure Google Test, GPU tests will not be built")
    # endif()
else()
    if(NOT HAS_ONNXRUNTIME)
        message("--- GPU tests not built: ONNX Runtime not available")
    else()
        message("--- GPU tests disabled by BUILD_GPU_TESTS option")
    endif()
endif()