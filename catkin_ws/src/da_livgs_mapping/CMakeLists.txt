cmake_minimum_required(VERSION 3.0.2)
project(da_livgs_mapping)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
if(CMAKE_VERSION VERSION_LESS 3.18)
  set(CMAKE_CUDA17_STANDARD_COMPILE_OPTION "--std=c++17")
  set(CMAKE_CUDA17_EXTENSION_COMPILE_OPTION "--std=c++17")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  eigen_conversions
  da_livgs_frontend
)

set(DA_LIVGS_DEPENDENCY_ROOT "$ENV{DA_LIVGS_DEPENDENCY_ROOT}" CACHE PATH
    "Root directory for locally installed third-party libraries")
if(DA_LIVGS_DEPENDENCY_ROOT)
  list(APPEND CMAKE_PREFIX_PATH "${DA_LIVGS_DEPENDENCY_ROOT}")
endif()

set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV build directory")
if(DEFINED ENV{CUDA_HOME} AND NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
  set(CUDA_TOOLKIT_ROOT_DIR "$ENV{CUDA_HOME}")
endif()
find_package(OpenCV 4 REQUIRED)

set(CUDA_TOOLKIT_ROOT_DIR "$ENV{CUDA_HOME}" CACHE PATH "CUDA toolkit directory")
if(CUDA_TOOLKIT_ROOT_DIR)
  set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc" CACHE FILEPATH "CUDA compiler")
endif()
set(TORCH_CUDA_ARCH_LIST "8.6" CACHE STRING "CUDA architectures for LibTorch")
find_package(CUDA REQUIRED)
if(NOT CUDA_LIB_DIR AND CUDA_TOOLKIT_ROOT_DIR)
  set(CUDA_LIB_DIR "${CUDA_TOOLKIT_ROOT_DIR}/lib64")
endif()

set(Torch_DIR "$ENV{Torch_DIR}" CACHE PATH "LibTorch CMake directory")
find_package(Torch REQUIRED)

set(TENSORRT_ROOT "$ENV{TENSORRT_ROOT}" CACHE PATH "TensorRT installation directory")
if(TENSORRT_ROOT)
  set(TENSORRT_INCLUDE_DIR "${TENSORRT_ROOT}/include")
  set(TENSORRT_LIB_DIR "${TENSORRT_ROOT}/lib")
endif()
set(GLM_INCLUDE_DIR "$ENV{GLM_INCLUDE_DIR}" CACHE PATH "GLM include directory")

find_library(FFI_LIB ffi)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES da_livgs_mapping
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIR}
)

add_executable(da_livgs_mapping 
              ${PROJECT_SOURCE_DIR}/src/mapping.cpp 
              ${PROJECT_SOURCE_DIR}/src/map_guard_state_machine.cpp
              ${PROJECT_SOURCE_DIR}/src/gaussian_feedback_observer.cpp
              ${PROJECT_SOURCE_DIR}/src/gaussian_feedback_projector.cpp
              ${PROJECT_SOURCE_DIR}/src/pose_perturbation.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_completion_confidence.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_completion_corruption.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_geometry_consistency.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_temporal_consistency.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_soft_initialization.cpp
              ${PROJECT_SOURCE_DIR}/src/provisional_lifecycle.cpp
              ${PROJECT_SOURCE_DIR}/src/mapping_update_response_observer.cpp
              ${PROJECT_SOURCE_DIR}/src/frequency_balanced_loss.cpp
              ${PROJECT_SOURCE_DIR}/src/affine_exposure.cpp
              ${PROJECT_SOURCE_DIR}/src/appearance_conditioner.cpp
              ${PROJECT_SOURCE_DIR}/src/appearance_execution_scheduler.cpp
              ${PROJECT_SOURCE_DIR}/src/appearance_timing_observer.cpp
              ${PROJECT_SOURCE_DIR}/src/appearance_feature_lifecycle.cpp
              ${PROJECT_SOURCE_DIR}/src/appearance_residual_cache.cpp
              ${PROJECT_SOURCE_DIR}/src/spherical_harmonics.cpp
              ${PROJECT_SOURCE_DIR}/src/appearance_gradient_router.cpp
              ${PROJECT_SOURCE_DIR}/src/gradient_decomposition.cpp
              ${PROJECT_SOURCE_DIR}/src/mapping_randomness.cpp
              ${PROJECT_SOURCE_DIR}/src/mature_refinement_observer.cpp
              ${PROJECT_SOURCE_DIR}/src/mature_refinement_mutation.cpp
              ${PROJECT_SOURCE_DIR}/src/projected_photometric_utility.cpp
              ${PROJECT_SOURCE_DIR}/src/mature_refinement_response_observer.cpp
              ${PROJECT_SOURCE_DIR}/src/adaptive_backward_scheduler.cpp
              ${PROJECT_SOURCE_DIR}/src/per_view_metrics.cpp
              ${PROJECT_SOURCE_DIR}/src/gaussian.cpp
              ${PROJECT_SOURCE_DIR}/src/tinyply.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_completer.cpp 
              ${PROJECT_SOURCE_DIR}/src/simple-knn/spatial.cu
              ${PROJECT_SOURCE_DIR}/src/simple-knn/simple_knn.cu
              ${PROJECT_SOURCE_DIR}/src/fused-ssim/ssim.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/renderer.cpp
              ${PROJECT_SOURCE_DIR}/src/rasterizer/rasterizer.cpp
              ${PROJECT_SOURCE_DIR}/src/rasterizer/rasterize_points.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/adam.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/forward.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/backward.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/rasterizer_impl.cu)
target_link_libraries(da_livgs_mapping 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS} 
                      ${TORCH_LIBRARIES} 
                      ${YAML_CPP_LIBRARIES} 
                      ${Python3_LIBRARIES}
                      ${FFI_LIB}
                      ${CUDA_LIB_DIR}/libcudart.so
                      ${TENSORRT_LIB_DIR}/libnvinfer.so
                      ${TENSORRT_LIB_DIR}/libnvonnxparser.so
                      ${TENSORRT_LIB_DIR}/libnvparsers.so
                      ${TENSORRT_LIB_DIR}/libnvinfer_plugin.so)
add_dependencies(da_livgs_mapping ${catkin_EXPORTED_TARGETS})
