cmake_minimum_required(VERSION 3.0.2)
project(da_livgs_frontend)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  eigen_conversions
  cv_bridge
  roslib
  rosbag
  tf
  message_generation
  image_transport
)

# FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization thread)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(Sophus REQUIRED)
if(DEFINED ENV{CUDA_HOME} AND NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
  set(CUDA_TOOLKIT_ROOT_DIR "$ENV{CUDA_HOME}")
endif()
if(DEFINED ENV{OpenCV_DIR} AND NOT OpenCV_DIR)
  set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV CMake directory")
endif()
if(DEFINED ENV{PCL_DIR} AND NOT PCL_DIR)
  set(PCL_DIR "$ENV{PCL_DIR}" CACHE PATH "PCL CMake directory")
endif()
find_package(OpenCV REQUIRED)
find_package(PCL 1.13.0 REQUIRED)

# find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  feature_cloud.msg
  imu_array.msg
  pose_array.msg
  DegradationStatus.msg
  GaussianPoseFeedback.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS message_runtime std_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  src/camera
  src/camera/loam/include
  src/camera/tools/
  src/camera/rgb_map
)
list(APPEND thirdparty_libraries
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SERIALIZATION_LIBRARY} # serialization
)

add_library(spline_lib src/spline/trajectory.cpp)
target_link_libraries(spline_lib ${thirdparty_libraries})

file(GLOB lidar_odometry_files
    "src/lidar/lidar_handler.cpp"
    "src/lidar/velodyne_feature_extraction.cpp"
    "src/lidar/livox_feature_extraction.cpp"
)
add_library(lidar_lib ${lidar_odometry_files})
add_dependencies(lidar_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
                 livox_ros_driver_generate_messages_cpp)
target_link_libraries(lidar_lib spline_lib ${thirdparty_libraries})

file(GLOB r3live_files
    "src/camera/loam/*.cpp"
    "src/camera/optical_flow/*.cpp"
    "src/camera/rgb_map/*.cpp"
    "src/camera/*.cpp"
)
list(FILTER r3live_files EXCLUDE REGEX ".*_test\\.cpp$")
add_library(r3live_lib ${r3live_files})
add_dependencies(r3live_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
                 livox_ros_driver_generate_messages_cpp)
target_link_libraries(r3live_lib ${thirdparty_libraries})

add_executable(da_livgs_odometry 
   src/odometry_node.cpp
   src/imu/imu_state_estimator.cpp
   src/imu/imu_initializer.cpp
   src/odom/msg_manager.cpp
   src/odom/trajectory_manager.cpp
   src/odom/trajectory_estimator.cpp
   src/odom/odometry_manager.cpp
   src/odom/observability/observability_analyzer.cpp
   src/odom/observability/degradation_controller.cpp
   src/odom/observability/support_conditioned_factor_policy.cpp
   src/odom/observability/persistent_degeneracy_policy.cpp
   src/odom/observability/visual_factor_routing_policy.cpp
   src/odom/observability/directional_fusion.cpp
   src/odom/observability/gaussian_endpoint_prior_factor.cpp
   src/odom/observability/gaussian_endpoint_prior_shadow.cpp
   src/odom/observability/gaussian_endpoint_replay.cpp
   src/odom/observability/gaussian_endpoint_trust_region.cpp
   src/odom/observability/gaussian_mapping_pose_overlay.cpp
   src/odom/observability/gaussian_feedback_shadow.cpp
   src/odom/observability/gaussian_feedback_transport.cpp
   src/odom/visual/pnp_pose_measurement.cpp
   src/odom/factor/analytic_diff/marginalization_factor.cpp
   src/utils/parameter_struct.cpp
)
add_dependencies(da_livgs_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
                 livox_ros_driver_generate_messages_cpp)

target_link_libraries(da_livgs_odometry 
   spline_lib 
   lidar_lib 
   r3live_lib
   ${thirdparty_libraries}
  #  fmt::fmt
)
