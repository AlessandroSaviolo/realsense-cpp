cmake_minimum_required(VERSION 3.6)
project(realsense_camera)

cmake_policy(SET CMP0057 NEW)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense_msgs REQUIRED)

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  Eigen3
  CUDA
  realsense2
  realsense_msgs
)

if (NOT CUDA_INCLUDE_DIRS)
  set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/include)
endif()
if (NOT CUDA_LIBRARIES)
  set(CUDA_LIBRARIES ${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/lib)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
)

add_library(rs_camera SHARED src/nodelet.cpp)
set_target_properties(rs_camera PROPERTIES COMPILE_DEFINITIONS "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(rs_camera ${dependencies})

target_link_libraries(rs_camera
  realsense2::realsense2
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_nodes(rs_camera "realsense_camera::RSCameraNodelet")

# Export dependencies and targets
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(rs_camera)

# Installation commands
install(TARGETS
  rs_camera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
