cmake_minimum_required(VERSION 3.8)
project(oculus_gz_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${gazebo_ros_INCLUDE_DIRS}
)

#=============
# Camera Gazebo plugin
#=============

add_library(
  CameraPositionPlugin SHARED
  src/camera_plugin.cpp
)
target_include_directories(CameraPositionPlugin PUBLIC include)
ament_target_dependencies(
  CameraPositionPlugin
  geometry_msgs
  gazebo_dev
  gazebo_ros
  rclcpp
)
ament_export_libraries(CameraPositionPlugin)

add_library(
  CameraHeadsetPlugin SHARED
  src/camera_headset_plugin.cpp
)
target_include_directories(CameraHeadsetPlugin PUBLIC include)
ament_target_dependencies(
  CameraHeadsetPlugin
  geometry_msgs
  gazebo_dev
  gazebo_ros
  rclcpp
)
ament_export_libraries(CameraHeadsetPlugin)

#=============
# Install
#=============

# Install targets
install(TARGETS 
  CameraPositionPlugin 
  CameraHeadsetPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install world and launch files
install(
  DIRECTORY models worlds launch
  DESTINATION share/${PROJECT_NAME}
)

#=============
# Run ament macros
#=============

ament_package()
