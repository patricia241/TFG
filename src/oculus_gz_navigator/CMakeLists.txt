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
find_package(gazebo_msgs REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(dependencies
    rclcpp
    gazebo
    gazebo_ros
    gazebo_dev
    gazebo_msgs
)

include_directories(include)

add_library(hello_world_plugin SHARED
  src/hello_world_plugin.cpp
)
target_include_directories(hello_world_plugin PUBLIC include)
ament_target_dependencies(hello_world_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_gridmap)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch models worlds
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  hello_world_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

ament_package()
