cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(wheel_speed_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(wheel_speed_plugin SHARED src/wheel_speed_plugin.cpp include/wheel_speed_plugin.hpp)
target_include_directories(wheel_speed_plugin PUBLIC include)

ament_target_dependencies(wheel_speed_plugin "gazebo_dev" "gazebo_ros" "rclcpp" "std_msgs" "geometry_msgs")

install(TARGETS wheel_speed_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(gazebo_ros rclcpp std_msgs geometry_msgs)
ament_package()
