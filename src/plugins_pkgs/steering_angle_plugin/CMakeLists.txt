cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(steering_angle_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(steering_angle_plugin SHARED src/steering_angle_plugin.cpp include/steering_angle_plugin.hpp)
target_include_directories(steering_angle_plugin PUBLIC include)

ament_target_dependencies(steering_angle_plugin "gazebo_dev" "gazebo_ros" "rclcpp" "std_msgs")

install(TARGETS steering_angle_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(gazebo_ros rclcpp std_msgs)
ament_package()
