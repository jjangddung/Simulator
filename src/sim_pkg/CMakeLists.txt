cmake_minimum_required(VERSION 3.5)
project(sim_pkg)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/set_initial_pose.py
  scripts/wheel_imu_odom.py
  scripts/localization_error_monitor.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
