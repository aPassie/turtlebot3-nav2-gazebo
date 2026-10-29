cmake_minimum_required(VERSION 3.8)
project(tb3_nav2_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/patrol.py scripts/plot_trajectory.py scripts/record_demo.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
