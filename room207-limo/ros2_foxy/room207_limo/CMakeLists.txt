cmake_minimum_required(VERSION 3.8)

project(room207_limo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS room207_limo/room207_simu_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME room207_simu_gui
)

install(
  PROGRAMS
    scripts/plot_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_odometry
)

ament_package()