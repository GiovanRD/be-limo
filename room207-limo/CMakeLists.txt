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
  PROGRAMS
    room207_limo/recording_simulation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME recording_simulation
)

install(
  PROGRAMS
    scripts/plot_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_odometry
)

ament_package()