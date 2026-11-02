cmake_minimum_required(VERSION 3.8)

project(qcar_large_track) # Project name, should match the package name. ${PROJECT_NAME} = qcar_large_track

find_package(ament_cmake REQUIRED) # Find the ament_cmake package, which is required for building ROS 2 packages.

# Install the models, worlds, launch, and rviz directories into the share directory of the package.
install(
  DIRECTORY
    models
    worlds
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the environment hook (when you source the package, set also the GAZEBO_MODEL_PATH and GAZEBO_RESOURCE_PATH environment variables).
ament_environment_hooks(env/qcar_large_track.dsv)

# Configure the package to be built with ament.
ament_package()