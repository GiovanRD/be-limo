cmake_minimum_required(VERSION 3.8)

project(agilex_sand_table) # Project name, should match the package name. ${PROJECT_NAME} = agilex_sand_table

find_package(ament_cmake REQUIRED) # Find the ament_cmake package, which is required for building ROS 2 packages.

# Install the models, worlds and launch directories into the share directory of the package.
install(
  DIRECTORY
    models
    worlds
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the environment hook (when you source the package, set also the GAZEBO_MODEL_PATH environment variable).
ament_environment_hooks(env/agilex_sand_table.dsv)

# Configure the package to be built with ament.
ament_package()
