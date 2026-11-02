cmake_minimum_required(VERSION 3.8)
project(ensem_room207)

find_package(ament_cmake REQUIRED)

# Gazebo GUI plugin hiding the light markers. Built only when Gazebo is
# installed (the physical LIMO has no Gazebo).
find_package(gazebo QUIET)
find_package(Qt5Widgets QUIET)
if(gazebo_FOUND AND Qt5Widgets_FOUND)
  add_library(ensem_room207_hide_light_markers SHARED src/hide_light_markers.cc)
  target_include_directories(ensem_room207_hide_light_markers SYSTEM PRIVATE
    ${GAZEBO_INCLUDE_DIRS})
  target_link_directories(ensem_room207_hide_light_markers PRIVATE ${GAZEBO_LIBRARY_DIRS})
  target_link_libraries(ensem_room207_hide_light_markers ${GAZEBO_LIBRARIES} Qt5::Widgets)
  target_compile_options(ensem_room207_hide_light_markers PRIVATE ${GAZEBO_CXX_FLAGS})
  install(TARGETS ensem_room207_hide_light_markers LIBRARY DESTINATION lib)
endif()

install(
  DIRECTORY
    models
    worlds
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Add the room models to GAZEBO_MODEL_PATH and the GUI plugin to
# GAZEBO_PLUGIN_PATH when the workspace is sourced.
ament_environment_hooks(env/ensem_room207.dsv)

ament_package()
