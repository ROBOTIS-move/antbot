cmake_minimum_required(VERSION 3.16)
project(antbot_swerve_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  sensor_msgs
  rcpputils
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

# Generate parameter library first
generate_parameter_library(swerve_drive_controller_parameter
  config/swerve_drive_controller_parameter.yaml
)

# Ensure the parameter library target exists and is built first
if(NOT TARGET swerve_drive_controller_parameter)
  message(FATAL_ERROR "swerve_drive_controller_parameter target was not created")
endif()

add_library(antbot_swerve_controller
  SHARED
  src/swerve_drive_controller.cpp
  src/swerve_motion_control.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(antbot_swerve_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(antbot_swerve_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  antbot_swerve_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(antbot_swerve_controller swerve_drive_controller_parameter)
# Ensure parameter library is built first
add_dependencies(antbot_swerve_controller swerve_drive_controller_parameter)

# Get the include directories from the parameter library target
get_target_property(PARAM_LIB_INCLUDE_DIRS swerve_drive_controller_parameter INTERFACE_INCLUDE_DIRECTORIES)
if(PARAM_LIB_INCLUDE_DIRS)
  target_include_directories(antbot_swerve_controller PUBLIC ${PARAM_LIB_INCLUDE_DIRS})
endif()

# Export the plugin definition file
pluginlib_export_plugin_description_file(controller_interface antbot_swerve_controller.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(antbot_swerve_controller PRIVATE "ANTBOT_SWERVE_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS antbot_swerve_controller swerve_drive_controller_parameter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install the plugin XML file
install(FILES antbot_swerve_controller.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  antbot_swerve_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()
