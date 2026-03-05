cmake_minimum_required(VERSION 3.5)
project(antbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/CargoStatus.msg"
)

set(srv_files
  "srv/CargoCommand.srv"
  "srv/DirectRead.srv"
  "srv/DirectWrite.srv"
  "srv/WiperOperation.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
