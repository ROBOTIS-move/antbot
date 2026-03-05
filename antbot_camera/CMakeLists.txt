cmake_minimum_required(VERSION 3.8)
project(antbot_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orbbec_camera REQUIRED)

# Novitec RECTRL library
set(RECTRL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(RECTRL_LIB "${RECTRL_LIB_DIR}/librectrl.so")

add_library(${PROJECT_NAME}_drivers SHARED
  src/camera_driver.cpp
  src/v4l2_driver.cpp
  src/usb_driver.cpp
  src/vendor/camera_vendor_handler.cpp
  src/vendor/novitec_camera_handler.cpp
  src/vendor/generic_camera_handler.cpp
  src/vendor/rectrl_wrapper.cpp
)
target_include_directories(${PROJECT_NAME}_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_drivers
  rclcpp sensor_msgs std_msgs cv_bridge OpenCV
)
target_link_libraries(${PROJECT_NAME}_drivers dl ${RECTRL_LIB})

add_executable(antbot_camera_node src/camera_node.cpp)
target_link_libraries(antbot_camera_node ${PROJECT_NAME}_drivers)
ament_target_dependencies(antbot_camera_node rclcpp rclcpp_components)

add_executable(camera_test_node src/camera_test_node.cpp)
ament_target_dependencies(camera_test_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_drivers LIBRARY DESTINATION lib)
install(TARGETS antbot_camera_node camera_test_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${RECTRL_LIB} DESTINATION lib)

ament_package()
