cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(vanjee_lidar_sdk)

#=======================================
# Compile Type (Release, Debug)
#=======================================
set(CMAKE_BUILD_TYPE "Release")

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZIRT)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD COLCON)

option(DISABLE_PCAP_PARSE         "Disable PCAP file parse" ON)
if(${DISABLE_PCAP_PARSE})

  message(=============================================================)
  message("-- Disable PCAP parse")
  message(=============================================================)

  add_definitions("-DDISABLE_PCAP_PARSE")

else()

  if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
  else()
    list(APPEND EXTERNAL_LIBS pcap)
  endif(WIN32)

endif(${DISABLE_PCAP_PARSE})

option(ENABLE_PCL_POINTCLOUD      "Enable PCL Point Cloud" OFF)
if(${ENABLE_PCL_POINTCLOUD})
  add_definitions("-DENABLE_PCL_POINTCLOUD")
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
endif(${ENABLE_PCL_POINTCLOUD})

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
 add_definitions("-DENABLE_TRANSFORM")

 find_package(Eigen3 REQUIRED)
 include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_GTEST "Enable gtest value" OFF)
if(${ENABLE_GTEST})
 add_definitions("-DENABLE_GTEST")
endif(${ENABLE_GTEST})

option(ENABLE_TBB "Enable TBB" OFF)

if(${ENABLE_TBB})
add_definitions("-DENABLE_TBB")
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIR})

endif(${ENABLE_TBB})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_DOUBLE_RCVBUF "Enable double size of RCVBUF" OFF)
if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

option(ENABLE_GRAVITY_ACCELERATION_REMOVE      "Enable Gravity acceleration remove" OFF)
if(${ENABLE_GRAVITY_ACCELERATION_REMOVE})
  add_definitions("-DENABLE_GRAVITY_ACCELERATION_REMOVE")
endif(${ENABLE_GRAVITY_ACCELERATION_REMOVE})

#========================
# Project details / setup
#========================
set(PROJECT_NAME vanjee_lidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  message("buile type: Release")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message("buile type: Debug")
else ()
  set(CMAKE_BUILD_TYPE "Release")
  message("buile type: Release")
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
add_definitions(-std=c++17)
else()
add_definitions(-std=c++14)
endif()

# add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
elseif(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
endif()

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Dependencies Setup
#========================

#ROS#
find_package(roscpp 1.12 QUIET)

if(roscpp_FOUND)

  message(=============================================================)
  message("-- ROS Found. ROS Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS_FOUND)

  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  #Catkin#
  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    add_definitions(-DRUN_IN_ROS_WORKSPACE)

    find_package(catkin REQUIRED COMPONENTS
      roscpp
      sensor_msgs
      roslib
      std_msgs
      message_generation)

      add_message_files(
        FILES
        DeviceCtrl.msg
      )

      generate_messages(
        DEPENDENCIES
        std_msgs)

    catkin_package(CATKIN_DEPENDS
      sensor_msgs
      roslib
      std_msgs
      message_runtime)

    include_directories( ${catkin_INCLUDE_DIRS}/ros_msg)

  endif(${COMPILE_METHOD} STREQUAL "CATKIN")

else(roscpp_FOUND)

  message(=============================================================)
  message("-- ROS Not Found. ROS Support is turned Off.")
  message(=============================================================)

endif(roscpp_FOUND)

#ROS2#
find_package(rclcpp QUIET)

if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS2_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(vanjee_lidar_msg REQUIRED)
  find_package(std_msgs REQUIRED)

else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Not Found. ROS2 Support is turned Off.")
  message(=============================================================)

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)
MESSAGE(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")

#Driver core#
add_subdirectory(src/vanjee_driver)
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================

add_executable(vanjee_lidar_sdk_node
               node/vanjee_lidar_sdk_node.cpp
               src/manager/node_manager.cpp)

target_link_libraries(vanjee_lidar_sdk_node
                      ${YAML_CPP_LIBRARIES}
                      ${vanjee_driver_LIBRARIES})

if(${ENABLE_TBB})
target_link_libraries(vanjee_lidar_sdk_node
                      TBB::tbb)
endif(${ENABLE_TBB})

MESSAGE(STATUS "vanjee_driver_LIBRARIES = ${vanjee_driver_LIBRARIES}")

#Ros#
if(roscpp_FOUND)

  target_link_libraries(vanjee_lidar_sdk_node
    ${ROS_LIBS})

  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    install(TARGETS vanjee_lidar_sdk_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

    install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
    install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
    )
    install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )

  endif()

endif(roscpp_FOUND)

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  ament_target_dependencies(vanjee_lidar_sdk_node
    rclcpp
    std_msgs
    sensor_msgs
    vanjee_lidar_msg)

  install(TARGETS
    vanjee_lidar_sdk_node
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    config
    param
    DESTINATION share/${PROJECT_NAME})

  ament_package()

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")



