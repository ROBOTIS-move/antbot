cmake_minimum_required(VERSION 3.10)

project(vanjee_lidar_driver_lib_demo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

option(ENABLE_PCL_POINTCLOUD      "Enable PCL Point Cloud" OFF)
if(${ENABLE_PCL_POINTCLOUD})
message(=============================================================)
message("-- ENABLE_PCL_POINTCLOUD")
message(=============================================================)
add_definitions("-DENABLE_PCL_POINTCLOUD")
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
endif(${ENABLE_PCL_POINTCLOUD})

find_package(yaml-cpp REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/api
                    ${PROJECT_SOURCE_DIR}/msg
                    ${PROJECT_SOURCE_DIR}/common)

# Add executable
add_executable(vanjee_lidar_driver_lib_demo vanjee_lidar_driver_lib_demo.cpp)

target_link_libraries(vanjee_lidar_driver_lib_demo 
                        vanjee_lidar_driver_lib pthread ${EXTERNAL_LIBS} ${YAML_CPP_LIBRARIES})
