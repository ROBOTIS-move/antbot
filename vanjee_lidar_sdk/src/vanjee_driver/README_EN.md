# **vanjee_driver** 

[English Version](README.md) 

## 1 Introduction

**vanjee_driver**is the driver for VanJee LiDAR.

## 2 Supported LiDAR Models

The supported LiDAR models are as follows.
- vanjee_716mini
- vanjee_718h
- vanjee_719
- vanjee_719c
- vanjee_719e
- vanjee_720 / vanjee_720_16
- vanjee_720_32
- vanjee_721
- vanjee_722
- vanjee_733
- vanjee_750
- vanjee_760

## 3 Supported Operating Systems

The supported operating systems and compilers are as follows. Note that the compiler must support the C++14 standard.
- Ubuntu (16.04, 18.04, 20.04)
  - gcc (4.8+)

- Windows
  - MSVC (Tested on Win10 / VS2019)

## 4 Dependent Third-Party Libraries

The dependent third-party libraries are as follows.
- libpcap (Optional. Can be ignored if PCAP files do not need to be parsed)
- Eigen3 (Required. Linux installation reference: 5.1; Windows installation reference: 6.1)
- PCL (Optional. Can be ignored if visualization tools are not needed.)
- Boost (Optional. Can be ignored if visualization tools are not required.)

## 5 Compilation and Installation on Ubuntu
### 5.1 Installing Third-Party Libraries

```bash
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```
### 5.2 Compilation

```bash
cd vanjee_driver
mkdir build && cd build
cmake .. && make -j4
```

### 5.3 Installation

```bash
sudo make install
```

### 5.4 Usage as a Third-Party Library

Configure your `CMakeLists` file to find the **vanjee_driver** library using the `find_package()` command and link it.

```cmake
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${vanjee_driver_LIBRARIES})
```

### 5.5 Usage as a Submodule

Add **vanjee_driver** as a submodule to your project and configure your `CMakeLists` file accordingly.

Use the `find_package()` command to locate the **vanjee_driver** library and link it.

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/vanjee_driver)
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})
target_link_libraries(project ${vanjee_driver_LIBRARIES})
```

## 6 Compilation and Installation on Windows

### 6.1 Installing Third-Party Libraries

#### libpcap

Install [libpcap Runtime Library](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe)。

Extract the [libpcap developer package](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip) to any location, and add the path of `WpdPack_4_1_2/WpdPack` to the environment variable `PATH`.

Install Eigen3(https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

#### PCL

If using the MSVC compiler, you can use the [PCL installer package](https://github.com/PointCloudLibrary/pcl/releases) provided by the PCL official site.

During installation, select "Add PCL to the system PATH for xxx":

### 6.2 Compiling vanjee_driver

Users can refer to.[ How to Compile vanjee_driver on Windows for guidance](./src/vanjee_lidar_sdk/doc/howto/07_how_to_compile_on_windows.md)

### 6.3 Installation

On Windows, **vanjee_driver** does not support installation at this time.

## 7 Quick Start

**vanjee_driver** provides three example programs located in the `vanjee_driver/demo` directory.

- demo_online.cpp
- demo_online_multi_lidars.cpp
- demo_pcap.cpp

`demo_online` parses data from an online LiDAR and outputs the point cloud, `demo_online_dual` parses data from two online LiDARs simultaneously and outputs the point cloud, and `demo_pcap` parses PCAP files and outputs the point cloud.

`demo_pcap` is based on the libpcap library.

**Before proceeding with the following steps to compile the demos, please ensure that you have correctly completed the SDK compilation and installation as described in step 5.**

Please complete the demo compilation using one of the following methods.

```bash
cd vanjee_driver
mkdir build && cd build
cmake -DCOMPILE_DEMOS=ON .. 
make -j4
```
Or

```bash
cd /vanjee_driver/demo
mkdir build && cd build
cmake ..
make -j4
```

For more information about `demo_online`, refer to[Online LiDAR Connection](doc/howto/02_how_to_decode_online_lidar.md)

For more information about `demo_pcap`, refer to[Decoding PCAP Files](doc/howto/04_how_to_decode_pcap_file.md)

## 8 Visualization Tools

**vanjee_driver** provides a point cloud visualization tool, `vanjee_driver_viewer`, located in the `vanjee_driver/tool` directory. It is based on the PCL library.

**Before proceeding with the following steps to compile the demo, please ensure that you have correctly completed the SDK compilation and installation as described in step 5.**

Please complete the tool compilation using one of the following methods.

```bash
cd vanjee_driver
mkdir build && cd build
cmake -DCOMPILE_TOOLS=ON .. 
make -j4
```
Or

```bash
cd /vanjee_driver/tool
mkdir build && cd build
cmake ..
make -j4
```

For usage instructions on `vanjee_driver_viewer`, please refer to the [Visualization Tool User Guide](doc/howto/06_how_to_use_vanjee_driver_viewer.md).

## 9 Compile into dynamic library

**vanjee_driver**Provides a method for compiling drivers into dynamic libraries,located in the```vanjee_driver/lib/lib``` directory.
**vanjee_driver**Provided a demo for calling dynamic libraries,located in the```vanjee_driver/lib/demo```directory.

**The data type of point cloud storage used for compiling into dynamic libraries and calling demos from dynamic libraries must be consistent (the definition 'ENABLE-PCL_POINTCLOUD' must be consistent)**

## 9.1 Compile and Install

Please complete the dynamic library compilation in one of the following ways.

```bash
cd vanjee_driver
mkdir build && cd build
cmake -DCOMPILE_LIB=ON .. 
make -j4
```
或

```bash
cd /vanjee_driver/lib/lib
mkdir build && cd build
cmake ..
make -j4
```

After compilation, it will generate ```libvanjee_lidar_driver_lib.so```，You can install the dynamic library by following these steps.
```bash
sudo cp libvanjee_lidar_driver_lib.so /usr/local/lib
```

## 9.2 Dynamic library call

**Before performing the following steps to compile the demo, please ensure that the configuration file path "config.path" in the main function is configured correctly.please have used absolute paths.**

Please complete the dynamic library call as follows.

**"libvanjee_lidar_driver_lib.so" is not in the standard library path of the system, and the "LD_LIBRRY-PATH" environment variable needs to be set to tell the dynamic linker where to look for it**
**In the bash shell, you can do this```export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH```**

```bash
cd /vanjee_driver/lib/demo
mkdir build && cd build
cmake ..
make -j4
```

After compilation is complete, an executable file named ```vanjee_lidar_driver_lib_demo``` will be generated.
Follow the steps below to run the demo.

```bash
./vanjee_lidar_driver_lib_demo
```

## 10 More Topics

For other topics related to **vanjee_driver**, please refer to the following links.

Coordinate Transformation: [Coordinate Transformation](doc/howto/05_how_to_use_coordinate_transformation_CN.md)
Advanced Topics in Network Configuration: [Advanced Topics](doc/howto/03_online_lidar_advanced_topics_CN.md)

The main interface files for **vanjee_driver** are as follows.

- Point Cloud Message Definition: ```vanjee_driver/src/vanjee_driver/msg/point_cloud_msg.hpp```, ```vanjee_driver/src/vanjee_driver/msg/pcl_point_cloud_msg.hpp```
- Interface Definition: ```vanjee_driver/src/vanjee_driver/api/lidar_driver.hpp```
- Parameter Definition: ```vanjee_driver/src/vanjee_driver/driver/driver_param.hpp```
- Error Code Definition: ```vanjee_driver/src/vanjee_driver/common/error_code.hpp```

