# **vanjee_driver** 

[English Version](README.md) 

## 1 简介

**vanjee_driver**为万集激光雷达驱动。

## 2 支持的雷达型号

支持的雷达型号如下。
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

## 3 支持的操作系统

支持的操作系统及编译器如下。注意编译器需支持C++14标准。
- Ubuntu (16.04, 18.04, 20.04)
  - gcc (4.8+)

- Windows
  - MSVC  (Win10 / VS2019 已测试)

## 4 依赖的第三方库

依赖的第三方库如下。
- libpcap (可选。如不需要解析PCAP文件，可忽略)
- Eigen3 (需要。linux安装参考:5.1;windows安装参考:6.1)
- PCL (可选。如不需要可视化工具，可忽略)
- Boost (可选。如不需要可视化工具，可忽略)

## 5 Ubuntu下的编译及安装
### 5.1 安装第三方库

```bash
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```
### 5.2 编译

```bash
cd vanjee_driver
mkdir build && cd build
cmake .. && make -j4
```

### 5.3 安装

```bash
sudo make install
```

### 5.4 作为第三方库使用

配置您的```CMakeLists```文件，使用find_package()指令找到**vanjee_driver**库，并链接。

```cmake
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${vanjee_driver_LIBRARIES})
```

### 5.5 作为子模块使用

将**vanjee_driver**作为子模块添加到您的工程，相应配置您的```CMakeLists```文件。

使用find_package()指令找到**vanjee_driver**库，并链接。

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/vanjee_driver)
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})
target_link_libraries(project ${vanjee_driver_LIBRARIES})
```

## 6 Windows下的编译及安装

### 6.1 安装第三方库

#### libpcap

安装[libpcap运行库](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe)。

解压[libpcap开发者包](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip)到任意位置，并将```WpdPack_4_1_2/WpdPack``` 的路径添加到环境变量```PATH```。

安装Eigen3(https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

#### PCL

如果使用MSVC编译器，可使用PCL官方提供的[PCL安装包](https://github.com/PointCloudLibrary/pcl/releases)安装。

安装过程中选择 “Add PCL to the system PATH for xxx”:

![](./doc/img/01_install_pcl.PNG)

### 6.2 编译vanjee_driver

可以参考[如何在Windows上编译vanjee_driver](./src/vanjee_lidar_sdk/doc/howto/07_how_to_compile_on_windows_CN.md)

### 6.3 安装

Windows下，**vanjee_driver** 暂不支持安装。

## 7 快速上手

**vanjee_driver**在目录```vanjee_driver/demo``` 下，提供了三个使用示例程序。

- demo_online.cpp
- demo_online_multi_lidars.cpp
- demo_pcap.cpp

`demo_online`解析在线雷达的数据，输出点云，`demo_online`同时解析两台在线雷达的数据，输出点云， `demo_pcap`解析PCAP文件，输出点云。

`demo_pcap`基于libpcap库。

**在执行以下步骤编译demo之前，请先确保已经按步骤5正确完成SDK编译和安装**

请按照以下方式之一完成demo编译

```bash
cd vanjee_driver
mkdir build && cd build
cmake -DCOMPILE_DEMOS=ON .. 
make -j4
```
或

```bash
cd /vanjee_driver/demo
mkdir build && cd build
cmake ..
make -j4
```

关于`demo_online`的更多说明，可以参考[在线连接雷达](doc/howto/02_how_to_decode_online_lidar.md)

关于`demo_pcap`的更多说明，可以参考[解析pcap包](doc/howto/04_how_to_decode_pcap_file.md)

## 8 可视化工具

**vanjee_driver**在目录```vanjee_driver/tool``` 下，提供了一个点云可视化工具`vanjee_driver_viewer`。它基于PCL库。

**在执行以下步骤编译demo之前，请先确保已经按步骤5正确完成SDK编译和安装**

请按照以下方式之一完成tool编译

```bash
cd vanjee_driver
mkdir build && cd build
cmake -DCOMPILE_TOOLS=ON .. 
make -j4
```
或

```bash
cd /vanjee_driver/tool
mkdir build && cd build
cmake ..
make -j4
```

关于`vanjee_driver_viewer`的使用方法，请参考[可视化工具操作指南](doc/howto/06_how_to_use_vanjee_driver_viewer.md) 

## 9 编译成动态库

**vanjee_driver**在目录```vanjee_driver/lib/lib``` 下，提供了将驱动编译成动态库方法。
**vanjee_driver**在目录```vanjee_driver/lib/demo``` 下，提供了动态库调用demo。
**编译成动态库和动态库调用demo使用的点云存储数据类型须保持一致(宏定义 `ENABLE_PCL_POINTCLOUD` 须保持一致)**

## 9.1 编译和安装

请按照以下方式之一完成动态库编译。

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

编译完成后会生成 ```libvanjee_lidar_driver_lib.so```，可通过以下步骤安装动态库。
```bash
sudo cp libvanjee_lidar_driver_lib.so /usr/local/lib
```

## 9.2 动态库调用

**在执行以下步骤编译demo之前，请先确保main函数中配置文件路径(config_path)是否配置正确(请使用过绝对路径)**

请按照以下方式完成动态库调用。

**libvanjee_lidar_driver_lib.so不在系统的标准库路径中，需要设置LD_LIBRARY_PATH环境变量来告诉动态链接器在哪里查找它。**
**在bash shell中，你可以这样做```export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH```**

```bash
cd /vanjee_driver/lib/demo
mkdir build && cd build
cmake ..
make -j4
```

编译完成后会生成可执行文件 ```vanjee_lidar_driver_lib_demo```，按以下步骤即可运行demo。

```bash
./vanjee_lidar_driver_lib_demo
```

## 10 更多主题

关于**vanjee_driver**的其他主题，请参考如下链接。

坐标变换: [坐标变换](doc/howto/05_how_to_use_coordinate_transformation_CN.md) 
网络配置的高级主题: [高级主题](doc/howto/03_online_lidar_advanced_topics_CN.md) 

**vanjee_driver**的主要接口文件如下。

- 点云消息定义: ```vanjee_driver/src/vanjee_driver/msg/point_cloud_msg.hpp```, ```vanjee_driver/src/vanjee_driver/msg/pcl_point_cloud_msg.hpp```
- 接口定义: ```vanjee_driver/src/vanjee_driver/api/lidar_driver.hpp```
- 参数定义: ```vanjee_driver/src/vanjee_driver/driver/driver_param.hpp```
- 错误码定义: ```vanjee_driver/src/vanjee_driver/common/error_code.hpp```

