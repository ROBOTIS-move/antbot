# 1 **vanjee_lidar_sdk**

## 1.1 工程简介

 **vanjee_lidar_sdk** 是万集激光雷达在Ubuntu环境下的雷达驱动软件包。它包括：
 + 雷达驱动内核[vanjee_driver]， 
 + ROS拓展功能

如果希望基于ROS进行二次开发，可以使用本软件包。配合ROS自带的可视化工具rviz，可以查看点云。 

如果希望将雷达驱动集成到自己的工程，作更深一步的二次开发，请基于vanjee_driver进行开发。

### 1.1.1 支持的雷达型号

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

### 1.1.2 支持的点类型

- XYZI - x, y, z, intensity
- XYZIRT - x, y, z, intensity, ring, timestamp

## 1.2 依赖介绍

### 1.3.1 ROS 

在ROS环境下使用雷达驱动，需要安装ROS相关依赖库。
+ Ubuntu 16.04 - ROS Kinetic desktop
+ Ubuntu 18.04 - ROS Melodic desktop
+ Ubuntu 20.04 - ROS Noetic desktop

安装方法请参考 http://wiki.ros.org。

**强烈建议安装ROS desktop-full版。这个过程会自动安装一些兼容版本的依赖库，如PCL库等。这样可以避免花大量时间，去逐个安装和配置它们**。


### 1.3.2 ROS2

在ROS2环境下使用雷达驱动，需要安装ROS2相关依赖库。
+ Ubuntu 16.04 - 不支持
+ Ubuntu 18.04 - ROS2 Eloquent desktop
+ Ubuntu 20.04 - ROS2 Galactic desktop
+ Ubuntu 22.04 - ROS2 Humble desktop

安装方法请参考 https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**请不要在一台电脑上同时安装ROS和ROS2，以避免可能的版本冲突，和手工安装其他库（如Yaml）的麻烦。**

### 1.3.3 Yaml (必需)

版本号:  >= v0.5.2 

*若已安装ROS desktop-full, 可跳过*

安装方法如下:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 1.3.4 libpcap (必需)

版本号： >= v1.7.4

安装方法如下：

```sh
sudo apt-get install -y  libpcap-dev
```



## 1.4 编译、运行

可以使用三种方式编译、运行vanjee_lidar_sdk。
**在启动ROS节点之前，请先将config文件夹下的对应LiDAR型号的配置文件(如config721.yaml)重命名为config.yaml**

### 1.4.1 直接编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**ORIGINAL**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD ORIGINAL)
```

(2) 在ROS1中，直接编译、运行程序。 

请先启动**roscore**，再运行**vanjee_lidar_sdk_node**，最后运行**rviz**查看点云。

```sh
cd vanjee_lidar_sdk
mkdir build && cd build
cmake .. && make -j4
./vanjee_lidar_sdk_node
```

### 1.4.2 依赖于ROS-catkin编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**CATKIN**.


```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将vanjee_lidar_sdk工程放入*src*文件夹内。

(3) 返回工作空间目录，执行以下命令即可编译、运行。

```sh
catkin_make
source devel/setup.bash
roslaunch vanjee_lidar_sdk start.launch
```

### 1.4.3 依赖于ROS2-colcon编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**COLCON**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) 将vanjee_lidar_sdk工程目录下的*package_ros2.xml*文件重命名为*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将vanjee_lidar_sdk工程放入*src*文件夹内。

(4) 将ROS2环境下的雷达packet消息定义，将vanjee_lidar_msg工程也放在刚刚新建的*src*文件夹内，与vanjee_lidar_sdk并列。

(5) 返回工作空间目录，执行以下命令即可编译、运行。如果使用.zsh，将第二行替换为*source install/setup.zsh*。

```sh
colcon build
source install/setup.bash
ros2 launch vanjee_lidar_sdk start.py
```

不同ROS2版本start.py的格式可能不同，请使用对应版本的start.py。如ROS2 Eloquent，请使用eloquent_start.py。

## 1.5 参数介绍

vanjee_lidar_sdk的功能通过配置参数文件来实现，请仔细阅读。 

[参数介绍](doc/intro/02_parameter_intro_CN.md)





## 1.6 快速上手

以下是一些常用功能的使用指南。

[连接在线雷达数据并发送点云到ROS](doc/howto/02_how_to_decode_online_lidar_CN.md)

[解析PCAP包并发送点云到ROS](doc/howto/04_how_to_decode_pcap_file_CN.md)

[切换点类型](doc/howto/01_how_to_change_point_type_CN.md) 