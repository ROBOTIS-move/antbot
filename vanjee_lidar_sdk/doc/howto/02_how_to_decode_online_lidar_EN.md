# 2 How to decode online LiDAR



## 2.1 Introduction

This document describes how to connect an online LiDAR and send point cloud data to ROS.

Before reading this document, please ensure that you have already read the LiDAR user manual and the [Parameter Introduction](../intro/02_parameter_intro_CN.md) 。



## 2.2 Steps

### 2.2.1 Obtain data port number

Please refer to the LiDAR product manual to connect the LiDAR and configure your computer's IP address.

Please refer to the LiDAR user manual or use third-party tools (such as Wireshark) to obtain the target data port number for the LiDAR.

### 2.2.2 Set up parameter files

To set up the parameter file `config.yaml`

#### 2.2.2.1 common部分

```yaml
common:
  msg_source: 1 
  send_point_cloud_ros: true                          
```

Messages originate from the online LiDAR, so please set `msg_source=1`.

To send point clouds to ROS for visualization, set `send_point_cloud_ros = true`.

#### 2.2.2.2 LiDAR-driver part

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721                              # LiDAR type
      connect_type: 1                                     # Connection method 1-udp  2-tcp
      host_msop_port: 3001                                # Port number on the computer for receiving LiDAR data
      lidar_msop_port: 3333                               # Port number on the LiDAR for sending data     
      start_angle: 0               
      end_angle: 360              
      min_distance: 0.3            
      max_distance: 45           
      use_lidar_clock: false
      host_address: 192.168.2.88
      lidar_address: 192.168.2.86    
```

- Set `lidar_type` to the type of LiDAR.
- Set `connect_type` to the LiDAR network connection type.
- Set `host_msop_port` to the port number on the computer for receiving LiDAR data.
- Set `lidar_msop_port` to the port number on the LiDAR for sending data.
- Set `host_address` to the IP address on the computer for receiving LiDAR data.
- Set `lidar_address` to the IP address on the LiDAR for sending data.

#### 2.2.2.3 lidar-rospart

```yaml
    ros:
      ros_frame_id: vanjee_lidar                       # Coordinate system
      ros_send_point_cloud_topic: /vanjee_points721    # point cloud topic name
```

Set `ros_send_point_cloud_topic` as the topic for sending point clouds. 

### 2.2.3 Run

Run program.

