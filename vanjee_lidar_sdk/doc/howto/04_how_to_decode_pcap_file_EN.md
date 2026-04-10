# 4 How to decode PCAP file



## 4.1 Introduction

This document demonstrates how to decode PCAP files and send point cloud data to ROS.

Before reading this document, please ensure that you have read the LiDAR user manual and the [Parameter Introduction](../intro/02_parameter_intro_CN.md) .



## 4.2 Steps

### 4.2.1 Obtain data port number

Please refer to the LiDAR product manual or use third-party tools (such as Wireshark) to capture packets and determine the target port for the LiDAR. The default port value is `3001`.

### 4.2.2 Set up parameter file

Set up parameter file```config.yaml```。

#### 4.2.2.1 Common part

```yaml
common:
  msg_source: 2                                   # 1: Messages originate from the online LiDAR.
                                                  # 2: messages originate from pcap
  send_point_cloud_ros: true                      # true: Send point clouds to ROS for visualization 
```

Messages originate from PCAP packets, so set ```msg_source = 2``` 。

To send point clouds to ROS for visualization, set  ```send_point_cloud_ros = true``` 。 

#### 4.2.2.2 LiDAR-driver part

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721                        # LiDAR model
      connect_type: 1                               # Connection method 1-udp  2-tcp
      host_msop_port: 3001                          # The port number on the host for receiving point cloud data
      lidar_msop_port: 3333                         #start_angle: 0 # Starting angle of the point cloud
end_angle: 360                                      # Ending angle of the point cloud
min_distance: 0.3                                   # Minimum distance of the point cloud
max_distance: 45                                    # Maximum distance of the point cloud
use_lidar_clocks: false                             # True: Use LiDAR time as message timestamp
                                                    # False: Use computer host time as message timestamp
pcap_path:                                          # Absolute path to the pcap file
pcap_repeat: true                                   # Whether to loop playback of the pcap
pcap_rate: 1                                        # Playback speed of the pcap
config_from_file: true                              # Get parameters from the configuration file
angle_path_ver: /src/vanjee_lidar_sdk/param/Vanjee_721_64_carside.csv       # Vertical angle configuration file path
angle_path_hor:                                     # Horizontal angle configuration file path
imu_param_path:                                     # IMU parameter configuration file path"
```

- Set `lidar_type` to the LiDAR type.
- Set `host_msop_port` to the target port number for LiDAR data, which is `3001`.
- Set `pcap_path` to the full path of the PCAP file.
- Set `pcap_repeat` to whether to loop playback of the PCAP file, which is `true`.
- Set `pcap_rate` to the playback speed of the PCAP file; the higher the value, the faster the speed. Here it is `1`.
- Set `angle_path_ver`, `angle_path_hor`, and `imu_param_path` to the LiDAR parameter configuration files. If configuration is needed, set the respective file paths. For example, for WLR-721, configure the vertical angle table `angle_path_ver`.
- Set `config_from_file` to whether to read the local configuration file. For WLR-721, since the vertical angle table needs to be configured, set it to `true`.

#### 4.2.2.3 lidar-ros part

```yaml
    ros:
      ros_frame_id: vanjee_lidar                       # Coordinate System
      ros_send_point_cloud_topic: /vanjee_points721    # Point cloud topic name
```

Set `ros_send_point_cloud_topic` to the topic for sending point clouds, which is `/vanjee_points733`

### 4.2.3 Run

Run program
