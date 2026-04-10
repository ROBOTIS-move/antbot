# 3 Online LiDAR - Advanced Topics



## 3.1 Introduction

+ The VanJee LiDAR can operate in the following scenarios:
  - Unicast/Multicast/Broadcast modes.
  - Connecting to multiple LiDARs.

This document describes how to configure the `vanjee_lidar_sdk` in these scenarios.

Before reading this document, please ensure that you have read:
+ LiDAR product Manual
+ [Parameter Introduction](../intro/02_parameter_intro_CN.md) 
+ [How to decode online lidar](./02_how_to_decode_online_lidar_CN.md)



## 3.2 Unicast, Multicast, Broadcast

### 3.2.1 Broadcast

The LiDAR sends MSOP/DIFOP packets to the host computer.
+ The LiDAR sends packets to `255.255.255.255:3001`, and the `vanjee_lidar_sdk` binds to port `3001` on the host.

Below is the configuration in `config.yaml`

```yaml
common:
  msg_source: 1                                       
  send_point_cloud_ros: true                            

lidar:
  - driver:
      lidar_type: vajee_721       
      host_msop_port: 3001             
    ros:
      ros_frame_id: vanjee_lidar                       # Coordinate system
      ros_send_point_cloud_topic: /vanjee_points721    # Data topic
```

Here are the settings for the `common` section and `lidar-ros` section. These settings will be used consistently in the examples later in this document and will not be listed again.

### 3.2.2 Unicast

To reduce network load, it is recommended for the LiDAR to use unicast mode.
+ The LiDAR sends packets to `192.168.1.102:3001`, and the `vanjee_lidar_sdk` binds to port `3001`.

Below is the configuration in `config.yaml`, which is similar to the broadcast method.

```yaml
lidar:
  - driver:
      lidar_type: vajee_721           
      host_msop_port: 3001
      lidar_msop_port: 3333
      host_address: 192.168.2.88
      lidar_address: 192.168.2.86
```

### 3.2.3 Multicast

The LiDAR can also operate in multicast mode.

+ The LiDAR sends packets to `224.1.1.1:3001` in multicast mode.
+ The `vanjee_lidar_sdk` binds to port `3001`. Additionally, it joins the multicast group `224.1.1.1` on the local network interface with IP address `192.168.1.102`.

Below is the configuration in `config.yaml`

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721        
      host_msop_port: 3001  
      group_address: 224.1.1.1
      host_address: 192.168.2.88
```



## 3.3 The situation of multiple LiDAR

### 3.3.1 Different Target Port

If you have two or more LiDAR devices, the preferred configuration is to assign them different target ports to avoid conflicts.

+ The first LiDAR sends packets to `192.168.1.102:3001`, and the first driver node configured in the `vanjee_lidar_sdk` binds to port `3001`.
+ The second LiDAR sends packets to `192.168.1.102:3001`, and the second driver node configured in the `vanjee_lidar_sdk` binds to port `3001`.

Below is the configuration in `config.yaml`:

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721          
      host_msop_port: 3001
      lidar_msop_prot: 3333
      host_address: 192.168.2.88
      lidar_address: 192.168.2.85  
  - driver:
      lidar_type: vanjee_721      
      host_msop_port: 3002
      lidar_msop_prot: 3333
      host_address: 192.168.2.88
      lidar_address: 192.168.2.86  
```

