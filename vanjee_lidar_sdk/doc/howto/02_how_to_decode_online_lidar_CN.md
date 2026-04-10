# 2 如何连接在线雷达



## 2.1 简介

本文档描述如何连接在线雷达，并发送点云数据到ROS。

在阅读本文档之前， 请确保已经阅读过雷达用户手册和[参数简介](../intro/02_parameter_intro_CN.md) 。



## 2.2 步骤

### 2.2.1 获取数据端口号

根据雷达用户手册连接雷达, 并设置好您的电脑的IP地址。

请参考雷达用户手册，或使用第三方工具（如WireShark等）得到雷达数据目标端口号。

### 2.2.2 设置参数文件

设置参数文件```config.yaml```。

#### 2.2.2.1 common部分

```yaml
common:
  msg_source: 1 
  send_point_cloud_ros: true                          
```

消息来源于在线雷达，因此请设置```msg_source=1```。

将点云发送到ROS以便查看，因此设置 ```send_point_cloud_ros = true``` 。

#### 6.2.2.2 lidar-driver部分

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721                              # LiDAR类型
      connect_type: 1                                     # 连接方式 1-udp  2-tcp
      host_msop_port: 3001                                # 电脑端接收LiDAR数据的端口号
      lidar_msop_port: 3333                               # LiDAR端发送数据的端口号      
      start_angle: 0               
      end_angle: 360              
      min_distance: 0.3            
      max_distance: 45           
      use_lidar_clock: false
      host_address: 192.168.2.88
      lidar_address: 192.168.2.86    
```

将 ```lidar_type``` 设置为LiDAR类型 。
将 ```connect_type``` 设置为LiDAR网络连接类型 。
设置 ```host_msop_port``` 电脑端接收LiDAR数据的端口号。
设置 ```lidar_msop_port```LiDAR端发送数据的端口号。
设置 ```host_address```   电脑端接收LiDAR数据的IP地址。
设置 ```lidar_address```   LiDAR端发送数据的IP地址。

#### 2.2.2.3 lidar-ros部分

```yaml
    ros:
      ros_frame_id: vanjee_lidar                       # 坐标系
      ros_send_point_cloud_topic: /vanjee_points721    # 点云话题名
```

将 ```ros_send_point_cloud_topic``` 设置为发送点云的话题。 

### 2.2.3 运行

运行程序。

