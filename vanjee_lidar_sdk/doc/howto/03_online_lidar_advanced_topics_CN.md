# 3 在线雷达 - 高级主题



## 3.1 简介

万集激光雷达可以工作在如下的场景。

+ 单播/组播/广播模式。
+ 接入多个雷达。

本文描述在这些场景下如何配置vanjee_lidar_sdk。

在阅读本文档之前， 请确保已经阅读过：
+ 雷达用户手册 
+ [参数介绍](../intro/02_parameter_intro_CN.md) 
+ [连接在线雷达](./02_how_to_decode_online_lidar_CN.md)



## 3.2 单播、组播、广播

### 3.2.1 广播

雷达发送 MSOP/DIFOP Packet到电脑主机
+ 雷达发送Packet到 `255.255.255.255` : `3001`, vanjee_lidar_sdk绑定到主机的端口 `3001`.

如下是配置`config.yaml`的方式。

```yaml
common:
  msg_source: 1                                       
  send_point_cloud_ros: true                            

lidar:
  - driver:
      lidar_type: vajee_721       
      host_msop_port: 3001             
    ros:
      ros_frame_id: vanjee_lidar                       # 坐标系
      ros_send_point_cloud_topic: /vanjee_points721    # 点云话题名 
```

这里列出了`common`部分和`lidar-ros`部分的设置。这两部分设置将在本文中后面的例子沿用，不再列出。

### 3.2.2 单播

为了减少网络负载，建议雷达使用单播模式。
+ 雷达发送Packet到 `192.168.2.88` : `3001`, vanjee_lidar_sdk绑定端口 `3001`。

如下是配置`config.yaml`的方式。这实际上与广播的方式一样。

```yaml
lidar:
  - driver:
      lidar_type: vajee_721           
      host_msop_port: 3001
      lidar_msop_port: 3333
      host_address: 192.168.2.88
      lidar_address: 192.168.2.86
```

### 3.2.3 组播

雷达也可以工作在组播模式。
+ 雷达发送Packet到 `224.1.1.1`:`3001` 
+ vanjee_lidar_sdk绑定到端口 `3001`。同时它将IP地址为`192.168.2.88`的本地网络接口加入组播组`224.1.1.1`。

如下是配置`config.yaml`的方式。

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721        
      host_msop_port: 3001  
      group_address: 224.1.1.1
      host_address: 192.168.2.88
```



## 3.3 多个雷达的情况

### 3.3.1 不同的目标端口

如果有两个或多个雷达，首选的配置是让它们有不同的目标端口。
+ 第一个雷达发送Packet到 `192.168.2.88`:`3001`, 给vanjee_lidar_sdk配置的第一个driver节点绑定到`3001`。
+ 第二个雷达发送Packet到 `192.168.2.88`:`3002`, 给vanjee_lidar_sdk配置的第二个driver节点绑定到`3002`。

如下是配置`config.yaml`的方式。

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

