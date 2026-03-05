# 5 如何使用坐标变换功能



## 5.1 简介

 ```vanjee_lidar_sdk ```支持对点云进行坐标变换，本文档展示如何作这种变换。 

在阅读本文档之前，请确保已阅读雷达用户手册。



## 5.2 依赖库

 ```vanjee_lidar_sdk ```的坐标变换基于 ```libeigen ```库，所以要先安装它。

```bash
sudo apt-get install libeigen3-dev
```



## 5.3 编译

要启用坐标变换，编译 ```vanjee_lidar_sdk ```时，需要将```ENABLE_TRANSFORM```选项设置为```ON```.

- 直接编译

  ```bash
  cmake -DENABLE_TRANSFORM=ON ..
  ```

- ROS

  ```bash
  catkin_make -DENABLE_TRANSFORM=ON
  ```

  


## 5.4 设置雷达参数

在`config.yaml`中，设置`lidar-lidar`部分的参数`x`、, `y`、 `z`、 `roll`、 `pitch` 、`yaw`。

```yaml
common:
  msg_source: 1 
  send_point_cloud_ros: true  
  send_point_cloud_proto: false                         
lidar:
  - driver:
      lidar_type: vanjee_721                        # LiDAR类型
      connect_type: 1                               # 连接方式 1-udp  2-tcp
      host_msop_port: 3001                          # 接收点云数据的主机端口号
      lidar_msop_port: 3333                         # 雷达端口号
      start_angle: 0                                # 点云起始角度
      end_angle: 360                                # 点云结束角度
      min_distance: 0.3                             # 点云最小距离
      max_distance: 45                              # 点云最大距离
      use_lidar_clocks: false                       # True: 使用雷达时间作为消息时间戳
                                                    # False: 使用电脑主机时间作为消息时间戳
      pcap_path:                                    # pcap文件绝对路径
      pcap_repeat: true                             # 是否循环播放pcap									    
      pcap_rate: 10                                 # pcap播放速度  
      config_from_file: false                       # 从配置文件内获取参数 
      angle_path_ver:                               # 垂直角度配置文件地址
      dense_points: false                           # 输出的点云中是否剔除标记为NAN的点
      ts_first_point: false                         # 点云的时间戳是否第一个点的时间 true=第一个点的时间，false-最后一个点的时间	
      publish_mode: 0                               # 回波模式 0-发布第一重，1-发布第二重；2-发布两重；
      group_address: 0.0.0.0                        # 组播地址
      host_address: 192.168.2.88                    # 接收点云数据的主机IP地址
      lidar_address: 192.168.2.86                   # 雷达IP地址
      x: 0.5                                        # x方向偏移量 m
      y: 0                                          # y方向偏移量 m
      z: 0                                          # z方向偏移量 m
      roll: 90                                      # 横滚角偏移量 °
      pitch: 0                                      # 俯仰角偏移量 °
      yaw: 0                                        # 航向角偏移量 °

```



## 5.5 运行

运行程序。
