# 5 How to use coordinate transformation



## 5.1 Introduction

`vanjee_lidar_sdk` supports coordinate transformation of point clouds. This document demonstrates how to perform this transformation.

Before reading this document, please ensure that you have read the LiDAR product manual.



## 5.2 Dependent Libraries

The coordinate transformation in `vanjee_lidar_sdk` is based on the `libeigen` library, so it needs to be installed first.

```bash
sudo apt-get install libeigen3-dev
```



## 5.3 Compile

To enable coordinate transformation, when compiling `vanjee_lidar_sdk`, the `ENABLE_TRANSFORM` option needs to be set to `ON`.

- Compile directly

  ```bash
  cmake -DENABLE_TRANSFORM=ON ..
  ```

- ROS

  ```bash
  catkin_make -DENABLE_TRANSFORM=ON
  ```

  


## 5.4 Set up LiDAR parameter

In the `config.yaml` file, set the parameters `x`, `y`, `z`, `roll`, `pitch`, and `yaw` under the `lidar-lidar` section.

```yaml
common:
  msg_source: 1 
  send_point_cloud_ros: true  
  send_point_cloud_proto: false                         
lidar:
  - driver:
     lidar_type: vanjee_721                        # LiDAR type
connect_type: 1                               # Connection method: 1-udp, 2-tcp
host_msop_port: 3001                          # Host port number for receiving point cloud data
lidar_msop_port: 3333                         # LiDAR port number
start_angle: 0                                # Starting angle of the point cloud
end_angle: 360                                # Ending angle of the point cloud
min_distance: 0.3                             # Minimum distance of the point cloud
max_distance: 45                              # Maximum distance of the point cloud
use_lidar_clocks: false                       # True: Use LiDAR time as message timestamp
                                              # False: Use computer host time as message timestamp
pcap_path:                                    # Absolute path of the pcap file
pcap_repeat: true                             # Whether to loop playback of the pcap file
pcap_rate: 10                                 # Playback speed of the pcap file
config_from_file: false                       # Whether to get parameters from the configuration file
angle_path_ver:                               # Vertical angle configuration file path
dense_points: false                           # Whether to exclude points marked as NAN in the output point cloud
ts_first_point: false                         # Whether the point cloud timestamp is the time of the first point (true) or the last point (false)
publish_mode: 0                               # Echo mode: 0=publish first echo, 1=publish second echo, 2=publish both echoes
group_address: 0.0.0.0                        # Multicast address
host_address: 192.168.2.88                    # Host IP address for receiving point cloud data
lidar_address: 192.168.2.86                   # LiDAR IP address
x: 0.5                                        # Offset in the x direction (m)
y: 0                                          # Offset in the y direction (m)
z: 0                                          # Offset in the z direction (m)
roll: 90                                      # Roll offset angle (°)
pitch: 0                                      # Pitch offset angle (°)
yaw: 0                                        # Yaw offset angle (°)
```



## 10.5 Run

Run program.
