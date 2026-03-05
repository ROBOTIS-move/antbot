# 2 Parameter Introduction

The `vanjee_lidar_sdk` reads the configuration file `config.yaml` to obtain all the parameters. The `config.yaml` file is located in the `vanjee_lidar_sdk/config` folder.

**config.yaml follows the YAML format. This format has strict requirements for indentation. After modifying config.yaml, please ensure that the indentation at the beginning of each line remains consistent!**

`config.yaml` consists of two sections: the `common` section and the `lidar` section.

The `vanjee_lidar_sdk` supports multiple LiDAR devices. The `common` section contains parameters shared among all LiDAR devices. The `lidar` section defines settings for each individual LiDAR device, with each sub-node corresponding to a specific LiDAR.



## 2.1 Common Part

The `common` section sets the source (where packets or point clouds come from) and destination (where packets or point clouds are published to) for LiDAR messages.

```yaml
common:
  msg_source: 1                                   # 1: The message originates from an online LiDAR
                                                  # 2: The message originates from ROS
                                                  # 3: The message originates from PCAP
  send_point_cloud_ros: true                      # true: Send point clouds to ROS for visualization
  send_imu_packet_ros: true                       # true: Send IMU to ROS for visualization
  send_laser_scan_ros: true                       # true: Send laser scan to ROS for visualization, only supports single line lidar
  send_device_ctrl_state_ros: true                # true: Send interactive data of device status settings to ROS for viewing
  recv_device_ctrl_cmd_ros: true                  # true: Topic on subscribing to device status settings
```

- ```msg_source```

  - 1 -- Connect to an online LiDAR. For more details on usage, please refer to [how to decode online lidar](../howto/02_how_to_decode_online_lidar_CN.md)。

  - 2-- Offline parsing of PCAP packets. For more usage details, please refer to[How to decode pcap file](../howto/04_how_to_decode_pcap_file_CN.md)。
   
- ```send_point_cloud_ros```

  - true -- The LiDAR point cloud messages will be published via ROS.

  The type of point cloud messages is defined by the official ROS sensor_msgs/PointCloud2 message type. Users can directly view the point cloud using Rviz. While users can record ROS point cloud messages, it's not recommended due to their large size. A better approach is to record packet data. Please refer to the case where `send_packet_ros=true` for more details.

- ```send_imu_packet_ros```

  - true -- The LiDAR imu messages will be published via ROS.

  The type of point cloud message is sensor_msgs/IMU, which is officially defined by ROS. Users can directly view point clouds using Rviz

 - ```send_laser_scan_ros```

  - true -- The LiDAR packet messages will be published via ROS.

    *Only supports single-line LiDAR

- ```send_device_ctrl_state_ros```

  - true -- The LiDAR status messages will be published via ROS.

  *Custom topic, please refer to details "03_device_ctrl_intro_EN.md"。

 - ```recv_device_ctrl_cmd_ros```

  - true -- subscribe to the LiDAR control messages.

  *Custom topic, please refer to details "03_device_ctrl_intro_EN.md"。



## 2.2 LiDAR part

The `lidar` section is configured based on the specific situation of each LiDAR device.

```yaml
lidar:
  - driver:
      lidar_type: vanjee_720_16                     # LiDAR Model
      connect_type: 1                               # Connection type: 1 - UDP, 2 - TCP
      host_msop_port: 3001                          # Host port number for receiving point cloud data
      lidar_msop_port: 3333                         # LiDAR port number
      start_angle: 0                                # Starting angle of the point cloud
      end_angle: 360                                # Ending angle of the point cloud
      wait_for_difop: true                          # Wait for the angle calibration parameters to be imported
      min_distance: 0.5                             # Minimum distance of the point cloud
      max_distance: 120                             # Maximum distance of the point cloud
      hide_points_range:                            # Hide point range
      use_lidar_clock: true                         # true: Use LiDAR time as the message timestamp
                                                    # false: Use host computer time as the message timestamp
      pcap_path:                                    # Absolute path to the PCAP file
      pcap_repeat: true                             # Whether to loop playback of the PCAP file
      pcap_rate: 10                                 # Playback speed of the PCAP file
      config_from_file: false                       # Whether to get parameters from the configuration file
      angle_path_ver:                               # Path to the vertical angle configuration file
      angle_path_hor:                               # Path to the horizontal angle configuration file
      imu_param_path:                               # Path to the IMU parameters configuration file
      dense_points: false                           # Whether to exclude points marked as NAN from the output point cloud
      ts_first_point: false                         # Whether the timestamp of the point cloud is the time of the first point
                                                    # true - time of the first point, false - time of the last point
      use_offset_timestamp: true                    # Use relative timestamp 
                                                    # true - each point in the point cloud uses the time difference relative to the topic
                                                    # false - each point uses UTC time
      publish_mode: 0                               # Echo mode: 0 - publish first return, 1 - publish second return, 2 - publish both returns
      group_address: 0.0.0.0                        # Multicast address
      host_address: 192.168.2.88                    # Host IP address for receiving point cloud data
      lidar_address: 192.168.2.86                   # LiDAR IP address
      x: 0                                          # X-axis offset
      y: 0                                          # Y-axis offset
      z: 0                                          # Z-axis offset
      roll: 0                                       # Roll angle offset
      pitch: 0                                      # Pitch angle offset
      yaw: 0                                        # Yaw angle offset
      use_vlan: false                               # Whether packets in the PCAP file contain VLAN layers

    ros:
      ros_frame_id: vanjee_lidar                            # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points720_16      # Topic used to send point cloud through ROS
      ros_send_imu_packet_topic: /vanjee_lidar_imu_packets  # Topic used to send imu through ROS
   
```

- ```lidar_type```

  The supported LiDAR models are listed in the `README` file of the `vanjee_lidar_sdk`.

- ```host_msop_port```

  The port number for receiving scan data packets. *If no messages are received, please check if this parameter is configured correctly first.*

   ```lidar_msop_port```

  The port number for sending scan data packets, which is also the port where the LiDAR receives query commands. *If no messages are received, please check if this parameter is configured correctly first.*

- ```start_angle```, ```end_angle```

  The starting and ending angles of the point cloud message. This setting is software masking, setting points outside the region to 0. The range for start_angle and end_angle is 0~360°, **the starting angle can be greater than the ending angle**.

- ```wait_for_difop```
  When connecting to an online LiDAR, if set to true, point cloud data will not be parsed until the LiDAR's angle calibration table is received. When reading a PCAP file, this parameter defaults to false. **For models 719/719c, which do not have an angle calibration table, the default is false.**

- ```min_distance```, ```max_distance```

  The minimum and maximum distances of the point cloud. This setting is used to filter points outside a specified distance range by setting them to NaN (Not a Number) points, without reducing the size of each point cloud frame.

- ```hide_points_range``` --masking range,
                          line id range: C1-C2，angle range: A1-A2，distance range: L1-L2；(Range is set in ascending order)
                          Group 1：1-3,0-0.5/100-105,0-1.5/10-20;(masking line id: 1 2 3，masking angle: 0-0.5°+100-105°，masking distance: 0-1.5m+10-20m);
                          Group 2：5,0/90/180/270,10;(masking line id: 5，masking angle: 0+90+180+270°，masking distance: 10m);
                          Group 3：1-3,0-0.5/100-105,0-1.5/10-20;5,0/90/180/270,10;(masking range = Group 1 + Group 2);
                          
                          masking Rules,
                          Separate group elements with commas (,), separate line id, angles, and distances with slashes (/) when they are not continuous, and separate two groups with semicolons (;), all using English characters as separators;

                          Note: The angle/distance ranges of Shielding Group 1 and Shielding Group 2 are independent of each other, as shown below. Shielding Group 4 and Shielding Group 5 have different ranges, while Shielding Group 4 and Shielding Group 6 have equal ranges;
                          
                          Group 4：5,10-20/50-60,1.5-2.5/11-13;
                          Group 5：5,10-20,1.5-2.5;5,50-60,11-13;
                          Group 6：5,10-20,1.5-2.5;5,10-20,11-13;5,50-60,1.5-2.5;5,50-60,11-13;

- ```use_lidar_clock```

  - true -- Use LiDAR time as the message timestamp.
  - false -- Use the computer host time as the message timestamp.

- ```dense_points ```

  - Whether to remove NaN points from the output point cloud. Default value is false.
    - true means to remove NaN points.
    - false means not to remove NaN points.

- ```pcap_path```

   Path to the PCAP file. This is valid when `msg_source=3`.

- ```pcap_repeat``` -- The default value is true. Users can set it to false to disable the PCAP loop playback feature.

- ```pcap_rate``` -- The default value is 1, which corresponds to a point cloud frequency of approximately 10 Hz. Users can adjust this parameter to control the playback speed of the PCAP. A larger value will increase the playback speed.

- ```config_from_file``` -- The default value is false. This parameter determines whether to read the LiDAR configuration information from an external parameter file. 

- ```angle_path_ver``` -- The path of the angle configuration file defines and saves the vertical angles for each channel of the LiDAR (vertical angles and initial horizontal angles). The file path is: /src/vanjee_lidar_sdk/param

- ```angle_path_hor``` -- The path of the angle configuration file defines and saves the horizontal angles for each channel of the LiDAR. The file path is: /src/vanjee_lidar_sdk/param

- ```imu_param_path``` -- The IMU parameter configuration file path defines and saves the IMU configuration parameters for the LiDAR. The file path is: /src/vanjee_lidar_sdk/param

- ```ts_first_point``` -- The default value is false. This parameter determines whether the timestamp of the point cloud corresponds to the time of the first point (`true`) or the time of the last point (`false`).

- ```use_offset_timestamp``` -- The default value is true. Is the timestamp of the point cloud used relative to the time difference at the beginning of the question. Each point in the point cloud uses the time difference relative to the topic(`true`), each point uses UTC time (`false`)。

- ```publish_mode``` -- Echo mode: 0 - publish the first echo, 1 - publish the second echo; 2 - publish both echoes.

- ```group_address``` -- If the LiDAR is in multicast mode, this parameter should be set to the multicast address. For specific usage, refer to[Online LiDAR-advanced topics](../howto/03_online_lidar_advanced_topics_CN.md) 。

- ```host_address``` -- If the host is receiving data from multiple LiDARs through multiple IP addresses, you can specify this parameter as the target IP address of the LiDAR.；If `group_address` is set, then `host_address` must also be set to add the network card of this IP address to the multicast group.

- ```lidar_address``` -- The IP address from which the LiDAR sends data.

- ```x, y, z, roll, pitch, yaw ``` -- are parameters for coordinate transformation. If the kernel's coordinate transformation feature is enabled, these parameters will be used to output the transformed point cloud. The units for `x, y, z` are meters, and the units for `roll, pitch, yaw` are radians. For detailed usage, refer to [how to sue coordinate transformation](../howto/05_how_to_use_coordinate_transformation_CN.md) 。

- ```use_vlan``` -- If the packets in the PCAP file include VLAN layers, you can specify `use_vlan` as `true` to skip this layer.

## 2.3 Example

### 2.3.1 Single LiDAR

To connect one WLR-733 LiDAR online and send the point cloud to ROS.

```yaml
common:
  msg_source: 1                                   # 0: not use lidar 
                                                  # 1: The message is sourced from an online LiDAR.
                                                  # 2: The message is sourced from pcap
  send_point_cloud_ros: true                      # true: Send point clouds to ROS for visualization  
  send_imu_packet_ros: true                       # true: Send IMU to ROS for visualization  
lidar:
  - driver:
      lidar_type: vanjee_720_16                     # LiDAR model
      connect_type: 1                               # Connection type 1-udp 2-tcp
      host_msop_port: 3001                          # Host port number for receiving point cloud data
      lidar_msop_port: 3333                         # Lidar port number
      start_angle: 0                                # Starting angle of the point cloud
      end_angle: 360                                # Ending angle of the point cloud
      wait_for_difop: true                          # Whether to wait for angle calibration table parameters to be imported
      min_distance: 0.5                             # Minimum distance of the point cloud
      max_distance: 120                             # Maximum distance of the point cloud
      hide_points_range:                            # Hide point range
      use_lidar_clock: true                         # true: Use Lidar time as message timestamp
                                                    # false: Use computer host time as message timestamp
      pcap_path:                                    # Absolute path of the pcap file
      pcap_repeat: true                             # Whether to loop play the pcap
      pcap_rate: 10                                 # Pcap playback speed
      config_from_file: false                       # Get parameters from the configuration file
      angle_path_ver:                               # Vertical angle configuration file address
      angle_path_hor:                               # Horizontal angle configuration file address
      imu_param_path:                               # IMU parameter configuration file address
      dense_points: false                           # Whether to remove points marked as NAN in the output point cloud
      ts_first_point: false                         # Whether the timestamp of the point cloud is the time of the first point
                                                    # true - time of the first point, false - time of the last point
      use_offset_timestamp: true                    # Use relative timestamp 
                                                    # true - each point in the point cloud uses the time difference relative to the topic
                                                    # false - each point uses UTC time
      publish_mode: 0                               # Echo mode 0-publish the first echo, 1-publish the second echo; 2-publish both echoes;
      group_address: 0.0.0.0                        # Multicast address
      host_address: 192.168.2.88                    # Host IP address for receiving point cloud data
      lidar_address: 192.168.2.86                   # Lidar IP address
      x: 0                                          # Offset in the x direction
      y: 0                                          # Offset in the y direction
      z: 0                                          # Offset in the z direction
      roll: 0                                       # Roll offset
      pitch: 0                                      # Pitch offset
      yaw: 0                                        # Yaw offset
      use_vlan: false                                 # Whether the Packet in the PCAP file contains VLAN layer

    ros:
      ros_frame_id: vanjee_lidar                            # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points720_16      # Topic used to send point cloud through ROS
      ros_send_imu_packet_topic: /vanjee_lidar_imu_packets  # Topic used to send imu through ROS 

     
```

### 2.3.2 Two LiDARs

Connect one "WLR-719" LiDAR and one "WLR-720" LiDAR online, and send point clouds to ROS.

*Note the indentation of parameters in the `lidar` section.*

```yaml
common:
  msg_source: 1                                   # 0: not use lidar 
                                                  # 1: Message source is online LiDAR
                                                  # 2: Message source is pcap
  send_point_cloud_ros: true                      # true: Send point cloud to ROS for viewing
  send_imu_packet_ros: true                       # true: Send IMU data to ROS for viewing
  send_laser_scan_ros: false                      # true: Send polar coordinate point cloud topic to ROS for viewing, supports only single-line LiDAR
lidar:
  - driver:
      lidar_type: vanjee_719                        # LiDAR type
      connect_type: 1                               # Connection type 1-udp  2-tcp
      host_msop_port: 3002                          # Host port number for receiving point cloud data
      lidar_msop_port: 6050                         # LiDAR port number
      start_angle: 0                                # Starting angle of the point cloud
      end_angle: 360                                # Ending angle of the point cloud
      wait_for_difop: false                         # Whether to wait for the angle calibration table parameters to be imported
      min_distance: 0.2                             # Minimum distance of the point cloud
      max_distance: 50                              # Maximum distance of the point cloud
      use_lidar_clock: true                         # true: Use LiDAR time as message timestamp
                                                    # false: Use computer host time as message timestamp
      pcap_path:                                    # Absolute path to pcap file
      pcap_repeat: true                             # Whether to loop the pcap file playback
      pcap_rate: 10                                 # Playback speed of pcap file
      config_from_file: false                       # Get parameters from the configuration file
      angle_path_ver:                               # Vertical angle configuration file address
      angle_path_hor:                               # Horizontal angle configuration file address
      dense_points: false                           # Whether to remove points marked as NAN in the output point cloud
      ts_first_point: false                         # Whether the timestamp of the point cloud is the time of the first point
                                                    # true - time of the first point, false - time of the last point
      use_offset_timestamp: true                    # Use relative timestamp 
                                                    # true - each point in the point cloud uses the time difference relative to the topic
                                                    # false - each point uses UTC time
      publish_mode: 0                               # Echo mode 0-publish the first echo, 1-publish the second echo; 2-publish both echoes;
      group_address: 0.0.0.0                        # Multicast address
      host_address: 192.168.0.110                   # Host IP address for receiving point cloud data
      lidar_address: 192.168.0.2                    # LiDAR IP address
      x: 0                                          # x-axis offset m
      y: 0                                          # y-axis offset m
      z: 0                                          # z-axis offset m
      roll: 0                                       # Roll offset °
      pitch: 0                                      # Pitch offset °
      yaw: 0                                        # Yaw offset °
      use_vlan: false                               # Whether the packets in the PCAP file contain VLAN layer
    ros:
      ros_frame_id: vanjee_lidar                       # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points719    # Topic used to send point cloud through ROS
      ros_send_laser_scan_topic: /vanjee_scan719       # Topic used to send laser scan through ROS
  - driver:
      lidar_type: vanjee_720_16                     # LiDAR type
      connect_type: 1                               # Connection type 1-udp 2-tcp
      host_msop_port: 3001                          # Host port to receive point cloud data
      lidar_msop_port: 3333                         # LiDAR port
      start_angle: 0                                # Starting angle of the point cloud
      end_angle: 360                                # Ending angle of the point cloud
      wait_for_difop: true                          # Whether to wait for the angle calibration table parameters
      min_distance: 0.5                             # Minimum distance of the point cloud
      max_distance: 120                             # Maximum distance of the point cloud
      hide_points_range:                            # Hide point range
      use_lidar_clock: true                         # true: Use LiDAR time as message timestamp
                                                    # false: Use host computer time as message timestamp
      pcap_path:                                    # Absolute path of the pcap file
      pcap_repeat: true                             # Whether to loop the pcap file playback
      pcap_rate: 10                                 # Playback speed of the pcap file
      config_from_file: false                       # Get parameters from the configuration file
      angle_path_ver:                               # Vertical angle configuration file path
      angle_path_hor:                               # Horizontal angle configuration file path
      imu_param_path:                               # IMU parameter configuration file path
      dense_points: false                           # Whether to remove points marked as NAN in the output point cloud
      ts_first_point: false                         # Whether the timestamp of the point cloud is the time of the first point
                                                    # true - time of the first point, false - time of the last point
      use_offset_timestamp: true                    # Use relative timestamp 
                                                    # true - each point in the point cloud uses the time difference relative to the topic
                                                    # false - each point uses UTC time
      publish_mode: 0                               # Echo mode: 0 - publish the first echo, 1 - publish the second echo, 2 - publish both echoes
      group_address: 0.0.0.0                        # Multicast address
      host_address: 192.168.2.88                    # Host IP address to receive point cloud data
      lidar_address: 192.168.2.86                   # LiDAR IP address
      x: 0                                          # Offset in the x direction (meters)
      y: 0                                          # Offset in the y direction (meters)
      z: 0                                          # Offset in the z direction (meters)
      roll: 0                                       # Roll angle offset (degrees)
      pitch: 0                                      # Pitch angle offset (degrees)
      yaw: 0                                        # Yaw angle offset (degrees)
      use_vlan: false                               # Whether PCAP files contain VLAN layer in the Packet

    ros:
      ros_frame_id: vanjee_lidar                                # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points720_16          # Topic used to send point cloud through ROS
      ros_send_imu_packet_topic: /vanjee_lidar_imu_packets_720  # Topic used to send imu through ROS 
```

