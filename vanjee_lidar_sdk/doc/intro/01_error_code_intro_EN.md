
# 1 Instructions

# 1.1 Normal Instructions

```Info_PcapRepeat```                               PCAP file playback completion and replay indicator.

```Info_PcapExit```                                  Exit after PCAP file playback completion.

```failed to create udp socket, retrying...```      Socket creation failed, waiting to recreate.

```failed to reconnect tcp server,retrying...```    TCP reconnection failed, waiting to reconnect.

```succeeded to connect tcp server!```              TCP reconnection successful.

# 1.2 Abnormal situation Instructions

# 1.2.1 Need to investigate related issues

```ERRCODE_MSOPTIMEOUT```                       No point cloud data received from the LiDAR, investigation needed: network connection status, network parameter settings, whether the LiDAR is transmitting data (investigate by capturing packets with Wireshark).

```ERRCODE_NODIFOPRECV```                       If it is necessary to obtain certain parameters (such as angle table, scan frequency, etc.) from the LiDAR before parsing (displaying) point clouds, but the LiDAR does not return the corresponding parameters to the ROS driver/SDK, this prompt will appear; use Wireshark to capture packets and check if the ROS driver/SDK has sent instructions to the LiDAR to obtain the corresponding parameters, or if the LiDAR has returned the corresponding parameters to the ROS driver/SDK; otherwise, set wait_for_difop in the config file to false.

```ERRCODE_WRONGMSOPLEN```                      Abnormal point cloud data packet length received, investigation required: whether the LiDAR point cloud protocol has changed, whether the LiDAR mode is set to scanning mode.

```failed to create udp socket, timeout!```     Socket creation timeout, investigation needed: whether the server network card initialization is successful, whether the value of the "host_address" parameter in the config file matches the computer's IP address, and whether the port is occupied.

# 1.2.2 Abnormal situation Notification

```ERRCODE_ZEROPOINTS```            The number of point clouds in the currently published circle is 0, possible reasons: the current packet data is abnormal (the LiDAR has just started and has not completed initialization).

```ERRCODE_PKTBUFOVERFLOW```        The data cache size is excessively large and has been cleared, possible reason: a momentary increase in data volume due to network congestion.

```ERRCODE_CLOUDOVERFLOW```         The number of point clouds exceeds the limit, possible reasons: network anomaly, point cloud data anomaly.

```ERRCODE_WRONGIMUPACKET```        Point cloud IMU data is abnormal, complete IMU data not parsed.

```ERRCODE_STARTBEFOREINIT```      Network initialization exception, initialization unsuccessful.

```ERRCODE_PCAPWRONGPATH```        PCAP path exception, investigation needed: whether the PCAP path setting is correct.

```ERRCODE_POINTCLOUDNULL```        Point cloud data is empty.

```ERRCODE_IMUPACKETNULL```          IMU data is empty.

```ERRCODE_LASERSCANPACKETNULL```  LASERSCAN data is empty.
