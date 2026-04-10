
# 1 提示说明

# 1.1 正常提示说明

```Info_PcapRepeat```                               PCAP文件播放完并重新播放标志。 

```Info_PcapExit```                                 PCAP文件播放完并退出。 

```failed to create udp socket, retrying...```      创建socket失败，等待重新创建。

```failed to reconnect tcp server,retrying...```    TCP重连失败，等待重连。

```succeeded to connect tcp server!```              TCP重连成功。

# 1.2 异常提示说明

# 1.2.1 需排查相关问题

```ERRCODE_MSOPTIMEOUT```                       未收到激光雷达发送的点云数据，需排查：网络连接状态、网络参数设置、雷达是否发送数据(通过wireshark抓包排查)。

```ERRCODE_NODIFOPRECV```                       如果需要在解析（显示）点云之前先从激光雷达中获取部分参数（如角度表、扫描频率等），但激光雷达未向ROS驱动/SDK返回对应参数时，会出现该提示；通过wireshark抓包检查ROS驱动/SDK 是否向激光雷达发送了获取对应参数的指令，又或者激光雷达是否向ROS驱动/SDK返回了对应参数；否则，将config文件中的 wait_for_difop 设置为 false。

```ERRCODE_WRONGMSOPLEN```                      接收点云数据包长度异常，需排查：雷达点云协议是否变动，雷达模式是否为扫描模式。

```failed to create udp socket, timeout!```     创建socket超时，需排查服务器网卡是否初始化成功、config文件中参数“host_address”的值和电脑IP地址是否一致、端口是否被占用。

# 1.2.2 异常提示

```ERRCODE_ZEROPOINTS```            当前发布的一圈点云里面点云数量为0，可能原因：当前包数据为异常数据(雷达刚启动未完成初始化)。 

```ERRCODE_PKTBUFOVERFLOW```        数据缓存量过大并清空缓存，可能原因：网络阻塞瞬时数据量剧增所致。 

```ERRCODE_CLOUDOVERFLOW```         点云数量超过上限，可能原因：网络异常、点云数据异常。 

```ERRCODE_WRONGIMUPACKET```        点云IMU数据异常，未解析到完整IMU数据。 

```ERRCODE_STARTBEFOREINIT```       网络初始化异常，未初始化成功。 

```ERRCODE_PCAPWRONGPATH```         PCAP路径异常，需排查：PCAP路径设置是否正确。

```ERRCODE_POINTCLOUDNULL```        点云数据为空。 

```ERRCODE_IMUPACKETNULL```         IMU数据为空。 

```ERRCODE_LASERSCANPACKETNULL```   LASERSCAN数据为空。 
