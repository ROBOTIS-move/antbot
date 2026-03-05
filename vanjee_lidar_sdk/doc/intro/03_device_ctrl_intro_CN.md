# 3 设备控制参数介绍

**支持雷达型号:**
  WLR-716mini
  WLR-718h
  WLR-719
  WLR-722

## 3.1 设备控制话题信息

``` 
std_msgs/Header header 
uint16 cmd_id
uint16 cmd_param
uint8 cmd_state
``` 

``` header ```
设备控制话题头信息:
ROS1包含内容有:序号(seq) 时间戳(timestamp) 坐标系(frame id);
ROS2包含内容有:时间戳(timestamp) 坐标系(frame id).

``` cmd_id ```
设备操作码:
1 - 设备运行模式设置;

``` cmd_param ```
"设置设备运行模式"的操作参数包括:
0 - 工作模式
1 - 待机模式
2 - 只开启25-28通道(目前只对vanjee_722生效)

``` cmd_state ```
设备操作状态:
返回设备对控制指令的响应状态.用户发布设备控制话题时，该字段不用赋值
如不作特殊说明，所有设备控制指令的响应状态包括：
0 - 失败;
1 - 成功;

``` 设备操作话题头信息
  uint32 seq = 0;
  float64 timestamp = 0.0;
  string frame_id = "";
```

## 3.2 设备控制接口信息

数据结构:
class DeviceCtrl {
 public:
  uint32 seq = 0;
  float64 timestamp = 0.0;
  uint16 cmd_id = 0;
  uint16 cmd_param = 0;
  uint8 cmd_state = 0;
};

``` 
接口函数:
void deviceCtrlApi(const DeviceCtrl& device_ctrl_cmd);
``` 
设备类:
LidarDriver<PointCloudMsg> driver;

## 3.2.1 接收设备控制指令的响应状态
回调函数注册:
driver.regDeviceCtrlCallback(allocateDeviceCtrlMemoryCallback, deviceCtrlCallback);
sdk通过allocateDeviceCtrlMemoryCallback提供的内存空间存储设备控制指令的响应状态
通过deviceCtrlCallback接收设备控制指令的响应状态

## 3.2.2 发送设备控制指令
设备控制指令示例:
DeviceCtrl deviceCtrl;
deviceCtrl.cmd_id = 1;
deviceCtrl.cmd_param = 0; // 0-工作模式; 1-待机模式; 2-只开启25-28通道(目前只对vanjee_722生效)
deviceCtrl.cmd_state = 0;

接口调用:
driver.deviceCtrlApi(deviceCtrl);

**具体实现方法可参考demo用例**

## 3.3 各型号雷达设备控制指令说明

### 3.3.1 WLR-716mini

cmd_id: 0x0100 - 设备运行状态上报(上行)
cmd_param: 0-正常 1-警告 2-故障
cmd_state: 0-正常 1-警告 2-故障

### 3.3.2 WLR-718h

cmd_id: 0x0100 - 设备运行状态上报(上行)
cmd_param: 0-正常 1-警告 2-故障
cmd_state: 0-正常 1-警告 2-故障

### 3.3.3 WLR-719

cmd_id: 0x0100 - 设备运行状态上报(上行)
cmd_param: 0-正常 1-警告 2-故障
cmd_state: 0-正常 1-警告 2-故障

### 3.3.4 WLR-722

cmd_id: 0x0001 - 设备控制指令设置(下行)
cmd_param: 0-工作模式 1-待机模式 2-只开启25-28通道
cmd_state: 0

cmd_id: 0x0001 - 设备控制指令响应状态(上行)
cmd_param: 0-工作模式 1-待机模式 2-只开启25-28通道
cmd_state: 0-设置失败 1-设置成功
