# 3 Introduction to Device Control Parameters

**Support lidar type:**
  WLR-716mini
  WLR-718h
  WLR-719
  WLR-722

## 3.1 Device control topic information

``` 
std_msgs/Header header 
uint16 cmd_id
uint16 cmd_param
uint8 cmd_state
``` 

``` header ```
device control topic header information:
ROS1 includes: sequence number(seq), timestamp (timestamp), coordinate system (frame id);
ROS2 includes: timestamp(timestamp),coordinate system (frame id)

``` cmd_id ```
command identification
1 - set device work mode 

``` cmd_param ```
command parameters
the parameters of "set device work mode" include:
0 - working mode
1 - standby mode
2 - only open channels 25-28(only for vanjee_722)

``` cmd_state ```
command responded state
If the device responded the corresponding device control command correctly.User shall ignore this field whiling issuing the device control topics
Unless otherwise specified, the response state of all device control commands include:
0 - failure
1- success


``` device control topic header information
  uint32 seq = 0;
  float64 timestamp = 0.0;
  string frame_id = "";
```

## 3.2 Device control interface information

data structure:
class DeviceCtrl {
 public:
  uint32 seq = 0;
  float64 timestamp = 0.0;
  uint16 cmd_id = 0;
  uint16 cmd_param = 0;
  uint8 cmd_state = 0;
};

``` 
application program interface:
void deviceCtrlApi(const DeviceCtrl& device_ctrl_cmd);
``` 
device class:
LidarDriver<PointCloudMsg> driver;

## 3.2.1 Receive the responded state of device control command
Register callback function:
driver.regDeviceCtrlCallback(allocateDeviceCtrlMemoryCallback, deviceCtrlCallback);

Sdk store the responded state of device control command into the memory provided by the allocateDeviceCtrlMemoryCallback function
Receive the responded state of device control command by deviceCtrlCallback function

## 3.2.2 Send device control command
example of device control command:
DeviceCtrl deviceCtrl;
deviceCtrl.cmd_id = 1;
deviceCtrl.cmd_param = 0; // working mode; 1-standby mode; 2-only open channels 25-28(only for vanjee_722)
deviceCtrl.cmd_state = 0;

Call interface:
driver.deviceCtrlApi(deviceCtrl);

**refer the demo for the specific implementation method**

## 3.3 Device control commands for all lidars

### 3.3.1 WLR-716mini

cmd_id: 0x0100 - Device operating state (to user)
cmd_param: 0-Normal 1-Warning 2-Fault
cmd_state: 0-Normal 1-Warning 2-Fault

### 3.3.2 WLR-718h

cmd_id: 0x0100 - Device operating state (to user)
cmd_param: 0-Normal 1-Warning 2-Fault
cmd_state: 0-Normal 1-Warning 2-Fault

### 3.3.3 WLR-719

cmd_id: 0x0100 - Device operating state (to user)
cmd_param: 0-Normal 1-Warning 2-Fault
cmd_state: 0-Normal 1-Warning 2-Fault

### 3.3.4 WLR-722

cmd_id: 0x0001 - Set lidar work mode (to lidar)
cmd_param: 0-working mode; 1-standby mode; 2-only open channels 25-28
cmd_state: 0

cmd_id: 0x0001 - The feedback of setting lidar operating mode command (to user)
cmd_param: 0-working mode; 1-standby mode; 2-only open channels 25-28
cmd_state: 0-failure 1-success

