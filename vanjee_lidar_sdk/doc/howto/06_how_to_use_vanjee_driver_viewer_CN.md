# 6 如何可视化点云


## 6.1 概述

vanjee_driver_viewer 是 vanjee_driver 自带的小工具，可以用于显示点云。

本文说明如何使用这个工具。


## 6.2 编译和运行

要编译 vanjee_driver_viewer，需要使能编译选项 COMPILE_TOOLS=ON。

```bash
cmake -DCOMPILE_TOOS=ON ..
```

运行vanjee_driver_viewer。

```bash
./tool/rs_driver_viewer 
```


## 6.2.1 帮助菜单

-h

打印帮助菜单

-x

坐标转换参数，默认值为0，单位:米

-y

坐标转换参数，默认值为0，单位:米

-z

坐标转换参数，默认值为0，单位:米

-roll

坐标转换参数，默认值为0，单位:度

-pitch

坐标转换参数，默认值为0，单位:度

-yaw

坐标转换参数，默认值为0，单位:度

注意：要使用坐标转换功能，需要使能编译选项 ENABLE_TRANSFORM=ON.


## 6.3 使用示例

从在线雷达 vanjee_720 接收 MSOP 包。坐标转换参数为x=1, y=2, z=0, roll=30, pitch=0, yaw=0。

```bash
vanjee_driver_viewer -x 1 -y 2 -roll 30
```
