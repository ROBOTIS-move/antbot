# 6 How to Visualize Point Clouds

## 6.1 Overview

`vanjee_driver_viewer` is a small tool included with `vanjee_driver` that can be used to display point clouds.

This document explains how to use this tool

## 6.2 Compilation and Execution

To compile `vanjee_driver_viewer`, you need to enable the compilation option `COMPILE_TOOLS=ON`.

```bash
cmake -DCOMPILE_TOOS=ON ..
```

Run vanjee_driver_viewer。

```bash
./tool/rs_driver_viewer 
```

## 6.2.1 Help Menu

- `-h`: Print the help menu.
- `-x`: Coordinate transformation parameter, default value is 0, unit: meters.
- `-y`: Coordinate transformation parameter, default value is 0, unit: meters.
- `-z`: Coordinate transformation parameter, default value is 0, unit: meters.
- `-roll`: Coordinate transformation parameter, default value is 0, unit: degrees.
- `-pitch`: Coordinate transformation parameter, default value is 0, unit: degrees.
- `-yaw`: Coordinate transformation parameter, default value is 0, unit: degrees.

Note: To use the coordinate transformation functionality, you need to enable the compilation option `ENABLE_TRANSFORM=ON`.


## 6.3 Usage Example

Receive MSOP packets from the online LiDAR `vanjee_720`. Coordinate transformation parameters are: `x=1`, `y=2`, `z=0`, `roll=30`, `pitch=0`, `yaw=0`.

```bash
vanjee_driver_viewer -x 1 -y 2 -roll 30
```
