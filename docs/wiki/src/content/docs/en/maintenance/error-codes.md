---
title: 6.3 Error Codes
description: AntBot error code list and troubleshooting
sidebar:
  order: 3
---

## Firmware Error Codes

<details>
<summary>View all RCU error codes</summary>

| Code | Category | Description |
| :--- | :---: | :--- |
| `0x00000000` | System | Normal (No error) |
| `0x0000001E` | System | Hardware initialization failed |
| `0x01030011` | Task | Task thread creation failed |
| `0x01030012` | Task | Task thread start failed |
| `0x01030014` | Task | System check timeout |
| `0x01030015` | Task | System check error |
| `0x01050001` | E-Stop | Emergency stop activated |
| `0x02010001` | Battery | Battery low voltage |
| `0x02010002` | Battery | Battery over voltage |
| `0x02010005` | Battery | BMS disconnected |
| `0x02030003` | Power | Regulator low voltage |
| `0x02030004` | Power | Regulator over voltage |
| `0x03020004` | Motor | Motor not connected |
| `0x03020009` | Motor | Motor driver over temperature |
| `0x03020010` | Motor | Motor state fault |
| `0x03020011` | Motor | Motor overload |
| `0x03020015` | Motor | Motor over temperature |
| `0x03020017` | Motor | Hall sensor failure |
| `0x03020019` | Motor | STO1 safety function triggered |
| `0x0302001C` | Motor | Input voltage fault |
| `0x0302001D` | Motor | Inverter fault |
| `0x0302001E` | Motor | Encoder fault |
| `0x0302001F` | Motor | Calibration fault |
| `0x03020020` | Motor | Bus watchdog timeout |
| `0x03020021` | Motor | Motor over speed |
| `0x03020022` | Motor | Undefined motor error |
| `0x040E0002` | Sensor | Ultrasonic sensor connection failed |
| `0x040F0001` | Sensor | Pressure sensor failure |
| `0x07020001` | Cargo | Cargo is locked |
| `0x07020002` | Cargo | Cargo is not locked |
| `0x07020003` | Cargo | Cargo solenoid error |

</details>
