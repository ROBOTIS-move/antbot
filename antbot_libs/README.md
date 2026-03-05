# antbot_libs

Shared C++ library for the ANTBot robot platform. Provides common utilities used across antbot packages.

## Overview

`antbot_libs` currently provides the following components:

- **Communicator** — Thread-safe, single-device serial communication via Dynamixel Protocol 2.0
- **ControlTableParser** — Parses XML files that define hardware register maps (address, length, R/W permission)
- **NodeThread** — Background thread utility for spinning rclcpp nodes/executors

## API

### ControlTableParser

Loads a control table definition from an XML file and provides lookup by item name.

```cpp
#include "antbot_libs/control_table_parser.hpp"

antbot::libs::ControlTableParser parser;
parser.load_xml_file("/path/to/control_table.xml");
parser.parse_control_table();

auto table = parser.get_control_table();
// table["Item_Name"] → {address, length, rw}
```

**XML format:**

```xml
<Device Name="DeviceName" ModelNumber="123" MinAddress="0" MaxAddress="255">
  <ControlItems>
    <Item Name="Item_Name" Address="0" Length="2" RW="3"/>
    <!-- ... -->
  </ControlItems>
</Device>
```

### Communicator

Thread-safe interface for reading and writing hardware registers.

```cpp
#include "antbot_libs/communicator.hpp"

// Factory function: opens port, sets baud rate, initializes, and verifies connection
auto communicator = antbot::libs::create_communicator(
  "/dev/ttyUSB0", 4000000, 2.0f, 200, "/path/to/control_table.xml");

// Read entire control table into internal buffer
int result = 0;
communicator->read_control_table(&result);

// Type-safe read from buffer
int16_t rpm = communicator->get_data<int16_t>("M1_Present_RPM");

// Write a value by item name
std::string msg;
communicator->write("M1_Goal_RPM", 100, &msg);

// Batch write (consecutive items)
int16_t rpms[4] = {100, -100, 100, -100};
communicator->write_batch("M1_Goal_RPM", rpms, 4, &msg);

// Partial read (address range)
communicator->read_control_table(min_addr, max_addr, &result);

// Raw write by address
uint8_t raw_data[4] = {0x64, 0x00, 0x00, 0x00};
communicator->write(41, 4, raw_data, &msg);

// Device utilities
bool connected = communicator->is_connected_to_device();
uint16_t model = communicator->get_model_number();
uint8_t fw_major = communicator->get_firmware_major_version();
uint8_t fw_minor = communicator->get_firmware_minor_version();
```

**Thread safety:** All read/write operations are protected by `std::mutex`. The internal data buffer (`data_mutex_`) and SDK handlers (`sdk_handler_mutex_`) use separate mutexes for minimal contention.

### NodeThread

Spins an rclcpp node in a dedicated background thread.

```cpp
#include "antbot_libs/node_thread.hpp"

auto node = rclcpp::Node::make_shared("my_node");
auto thread = std::make_unique<antbot::libs::NodeThread>(
  node->get_node_base_interface());
// Node callbacks are processed in the background thread
```

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `rclcpp` | ROS 2 C++ client library (logging) |
| `dynamixel_sdk` | ROBOTIS Dynamixel SDK |
| `tinyxml2_vendor` | XML parsing library |

## Build

```bash
colcon build --symlink-install --packages-select antbot_libs
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
