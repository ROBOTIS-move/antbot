# antbot_camera

A multi-driver ROS 2 camera package with a polymorphic driver architecture. Supports multiple camera types through a unified interface with automatic device discovery, vendor-specific calibration, and error recovery.

**Supported cameras:**

- Novitec V4L2 cameras (with ROM-based intrinsic calibration)
- Generic V4L2 cameras (UYVY/YUYV pixel formats)
- USB cameras via OpenCV VideoCapture (with optional rotation)
- Orbbec Gemini 336L RGB-D camera (color + depth streams)

**License:** Apache-2.0

> For driver internals, calibration details, and source code reference, see [README_DEV.md](README_DEV.md).

## System Requirements

- **Platform:** NVIDIA Jetson (Linux aarch64, tested on L4T 5.15.136-tegra)
- **ROS 2 Humble** with ament_cmake build system
- **C++17** compiler
- **Video device access** (`/dev/videoX`) — the user must be in the `video` group:
  ```bash
  sudo usermod -aG video $USER
  ```

## Dependencies

### ROS 2 Packages

| Package | Description |
|---------|-------------|
| `rclcpp` | ROS 2 C++ client library |
| `rclcpp_components` | Component node support |
| `sensor_msgs` | Image and CameraInfo messages |
| `std_msgs` | Standard message types |
| `cv_bridge` | OpenCV ↔ ROS image conversion |
| `orbbec_camera` | Orbbec SDK ROS 2 wrapper (Gemini 336L support) |

### System Libraries

| Library | Description |
|---------|-------------|
| OpenCV | Image processing and video capture |
| libdl | Dynamic library loading |
| spdlog | Logging (build-time dependency only) |

### Bundled

| Library | Description |
|---------|-------------|
| `lib/librectrl.so` | Novitec RECTRL SDK (precompiled aarch64 binary) |

## Build

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --symlink-install --packages-select antbot_camera

# Source the workspace
source install/setup.bash
```

> **Note:** The `orbbec_camera` package is declared as a build dependency and must be present in the workspace. If you do not need Gemini 336L support, remove `orbbec_camera` from `package.xml` and the `GroupAction` block in `launch/camera.launch.py`.

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│                         CameraNode                           │
│                    (wall timer ~30 Hz)                        │
│                                                              │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐  │
│  │  V4l2Driver    │  │  V4l2Driver    │  │   UsbDriver    │  │
│  │  (left)        │  │  (rear)        │  │   (cargo)      │  │
│  │                │  │                │  │                │  │
│  │ NovitecHandler │  │ GenericHandler │  │                │  │
│  └───────┬────────┘  └───────┬────────┘  └───────┬────────┘  │
│          │                   │                    │           │
│    /dev/video0         /dev/video4          /dev/video6       │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│  Gemini 336L (external orbbec_camera)    │
│  /sensor/camera/gemini336l/front/        │
└──────────────────────────────────────────┘
```

The package uses a two-layer polymorphic design:

**Driver layer** — Abstracted by `CameraDriver`, with concrete implementations for each camera type:

| Driver | Backend | Use case |
|--------|---------|----------|
| `V4l2Driver` | Linux V4L2 (mmap) | Novitec cameras, generic V4L2 cameras |
| `UsbDriver` | OpenCV `VideoCapture` | Standard USB cameras |
| Gemini 336L | External `orbbec_camera` node | Orbbec RGB-D camera |

**Vendor handler layer** — Abstracted by `CameraVendorHandler`, handles device discovery and calibration:

| Handler | Device discovery | ROM calibration |
|---------|-----------------|-----------------|
| `NovitecCameraHandler` | sysfs `vi-output` port matching | Supported (RECTRL SDK) |
| `GenericCameraHandler` | sysfs name substring matching | Not supported |

All drivers share a common lifecycle: `initialize()` → `update()` → `release()`. The `CameraNode` polls all drivers at ~30 Hz via a single wall timer, publishing frames and automatically recovering drivers that enter an error state.

## Configuration

All camera configuration is defined in `param/camera_config.yaml`. The configuration is organized by driver type, with shared settings and per-position overrides.

### Top-Level Structure

```yaml
/**:
  ros__parameters:
    camera:
      camera_list:          # List of enabled driver types
        - "v4l2_driver"
        - "usb_camera"

      v4l2_driver:          # V4L2 driver section
        ...
      usb_camera:           # USB camera section
        ...
```

The `camera_list` array controls which driver types are instantiated. Only listed types are created at startup.

### V4L2 Driver Configuration

```yaml
v4l2_driver:
  resolution:
    width: 640              # Output image width (pixels)
    height: 360             # Output image height (pixels)
  default_fps: 30           # Default frame rate (per-position override available)
  use_rom_mode: true        # Enable ROM-based calibration (Novitec cameras)
  port_info:
    root: "/sys/class/video4linux"  # sysfs root for device discovery
  camera_position_list:     # List of camera positions to instantiate
    - "left"
    - "front"
    - "right"
    - "rear"
```

**Per-position settings:**

```yaml
  left:
    port: "2-001a"                          # Port ID for device discovery
    fourcc_type: "UYVY"                     # Pixel format: "UYVY" or "YUYV"
    distortion_model: "equidistant"         # "equidistant" or "plumb_bob"
    frame_id: "left_camera_optical_frame"   # TF frame ID
    # vendor: "novitec"                     # Default; set to "Generic" to skip ROM calibration
    # fps: 30                               # Per-position FPS override (defaults to default_fps)
    # grab_width: 1920                      # Capture width (default: 1920)
    # grab_height: 1080                     # Capture height (default: 1080)
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | (required) | Port ID string used for device lookup via sysfs. Novitec cameras use the USB port suffix (e.g., `2-001a`). Generic cameras match by device name substring. |
| `fourcc_type` | `"UYVY"` | V4L2 pixel format. Determines color conversion: UYVY→BGR or YUYV→RGB. |
| `distortion_model` | `"equidistant"` | Lens distortion model. Affects ROM calibration data interpretation and CameraInfo output. |
| `frame_id` | `"{position}_camera_optical_frame"` | TF frame ID published in image and CameraInfo headers. |
| `vendor` | `"novitec"` | Vendor handler selection. Use `"Generic"` for non-Novitec V4L2 cameras. |
| `fps` | `default_fps` value | Per-position frame rate override. Falls back to the parent `default_fps` value if not specified. |
| `grab_width` / `grab_height` | 1920 / 1080 | Capture resolution. Can differ from output resolution (`resolution`). |

### USB Camera Configuration

```yaml
usb_camera:
  resolution:
    width: 640              # Output image width (pixels)
    height: 480             # Output image height (pixels)
  default_fps: 10           # Default frame rate
  camera_position_list:
    - "cargo"
```

**Per-position settings:**

```yaml
  cargo:
    device_name: "Integrated Camera"            # Device name for sysfs lookup
    rotation: ""                                # Rotation: "", "90", "180", "270"
    frame_id: "cargo_camera_optical_frame"      # TF frame ID
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_name` | (required) | Device name substring to search for in `/sys/class/video4linux/*/name`. |
| `rotation` | `""` (none) | Image rotation. `"90"` = clockwise, `"180"`, `"270"` = counterclockwise. |
| `frame_id` | `"{position}_camera_optical_frame"` | TF frame ID. |
| `fps` | `default_fps` value | Per-position frame rate override. |

### Gemini 336L Configuration

The Gemini 336L is not configured via `camera_config.yaml`. It is enabled through launch arguments that include the `orbbec_camera` launch file. See the [Usage](#usage) section for launch arguments.

### Full Configuration Example

```yaml
/**:
  ros__parameters:
    camera:
      camera_list:
        - "v4l2_driver"
        - "usb_camera"

      v4l2_driver:
        resolution:
          width: 640
          height: 360
        default_fps: 30
        use_rom_mode: true
        port_info:
          root: "/sys/class/video4linux"
        camera_position_list:
          - "left"
          - "front"
          - "right"
          - "rear"
        left:
          port: "2-001a"
          fourcc_type: "UYVY"
          distortion_model: "equidistant"
          frame_id: "left_camera_optical_frame"
        front:
          port: "2-001b"
          fourcc_type: "UYVY"
          distortion_model: "equidistant"
          frame_id: "front_camera_optical_frame"
        right:
          port: "2-001c"
          fourcc_type: "UYVY"
          distortion_model: "equidistant"
          frame_id: "right_camera_optical_frame"
        rear:
          fps: 15
          vendor: "Generic"
          port: "XCG"
          fourcc_type: "YUYV"
          grab_width: 640
          grab_height: 480
          frame_id: "rear_camera_optical_frame"

      usb_camera:
        resolution:
          width: 640
          height: 480
        default_fps: 10
        camera_position_list:
          - "cargo"
        cargo:
          device_name: "Integrated Camera"
          rotation: ""
          frame_id: "cargo_camera_optical_frame"
```

## Usage

### Main Camera Node

Launch with default configuration:

```bash
ros2 launch antbot_camera camera.launch.py
```

Launch with a custom configuration file:

```bash
ros2 launch antbot_camera camera.launch.py config_file:=/path/to/custom_config.yaml
```

### Gemini 336L RGB-D Camera

The Gemini 336L is always launched alongside the main camera node. To specify a particular device, pass the serial number or USB port:

```bash
ros2 launch antbot_camera camera.launch.py \
  gemini336l_serial_number:=<SERIAL_NUMBER> \
  gemini336l_usb_port:=<USB_PORT>
```

The Orbbec camera node launches under the `/sensor/camera/gemini336l` namespace with the following default settings:

| Setting | Value |
|---------|-------|
| Color resolution | 640 x 480 @ 15 fps |
| Depth resolution | 640 x 480 @ 15 fps |
| Depth registration | Enabled |
| Alignment mode | HW (hardware) |
| Point cloud | Disabled |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `param/camera_config.yaml` | Path to camera configuration file |
| `gemini336l_serial_number` | `""` | Gemini 336L serial number filter |
| `gemini336l_usb_port` | `""` | Gemini 336L USB port filter |

### Test Node

The test node subscribes to all configured camera topics, overlays diagnostic information (topic name, frame ID, timestamp, resolution, FPS, calibration data), and saves frames as JPEG images.

```bash
ros2 launch antbot_camera camera_test.launch.py
```

With custom parameters:

```bash
ros2 launch antbot_camera camera_test.launch.py \
  output_dir:=/tmp/frames \
  save_interval_sec:=2.0
```

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `param/camera_test_config.yaml` | Config file (reads `camera_list` and position lists) |
| `output_dir` | `/tmp/antbot_camera_test` | Directory to save captured frames |
| `save_interval_sec` | `1.0` | Minimum interval between saved frames (seconds) |

## ROS 2 Interface

### Topic Naming Convention

```
/sensor/camera/{driver_type}/{position}/image_raw
/sensor/camera/{driver_type}/{position}/camera_info
```

For the Gemini 336L (launched via `orbbec_camera`), topics appear under the `/sensor/camera/gemini336l/front/` namespace following the Orbbec SDK topic structure.

### Message Types

| Topic suffix | Message type | Encoding | Description |
|-------------|-------------|----------|-------------|
| `/image_raw` | `sensor_msgs/msg/Image` | `bgr8` | Color image |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | — | Intrinsic calibration and distortion |

Gemini 336L depth images use `16UC1` encoding (depth in millimeters).

### QoS Profile

All publishers use `SensorDataQoS`:
- **Reliability:** Best effort
- **Durability:** Volatile
- **History depth:** 5

### Example Topic List

With the default configuration:

| Topic | Type |
|-------|------|
| `/sensor/camera/v4l2_driver/left/image_raw` | `sensor_msgs/msg/Image` |
| `/sensor/camera/v4l2_driver/left/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/sensor/camera/v4l2_driver/front/image_raw` | `sensor_msgs/msg/Image` |
| `/sensor/camera/v4l2_driver/front/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/sensor/camera/v4l2_driver/right/image_raw` | `sensor_msgs/msg/Image` |
| `/sensor/camera/v4l2_driver/right/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/sensor/camera/v4l2_driver/rear/image_raw` | `sensor_msgs/msg/Image` |
| `/sensor/camera/v4l2_driver/rear/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/sensor/camera/usb_camera/cargo/image_raw` | `sensor_msgs/msg/Image` |
| `/sensor/camera/usb_camera/cargo/camera_info` | `sensor_msgs/msg/CameraInfo` |

## Troubleshooting

### Camera not detected

- Verify the user is in the `video` group: `groups $USER`
- Check that the device exists: `ls /dev/video*`
- For Novitec cameras, verify the port ID matches the sysfs entry:
  ```bash
  cat /sys/class/video4linux/video*/name
  ```
- For USB cameras, check the device name matches the sysfs entry.

### ROM calibration failure

- Check the Novitec bridge board firmware version in the log output.
- ROM reads are retried up to 2 times on invalid data.
- Verify the intrinsic parameter values in the log (look for "ROM calibration intrinsic parameters").
- If parameters are consistently out of range, the camera ROM may need to be reprogrammed.

### Frame drop / low FPS

- Reduce resolution or FPS in `camera_config.yaml`.
- Check USB bandwidth — multiple high-resolution cameras on the same USB hub can cause contention.
- Monitor for "Dead lock detected in ioctl" messages, which indicate V4L2 buffer timeout issues.

### Gemini 336L issues

- Ensure the `orbbec_camera` package is installed and built.
- Verify the serial number and/or USB port arguments match the connected device.
- Check that the Orbbec SDK udev rules are installed.
