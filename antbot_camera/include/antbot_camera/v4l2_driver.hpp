// Copyright 2026 ROBOTIS AI CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Jaehun Park

#ifndef ANTBOT_CAMERA__V4L2_DRIVER_HPP_
#define ANTBOT_CAMERA__V4L2_DRIVER_HPP_

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "opencv2/imgproc.hpp"

#include "antbot_camera/camera_driver.hpp"
#include "antbot_camera/vendor/camera_vendor_handler.hpp"

namespace antbot_camera
{

struct Intrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double ppx = 0.0;
  double ppy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
  double k5 = 0.0;
  double k6 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  std::string distortion_model = "plumb_bob";
  std::string frame_id;
};

class V4l2Driver : public CameraDriver
{
public:
  V4l2Driver(
    const std::string & position,
    rclcpp::Node * node,
    const cv::Size & image_size,
    int fps,
    const std::string & port_id,
    const std::string & frame_id,
    const std::string & fourcc_type,
    bool use_rom_mode,
    const cv::Size & grab_size,
    const std::string & distortion_model,
    const std::shared_ptr<vendor::CameraVendorHandler> & vendor_handler);
  ~V4l2Driver() override;

  bool initialize() override;
  bool update() override;
  void release() override;

private:
  struct buffer
  {
    void * start = nullptr;
    size_t length = 0;
  };

  bool init_format();
  bool init_framerate();
  bool init_buffers();
  bool test_grab_once();
  void log_supported_framerates();
  void measure_framerate();

  void query_buffer(struct v4l2_buffer & buf);
  void query_stream_on();
  void query_stream_off();

  bool queue_buffer(int index);
  bool dequeue_buffer();
  std::future<void> query_dequeue_buffer_async();
  std::future<void> query_queue_buffer_async(int index);

  bool load_camera_calibration();
  void set_intrinsics_from_vec(
    Intrinsics & intrinsics,
    const std::vector<double> & intrinsic_vec);
  bool is_valid_intrinsic_vec(const std::vector<double> & vec);
  void populate_camera_info();

  bool get_driver_connected_flag();
  void close_device();
  void print_error();

  cv::Size image_size_;
  cv::Size grab_size_;
  int fps_;
  std::string port_id_;
  std::string port_path_;
  std::string frame_id_;
  std::string fourcc_type_;
  std::string distortion_model_;
  bool use_rom_mode_;

  std::shared_ptr<vendor::CameraVendorHandler> vendor_handler_;
  int vendor_device_id_ = -1;

  int driver_ = -1;
  int buffer_count_ = 4;
  struct v4l2_format format_;
  struct v4l2_requestbuffers req_buffers_;
  struct v4l2_buffer buffer_;
  struct buffer * buffers_ = nullptr;

  cv::Mat yuv_image_;
  cv::Mat color_image_;
  std::mutex grab_mutex_;

  Intrinsics intrinsics_;
  std::vector<double> intrinsic_vec_;
};

}  // namespace antbot_camera
#endif  // ANTBOT_CAMERA__V4L2_DRIVER_HPP_
