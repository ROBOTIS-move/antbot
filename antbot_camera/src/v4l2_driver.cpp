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

#include "antbot_camera/v4l2_driver.hpp"

namespace antbot_camera
{

V4l2Driver::V4l2Driver(
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
  const std::shared_ptr<vendor::CameraVendorHandler> & vendor_handler)
: CameraDriver(position, node),
  image_size_(image_size),
  grab_size_(grab_size),
  fps_(fps),
  port_id_(port_id),
  frame_id_(frame_id),
  fourcc_type_(fourcc_type),
  distortion_model_(distortion_model),
  use_rom_mode_(use_rom_mode),
  vendor_handler_(vendor_handler)
{
  frame_data_.frame_id = frame_id;
  intrinsics_.distortion_model = distortion_model;
  intrinsics_.frame_id = frame_id;

  memset(&format_, 0, sizeof(format_));
  memset(&req_buffers_, 0, sizeof(req_buffers_));
  memset(&buffer_, 0, sizeof(buffer_));
}

V4l2Driver::~V4l2Driver()
{
  release();
}

bool V4l2Driver::initialize()
{
  // Resolve device path from port_id via vendor handler
  port_path_ = vendor_handler_->get_device_path(port_id_);
  if (port_path_ == "Unknown") {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to find device path for port: %s", port_id_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }

  driver_ = open(port_path_.c_str(), O_RDWR);
  if (driver_ == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open camera: %s", port_path_.c_str());
    print_error();
    set_status(DriverStatus::ERROR);
    return false;
  }

  if (!init_format()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set format: %s", port_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }
  if (!init_framerate()) {
    // Detailed warning is logged inside init_framerate()
  }
  if (!init_buffers()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set buffers: %s", port_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }

  vendor_device_id_ = vendor_handler_->get_vendor_device_id(port_id_);
  if (vendor_device_id_ == -1 && vendor_handler_->supports_rom_mode()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to prepare vendor device: %s", port_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }

  if (!load_camera_calibration()) {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to load camera calibration: %s", port_path_.c_str());
  }

  yuv_image_.release();
  yuv_image_.create(
    format_.fmt.pix.height,
    format_.fmt.pix.width,
    CV_8UC2);

  if (!test_grab_once()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to grab test frame: %s", port_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }

  query_stream_on();

  RCLCPP_INFO(node_->get_logger(), "Initialized v4l2 driver: %s", port_path_.c_str());
  set_status(DriverStatus::OK);
  return true;
}

bool V4l2Driver::update()
{
  if (get_status() != DriverStatus::OK || !get_driver_connected_flag()) {
    return false;
  }

  if (!dequeue_buffer()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to dequeue buffer: %s", port_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(grab_mutex_);
    memcpy(
      yuv_image_.data,
      buffers_[buffer_.index].start,
      format_.fmt.pix.sizeimage);

    if (fourcc_type_ == "YUYV") {
      cv::cvtColor(yuv_image_, color_image_, cv::COLOR_YUV2RGB_YVYU);
    } else {
      cv::cvtColor(yuv_image_, color_image_, cv::COLOR_YUV2BGR_UYVY);
    }
  }

  queue_buffer(buffer_.index);

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    color_image_.copyTo(frame_data_.color);
  }
  return true;
}

void V4l2Driver::release()
{
  query_stream_off();
  close_device();
  if (buffers_ != nullptr) {
    for (int i = 0; i < buffer_count_; ++i) {
      if (buffers_[i].start != nullptr) {
        munmap(buffers_[i].start, buffers_[i].length);
      }
    }
    free(buffers_);
    buffers_ = nullptr;
  }
  set_status(DriverStatus::UNINITIALIZED);
}

bool V4l2Driver::test_grab_once()
{
  if (!get_driver_connected_flag()) {
    RCLCPP_WARN(node_->get_logger(), "Driver is not ready: %s", port_path_.c_str());
    return false;
  }
  query_stream_on();
  if (!dequeue_buffer()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to dequeue buffer: %s", port_path_.c_str());
    return false;
  }
  query_stream_off();

  // STREAMOFF dequeues all buffers; re-queue all before next STREAMON
  for (int i = 0; i < buffer_count_; ++i) {
    memset(&buffer_, 0, sizeof(buffer_));
    buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer_.memory = V4L2_MEMORY_MMAP;
    buffer_.index = i;
    if (!queue_buffer(i)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to re-queue buffer %d: %s", i, port_path_.c_str());
      return false;
    }
  }
  return true;
}

bool V4l2Driver::init_format()
{
  memset(&format_, 0, sizeof(format_));
  format_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format_.fmt.pix.width = grab_size_.width;
  format_.fmt.pix.height = grab_size_.height;
  format_.fmt.pix.field = V4L2_FIELD_NONE;

  if (fourcc_type_ == "YUYV") {
    format_.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  } else {
    format_.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
  }

  if (ioctl(driver_, VIDIOC_S_FMT, &format_) == -1) {
    print_error();
    close_device();
    return false;
  }

  if (static_cast<int>(format_.fmt.pix.width) != grab_size_.width ||
    static_cast<int>(format_.fmt.pix.height) != grab_size_.height)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Resolution adjusted by driver: requested %dx%d, actual %ux%u for %s",
      grab_size_.width, grab_size_.height,
      format_.fmt.pix.width, format_.fmt.pix.height,
      port_path_.c_str());
    grab_size_.width = format_.fmt.pix.width;
    grab_size_.height = format_.fmt.pix.height;
  }
  return true;
}

bool V4l2Driver::init_framerate()
{
  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps_;

  if (ioctl(driver_, VIDIOC_S_PARM, &parm) == -1) {
    // Vendors that don't support framerate setting (e.g. Novitec) use hardware-fixed fps
    if (!vendor_handler_->supports_framerate_setting()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Framerate is fixed at 30 fps by hardware (requested %d fps): %s",
        fps_, port_path_.c_str());
      return false;
    }
    // Try querying current framerate via G_PARM
    struct v4l2_streamparm current_parm;
    memset(&current_parm, 0, sizeof(current_parm));
    current_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(driver_, VIDIOC_G_PARM, &current_parm) == 0 &&
      current_parm.parm.capture.timeperframe.numerator > 0)
    {
      int cur_den = current_parm.parm.capture.timeperframe.denominator;
      int cur_num = current_parm.parm.capture.timeperframe.numerator;
      RCLCPP_WARN(
        node_->get_logger(),
        "Framerate is fixed by device: requested %d fps, current %d/%d fps for %s",
        fps_, cur_den, cur_num, port_path_.c_str());
    } else {
      // G_PARM also failed — query supported framerates via ENUM_FRAMEINTERVALS
      log_supported_framerates();
    }
    return false;
  }

  int actual_den = parm.parm.capture.timeperframe.denominator;
  int actual_num = parm.parm.capture.timeperframe.numerator;
  if (actual_num > 0 && actual_den / actual_num != fps_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Framerate adjusted by driver: requested %d, actual %d/%d for %s",
      fps_, actual_den, actual_num, port_path_.c_str());
  }
  return true;
}

void V4l2Driver::log_supported_framerates()
{
  struct v4l2_frmivalenum frmival;
  memset(&frmival, 0, sizeof(frmival));
  frmival.index = 0;
  frmival.pixel_format = format_.fmt.pix.pixelformat;
  frmival.width = grab_size_.width;
  frmival.height = grab_size_.height;

  std::string supported;
  while (ioctl(driver_, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
    if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
      if (!supported.empty()) {
        supported += ", ";
      }
      supported += std::to_string(frmival.discrete.denominator) + "/" +
        std::to_string(frmival.discrete.numerator);
    }
    frmival.index++;
  }

  if (!supported.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to set framerate (%d fps). Supported framerates: [%s] fps for %s",
      fps_, supported.c_str(), port_path_.c_str());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to set framerate (%d fps) and unable to query device info: %s",
      fps_, port_path_.c_str());
  }
}

void V4l2Driver::measure_framerate()
{
  constexpr int kMeasureFrames = 10;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (ioctl(driver_, VIDIOC_STREAMON, &type) == -1) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to start stream for framerate measurement: %s", port_path_.c_str());
    return;
  }

  struct v4l2_buffer buf;

  // Warm-up: discard first frame
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(driver_, VIDIOC_DQBUF, &buf) == -1) {
    ioctl(driver_, VIDIOC_STREAMOFF, &type);
    return;
  }
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  ioctl(driver_, VIDIOC_QBUF, &buf);

  // Measure frame intervals using v4l2_buffer.timestamp (kernel capture timestamp)
  struct timeval first_ts = {0, 0};
  struct timeval last_ts = {0, 0};
  int captured = 0;

  for (int i = 0; i < kMeasureFrames; ++i) {
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(driver_, VIDIOC_DQBUF, &buf) == -1) {
      break;
    }
    if (captured == 0) {
      first_ts = buf.timestamp;
    }
    last_ts = buf.timestamp;
    captured++;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ioctl(driver_, VIDIOC_QBUF, &buf);
  }

  ioctl(driver_, VIDIOC_STREAMOFF, &type);

  // Re-queue all buffers after STREAMOFF
  for (int i = 0; i < buffer_count_; ++i) {
    struct v4l2_buffer rebuf;
    memset(&rebuf, 0, sizeof(rebuf));
    rebuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rebuf.memory = V4L2_MEMORY_MMAP;
    rebuf.index = i;
    ioctl(driver_, VIDIOC_QBUF, &rebuf);
  }

  if (captured > 1) {
    double elapsed =
      (last_ts.tv_sec - first_ts.tv_sec) +
      (last_ts.tv_usec - first_ts.tv_usec) / 1e6;
    if (elapsed > 0.0) {
      double measured_fps = (captured - 1) / elapsed;
      RCLCPP_INFO(
        node_->get_logger(),
        "Measured framerate: %.1f fps (%d frames in %.3f s) for %s",
        measured_fps, captured, elapsed, port_path_.c_str());
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to measure framerate (insufficient frames captured): %s",
      port_path_.c_str());
  }
}

bool V4l2Driver::init_buffers()
{
  memset(&req_buffers_, 0, sizeof(req_buffers_));
  req_buffers_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req_buffers_.memory = V4L2_MEMORY_MMAP;
  req_buffers_.count = buffer_count_;

  if (ioctl(driver_, VIDIOC_REQBUFS, &req_buffers_) == -1) {
    print_error();
    close_device();
    return false;
  }
  buffers_ = static_cast<struct buffer *>(calloc(buffer_count_, sizeof(*buffers_)));

  for (int i = 0; i < buffer_count_; ++i) {
    memset(&buffer_, 0, sizeof(buffer_));
    buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer_.memory = V4L2_MEMORY_MMAP;
    buffer_.index = i;

    query_buffer(buffer_);

    buffers_[i].length = buffer_.length;
    buffers_[i].start = mmap(
      NULL, buffer_.length, PROT_READ | PROT_WRITE, MAP_SHARED, driver_, buffer_.m.offset);

    if (buffers_[i].start == MAP_FAILED) {
      print_error();
      close_device();
      return false;
    }
    queue_buffer(i);
  }
  return true;
}

void V4l2Driver::query_buffer(struct v4l2_buffer & buf)
{
  if (ioctl(driver_, VIDIOC_QUERYBUF, &buf) == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to query buffer: %s", port_path_.c_str());
    print_error();
    close_device();
  }
}

void V4l2Driver::query_stream_on()
{
  if (!get_driver_connected_flag()) {
    return;
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(driver_, VIDIOC_STREAMON, &type) == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start stream: %s", port_path_.c_str());
    print_error();
    close_device();
  }
}

void V4l2Driver::query_stream_off()
{
  if (!get_driver_connected_flag()) {
    return;
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(driver_, VIDIOC_STREAMOFF, &type) == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to stop stream: %s", port_path_.c_str());
    print_error();
    close_device();
  }
}

bool V4l2Driver::queue_buffer(const int index)
{
  if (!(buffer_.flags & V4L2_BUF_FLAG_QUEUED)) {
    auto future = query_queue_buffer_async(index);
    if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout) {
      RCLCPP_ERROR(
        node_->get_logger(), "Dead lock detected in ioctl queue buffer: %s", port_path_.c_str());
      set_status(DriverStatus::ERROR);
      close_device();
      return false;
    }
  }
  if (!get_driver_connected_flag()) {
    return false;
  }
  return true;
}

bool V4l2Driver::dequeue_buffer()
{
  if (buffer_.flags & V4L2_BUF_FLAG_QUEUED) {
    auto future = query_dequeue_buffer_async();
    if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Dead lock detected in ioctl dequeue buffer: %s", port_path_.c_str());
      set_status(DriverStatus::ERROR);
      close_device();
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Buffer is not queued: %s", port_path_.c_str());
    return false;
  }
  if (!get_driver_connected_flag()) {
    return false;
  }
  return true;
}

std::future<void> V4l2Driver::query_dequeue_buffer_async()
{
  return std::async(
    std::launch::async, [this]() {
      if (ioctl(driver_, VIDIOC_DQBUF, &buffer_) == -1) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to dequeue buffer: %s", port_path_.c_str());
        print_error();
        close_device();
      }
    });
}

std::future<void> V4l2Driver::query_queue_buffer_async(const int index)
{
  buffer_.index = index;
  buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer_.memory = V4L2_MEMORY_MMAP;
  return std::async(
    std::launch::async, [this]() {
      if (ioctl(driver_, VIDIOC_QBUF, &buffer_) == -1) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to queue buffer: %s", port_path_.c_str());
        print_error();
        close_device();
      }
    });
}

bool V4l2Driver::load_camera_calibration()
{
  if (!use_rom_mode_ || !vendor_handler_->supports_rom_mode()) {
    RCLCPP_WARN(
      node_->get_logger(), "Vendor does not support ROM mode for: %s", position_.c_str());
    return false;
  }

  vendor::CalibrationData calib_data;
  if (!vendor_handler_->read_calibration_data(
      port_id_, vendor_device_id_, distortion_model_,
      cv::Size(image_size_.width, image_size_.height), calib_data))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to read calibration from ROM for: %s", position_.c_str());
    return false;
  }

  intrinsic_vec_ = calib_data.intrinsic;
  set_intrinsics_from_vec(intrinsics_, intrinsic_vec_);
  populate_camera_info();
  RCLCPP_INFO(
    node_->get_logger(), "Loaded calibration from ROM for: %s", position_.c_str());
  return true;
}

void V4l2Driver::set_intrinsics_from_vec(
  Intrinsics & intrinsics,
  const std::vector<double> & intrinsic_vec)
{
  intrinsics.fx = intrinsic_vec[0];
  intrinsics.fy = intrinsic_vec[1];
  intrinsics.ppx = intrinsic_vec[2];
  intrinsics.ppy = intrinsic_vec[3];

  intrinsics.distortion_model = distortion_model_;

  if (intrinsics.distortion_model == "equidistant" ||
    intrinsics.distortion_model == "fisheye")
  {
    intrinsics.k1 = (intrinsic_vec.size() > 4) ? intrinsic_vec[4] : 0.0;
    intrinsics.k2 = (intrinsic_vec.size() > 5) ? intrinsic_vec[5] : 0.0;
    intrinsics.k3 = (intrinsic_vec.size() > 6) ? intrinsic_vec[6] : 0.0;
    intrinsics.k4 = (intrinsic_vec.size() > 7) ? intrinsic_vec[7] : 0.0;
    intrinsics.p1 = 0.0;
    intrinsics.p2 = 0.0;
    intrinsics.k5 = 0.0;
    intrinsics.k6 = 0.0;
  } else {
    intrinsics.k1 = (intrinsic_vec.size() > 4) ? intrinsic_vec[4] : 0.0;
    intrinsics.k2 = (intrinsic_vec.size() > 5) ? intrinsic_vec[5] : 0.0;
    intrinsics.p1 = (intrinsic_vec.size() > 6) ? intrinsic_vec[6] : 0.0;
    intrinsics.p2 = (intrinsic_vec.size() > 7) ? intrinsic_vec[7] : 0.0;
    intrinsics.k3 = (intrinsic_vec.size() > 8) ? intrinsic_vec[8] : 0.0;
    intrinsics.k4 = (intrinsic_vec.size() > 9) ? intrinsic_vec[9] : 0.0;
    intrinsics.k5 = (intrinsic_vec.size() > 10) ? intrinsic_vec[10] : 0.0;
    intrinsics.k6 = (intrinsic_vec.size() > 11) ? intrinsic_vec[11] : 0.0;
  }
}

bool V4l2Driver::is_valid_intrinsic_vec(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return false;
  }
  return !std::all_of(
    vec.begin(), vec.end(),
    [](double v) {return v == 0.0;});
}

void V4l2Driver::populate_camera_info()
{
  auto & info = frame_data_.color_info;
  info.width = image_size_.width;
  info.height = image_size_.height;
  info.header.frame_id = frame_id_;
  info.distortion_model = distortion_model_;

  // K matrix (3x3)
  info.k[0] = intrinsics_.fx;
  info.k[1] = 0.0;
  info.k[2] = intrinsics_.ppx;
  info.k[3] = 0.0;
  info.k[4] = intrinsics_.fy;
  info.k[5] = intrinsics_.ppy;
  info.k[6] = 0.0;
  info.k[7] = 0.0;
  info.k[8] = 1.0;

  // D vector
  info.d = {intrinsics_.k1, intrinsics_.k2, intrinsics_.p1, intrinsics_.p2, intrinsics_.k3};

  // P matrix (3x4) - projection matrix
  info.p[0] = intrinsics_.fx;
  info.p[1] = 0.0;
  info.p[2] = intrinsics_.ppx;
  info.p[3] = 0.0;
  info.p[4] = 0.0;
  info.p[5] = intrinsics_.fy;
  info.p[6] = intrinsics_.ppy;
  info.p[7] = 0.0;
  info.p[8] = 0.0;
  info.p[9] = 0.0;
  info.p[10] = 1.0;
  info.p[11] = 0.0;

  // R matrix (3x3) - identity
  info.r[0] = 1.0;
  info.r[4] = 1.0;
  info.r[8] = 1.0;
}

bool V4l2Driver::get_driver_connected_flag()
{
  return driver_ != -1;
}

void V4l2Driver::close_device()
{
  if (driver_ != -1) {
    close(driver_);
    driver_ = -1;
  }
}

void V4l2Driver::print_error()
{
  RCLCPP_ERROR(
    node_->get_logger(), "Error number: %d, description: %s", errno, strerror(errno));
}

}  // namespace antbot_camera
