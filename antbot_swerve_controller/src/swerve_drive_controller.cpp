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

/*
 * Author: Geonhee Lee
 */

#include "antbot_swerve_controller/swerve_drive_controller.hpp"

 #include <memory>
 #include <string>
 #include <vector>
 #include <cmath>
 #include <algorithm>
 #include <limits>
 #include <stdexcept>
 #include <functional>

 #include "controller_interface/helpers.hpp"
 #include "hardware_interface/types/hardware_interface_type_values.hpp"
 #include "lifecycle_msgs/msg/state.hpp"
 #include "rclcpp/logging.hpp"
 #include "rclcpp/parameter.hpp"
 #include "rcl_interfaces/msg/parameter_descriptor.hpp"
 #include "tf2/LinearMath/Quaternion.h"
 #include "tf2/utils.h"
 #include "pluginlib/class_list_macros.hpp"


namespace antbot
{

namespace swerve_drive_controller
{

// Reset function
void reset_controller_reference_msg(
  const std::shared_ptr<geometry_msgs::msg::Twist> & msg)
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}


using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterType;

// helper function to normalize angles to [-pi, pi)
double SwerveDriveController::normalize_angle(double angle_rad)
{
  // Use fmod for potentially better performance and handling edge cases
  double remainder = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (remainder < 0.0) {
    remainder += 2.0 * M_PI;
  }
  return remainder - M_PI;
}

// helper function to calculate the shortest angular distance
double normalize_angle_positive(double angle)
{
  // Use fmod and add 2*PI to handle negative results correctly
  return std::fmod(std::fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double SwerveDriveController::shortest_angular_distance(double from, double to)
{
  // Ensure angles are normalized between 0 and 2*pi for correct subtraction
  double result = normalize_angle_positive(to) - normalize_angle_positive(from);
  // Adjust the result to be in [-pi, pi]
  if (result > M_PI) {
    result -= 2.0 * M_PI;
  } else if (result < -M_PI) {
    result += 2.0 * M_PI;
  }
  return result;
}

// --- Controller Implementation ---
SwerveDriveController::SwerveDriveController()
: controller_interface::ControllerInterface(),
  ref_timeout_{0, 0}
{
  // Ensure sane defaults
  num_modules_ = 0;
}
// *****************************************

CallbackReturn SwerveDriveController::on_init()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Initializing SwerveDriveController");
  try {
    // Initialize parameter listener (auto-declares all parameters from YAML)
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    // Manually declare runtime-configurable parameters for ROS parameter server compatibility
    // Use parameter library values as defaults
    auto_declare<double>("cmd_vel_timeout", params_.antbot_swerve_controller.cmd_vel_timeout);
    auto_declare<bool>(
      "enabled_steering_flip",
      params_.antbot_swerve_controller.enabled_steering_flip);
    auto_declare<double>(
      "steering_alignment_angle_error_threshold",
      params_.antbot_swerve_controller.steering_alignment_angle_error_threshold);
    auto_declare<bool>(
      "enabled_wheel_saturation_scaling",
      params_.antbot_swerve_controller.enabled_wheel_saturation_scaling);
    auto_declare<bool>(
      "enable_direct_joint_commands",
      params_.antbot_swerve_controller.enable_direct_joint_commands);
    auto_declare<double>(
      "direct_joint_command_timeout_sec",
      params_.antbot_swerve_controller.direct_joint_command_timeout_sec);
    auto_declare<std::string>("direct_joint_command_topic", "~/direct_joint_commands");
    auto_declare<double>(
      "realigning_angle_threshold",
      params_.antbot_swerve_controller.realigning_angle_threshold);
    auto_declare<double>(
      "discontinuous_motion_steering_tolerance",
      params_.antbot_swerve_controller.discontinuous_motion_steering_tolerance);
    auto_declare<double>(
      "velocity_deadband",
      params_.antbot_swerve_controller.velocity_deadband);
    auto_declare<bool>(
      "enable_steering_scrub_compensator",
      params_.antbot_swerve_controller.enable_steering_scrub_compensator);
    auto_declare<double>(
      "steering_scrub_compensator_scale_factor",
      params_.antbot_swerve_controller.steering_scrub_compensator_scale_factor);
    auto_declare<bool>("enable_odom_tf", params_.antbot_swerve_controller.enable_odom_tf);

    // Steering/wheel command interface options (loaded early for command_interface_configuration)
    auto_declare<bool>(
      "steering.use_velocity_command",
      params_.antbot_swerve_controller.steering.use_velocity_command);
    auto_declare<bool>(
      "steering.use_acceleration_command",
      params_.antbot_swerve_controller.steering.use_acceleration_command);
    auto_declare<bool>(
      "wheel.use_acceleration_command",
      params_.antbot_swerve_controller.wheel.use_acceleration_command);

    // Set member variables for command_interface_configuration
    // Use get_parameter to read the actual (possibly overridden) values
    get_node()->get_parameter(
      "steering.use_velocity_command", use_steering_velocity_command_);
    get_node()->get_parameter(
      "steering.use_acceleration_command", use_steering_acceleration_command_);
    get_node()->get_parameter(
      "wheel.use_acceleration_command", use_wheel_acceleration_command_);

    RCLCPP_INFO(
      get_node()->get_logger(),
      "[on_init] Command interfaces - steering_vel: %s, steering_acc: %s, wheel_acc: %s",
      use_steering_velocity_command_ ? "true" : "false",
      use_steering_acceleration_command_ ? "true" : "false",
      use_wheel_acceleration_command_ ? "true" : "false");

    // Initialize chassis speeds history
    previous_chassis_speeds_.linear.x = 0.0;
    previous_chassis_speeds_.linear.y = 0.0;
    previous_chassis_speeds_.angular.z = 0.0;
    // Defer sizing until num_modules_ is known in on_configure
    previous_steering_commands_.clear();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Exception during parameter declaration: %s",
      e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Parameter declaration successful");
  return CallbackReturn::SUCCESS;
}

// command_interface_configuration, state_interface_configuration
controller_interface::InterfaceConfiguration
SwerveDriveController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> steering_names;
  std::vector<std::string> wheel_names;
  try {
    // Use get_node() which is available after on_init()
    steering_names = get_node()->get_parameter("steering_joint_names").as_string_array();
    wheel_names = get_node()->get_parameter("wheel_joint_names").as_string_array();
  } catch (const std::exception & e) {
    // Log error but don't crash, configuration might be incomplete
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error reading joint names during command config: %s.",
      e.what());
    // It's safer to return an empty config here, let CM handle missing interfaces later
    return conf;
  }

  if (steering_names.empty() || wheel_names.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Joint names parameters are empty during command config.");
    return conf;
  }
  if (steering_names.size() != wheel_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Steering and wheel joint names parameters must have the same size!");
    return conf;
  }

  // Reserve: steering (position + optional velocity + optional acceleration)
  // + wheel (velocity + optional acceleration)
  size_t steering_interface_count = 1;  // position is always required
  if (use_steering_velocity_command_) {
    ++steering_interface_count;
  }
  if (use_steering_acceleration_command_) {
    ++steering_interface_count;
  }
  size_t wheel_interface_count = 2;  // velocity + acceleration (always claimed)
  conf.names.reserve(
    steering_names.size() * steering_interface_count +
    wheel_names.size() * wheel_interface_count);
  for (const auto & joint_name : steering_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_POSITION);
    if (use_steering_velocity_command_) {
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    if (use_steering_acceleration_command_) {
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_ACCELERATION);
    }
  }
  for (const auto & joint_name : wheel_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_ACCELERATION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> steering_names;
  std::vector<std::string> wheel_names;
  try {
    steering_names = get_node()->get_parameter("steering_joint_names").as_string_array();
    wheel_names = get_node()->get_parameter("wheel_joint_names").as_string_array();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error reading joint names during state config: %s.",
      e.what());
    return conf;
  }

  if (steering_names.empty() || wheel_names.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Joint names parameters are empty during state config.");
    return conf;
  }
  if (steering_names.size() != wheel_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Steering and wheel joint names parameters must have the same size!");
    return conf;
  }

  conf.names.reserve(steering_names.size() + wheel_names.size());
  for (const auto & joint_name : steering_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : wheel_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return conf;
}

CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Configuring Swerve Drive Controller...");

  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // Get parameters from hybrid approach:
  // - Structural parameters from parameter library (params_)
  // - Runtime-configurable parameters from ROS parameter server (get_parameter)
  try {
    // Structural parameters: Use ROS parameter if available,
    // otherwise use parameter library defaults
    // This allows YAML overrides while maintaining parameter library structure

    // Helper lambda to get parameter with fallback
    auto get_param_or_default = [this](
      const std::string & param_name, auto default_value) {
        try {
          if (get_node()->has_parameter(param_name)) {
            if constexpr (std::is_same_v<decltype(default_value), std::vector<std::string>>) {
              return get_node()->get_parameter(param_name).as_string_array();
            } else if constexpr (std::is_same_v<decltype(default_value), std::vector<double>>) {
              return get_node()->get_parameter(param_name).as_double_array();
            } else if constexpr (std::is_same_v<decltype(default_value), double>) {
              return get_node()->get_parameter(param_name).as_double();
            } else if constexpr (std::is_same_v<decltype(default_value), std::string>) {
              return get_node()->get_parameter(param_name).as_string();
            } else if constexpr (std::is_same_v<decltype(default_value), bool>) {
              return get_node()->get_parameter(param_name).as_bool();
            }
            RCLCPP_INFO(
              get_node()->get_logger(),
              "Parameter %s has unsupported type, using default.", param_name.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Parameter %s not found or invalid, using default: %s", param_name.c_str(), e.what());
        }
        return default_value;
      };

    steering_joint_names_ = get_param_or_default(
      "steering_joint_names", params_.antbot_swerve_controller.steering_joint_names);
    wheel_joint_names_ = get_param_or_default(
      "wheel_joint_names", params_.antbot_swerve_controller.wheel_joint_names);
    wheel_radius_ = get_param_or_default(
      "wheel_radius", params_.antbot_swerve_controller.wheel_radius);
    module_x_offsets_ = get_param_or_default(
      "module_x_offsets", params_.antbot_swerve_controller.module_x_offsets);
    module_y_offsets_ = get_param_or_default(
      "module_y_offsets", params_.antbot_swerve_controller.module_y_offsets);
    module_angle_offsets_ = get_param_or_default(
      "module_angle_offsets", params_.antbot_swerve_controller.module_angle_offsets);
    steering_to_wheel_y_offsets_ = get_param_or_default(
      "steering_to_wheel_y_offsets",
      params_.antbot_swerve_controller.steering_to_wheel_y_offsets);
    module_steering_limit_lower_ = get_param_or_default(
      "module_steering_limit_lower",
      params_.antbot_swerve_controller.module_steering_limit_lower);
    module_steering_limit_upper_ = get_param_or_default(
      "module_steering_limit_upper",
      params_.antbot_swerve_controller.module_steering_limit_upper);
    module_wheel_speed_limit_lower_ = get_param_or_default(
      "module_wheel_speed_limit_lower",
      params_.antbot_swerve_controller.module_wheel_speed_limit_lower);
    module_wheel_speed_limit_upper_ = get_param_or_default(
      "module_wheel_speed_limit_upper",
      params_.antbot_swerve_controller.module_wheel_speed_limit_upper);

    // Runtime-configurable parameters: Use ROS parameter if available,
    // otherwise use parameter library defaults
    enabled_steering_flip_ = get_param_or_default(
      "enabled_steering_flip", params_.antbot_swerve_controller.enabled_steering_flip);
    enabled_wheel_saturation_scaling_ = get_param_or_default(
      "enabled_wheel_saturation_scaling",
      params_.antbot_swerve_controller.enabled_wheel_saturation_scaling);
    enable_steering_scrub_compensator_ = get_param_or_default(
      "enable_steering_scrub_compensator",
      params_.antbot_swerve_controller.enable_steering_scrub_compensator);
    steering_scrub_compensator_scale_factor_ = get_param_or_default(
      "steering_scrub_compensator_scale_factor",
      params_.antbot_swerve_controller.steering_scrub_compensator_scale_factor);
    non_coaxial_ik_iterations_ = params_.antbot_swerve_controller.non_coaxial_ik_iterations;
    realigning_angle_threshold_ = get_param_or_default(
      "realigning_angle_threshold",
      params_.antbot_swerve_controller.realigning_angle_threshold);
    discontinuous_motion_steering_tolerance_ = get_param_or_default(
      "discontinuous_motion_steering_tolerance",
      params_.antbot_swerve_controller.discontinuous_motion_steering_tolerance);
    velocity_deadband_ = get_param_or_default(
      "velocity_deadband", params_.antbot_swerve_controller.velocity_deadband);
    trajectory_delay_time_ = params_.antbot_swerve_controller.trajectory_delay_time;

    // Steering velocity/acceleration command interface options
    use_steering_velocity_command_ = get_param_or_default(
      "steering.use_velocity_command",
      params_.antbot_swerve_controller.steering.use_velocity_command);
    use_steering_acceleration_command_ = get_param_or_default(
      "steering.use_acceleration_command",
      params_.antbot_swerve_controller.steering.use_acceleration_command);
    RCLCPP_INFO(
      logger, "Steering command interfaces - velocity: %s, acceleration: %s",
      use_steering_velocity_command_ ? "enabled" : "disabled",
      use_steering_acceleration_command_ ? "enabled" : "disabled");

    // Wheel acceleration command interface option
    use_wheel_acceleration_command_ = get_param_or_default(
      "wheel.use_acceleration_command",
      params_.antbot_swerve_controller.wheel.use_acceleration_command);
    RCLCPP_INFO(
      logger, "Wheel command interfaces - acceleration: %s",
      use_wheel_acceleration_command_ ? "enabled" : "disabled");

    steering_alignment_angle_error_threshold_ = get_param_or_default(
      "steering_alignment_angle_error_threshold",
      params_.antbot_swerve_controller.steering_alignment_angle_error_threshold);

    cmd_vel_topic_ = get_param_or_default(
      "cmd_vel_topic", params_.antbot_swerve_controller.cmd_vel_topic);
    use_stamped_cmd_vel_ = params_.antbot_swerve_controller.use_stamped_cmd_vel;
    cmd_vel_timeout_ = get_param_or_default(
      "cmd_vel_timeout", params_.antbot_swerve_controller.cmd_vel_timeout);
    odom_frame_id_ = get_param_or_default(
      "odom_frame_id", params_.antbot_swerve_controller.odom_frame_id);
    base_frame_id_ = get_param_or_default(
      "base_frame_id", params_.antbot_swerve_controller.base_frame_id);
    enable_odom_tf_ = get_param_or_default(
      "enable_odom_tf", params_.antbot_swerve_controller.enable_odom_tf);
    RCLCPP_INFO(
      logger, "[on_configure] enable_odom_tf_ set to: %s",
      enable_odom_tf_ ? "true" : "false");
    pose_covariance_diagonal_ = get_param_or_default(
      "pose_covariance_diagonal",
      params_.antbot_swerve_controller.pose_covariance_diagonal);
    twist_covariance_diagonal_ = get_param_or_default(
      "twist_covariance_diagonal", params_.antbot_swerve_controller.twist_covariance_diagonal);
    odom_solver_method_str_ = get_param_or_default(
      "odom_solver_method", params_.antbot_swerve_controller.odom_solver_method);
    odom_integration_method_str_ = get_param_or_default(
      "odom_integration_method", params_.antbot_swerve_controller.odom_integration_method);
    velocity_rolling_window_size_ =
      params_.antbot_swerve_controller.velocity_rolling_window_size;

    ref_timeout_ = rclcpp::Duration::from_seconds(cmd_vel_timeout_);

    enabled_speed_limits_ = params_.antbot_swerve_controller.enabled_speed_limits;
    publish_limited_velocity_ = params_.antbot_swerve_controller.publish_limited_velocity;
    limiter_linear_x_ = SpeedLimiter(
      params_.antbot_swerve_controller.linear.x.has_velocity_limits,
      params_.antbot_swerve_controller.linear.x.has_acceleration_limits,
      params_.antbot_swerve_controller.linear.x.has_jerk_limits,
      params_.antbot_swerve_controller.linear.x.min_velocity,
      params_.antbot_swerve_controller.linear.x.max_velocity,
      params_.antbot_swerve_controller.linear.x.min_acceleration,
      params_.antbot_swerve_controller.linear.x.max_acceleration,
      params_.antbot_swerve_controller.linear.x.min_jerk,
      params_.antbot_swerve_controller.linear.x.max_jerk);
    limiter_linear_y_ = SpeedLimiter(
      params_.antbot_swerve_controller.linear.y.has_velocity_limits,
      params_.antbot_swerve_controller.linear.y.has_acceleration_limits,
      params_.antbot_swerve_controller.linear.y.has_jerk_limits,
      params_.antbot_swerve_controller.linear.y.min_velocity,
      params_.antbot_swerve_controller.linear.y.max_velocity,
      params_.antbot_swerve_controller.linear.y.min_acceleration,
      params_.antbot_swerve_controller.linear.y.max_acceleration,
      params_.antbot_swerve_controller.linear.y.min_jerk,
      params_.antbot_swerve_controller.linear.y.max_jerk);
    limiter_angular_z_ = SpeedLimiter(
      params_.antbot_swerve_controller.angular.z.has_velocity_limits,
      params_.antbot_swerve_controller.angular.z.has_acceleration_limits,
      params_.antbot_swerve_controller.angular.z.has_jerk_limits,
      params_.antbot_swerve_controller.angular.z.min_velocity,
      params_.antbot_swerve_controller.angular.z.max_velocity,
      params_.antbot_swerve_controller.angular.z.min_acceleration,
      params_.antbot_swerve_controller.angular.z.max_acceleration,
      params_.antbot_swerve_controller.angular.z.min_jerk,
      params_.antbot_swerve_controller.angular.z.max_jerk);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Exception during parameter reading: %s", e.what());
    return CallbackReturn::ERROR;
  }

  num_modules_ = steering_joint_names_.size();

  // --- Parameter Validation ---
  if (steering_joint_names_.size() != num_modules_ || wheel_joint_names_.size() != num_modules_ ||
    module_x_offsets_.size() != num_modules_ || module_y_offsets_.size() != num_modules_ ||
    module_angle_offsets_.size() != num_modules_ ||
    steering_to_wheel_y_offsets_.size() != num_modules_ ||
    module_steering_limit_lower_.size() != num_modules_ ||
    module_steering_limit_upper_.size() != num_modules_ ||
    module_wheel_speed_limit_lower_.size() != num_modules_ ||
    module_wheel_speed_limit_upper_.size() != num_modules_)
  {
    RCLCPP_FATAL(
      logger,
      "Parameter array lengths do not match expected number of modules (%zu). "
      "Check YAML configuration.",
      num_modules_);
    RCLCPP_FATAL(
      logger,
      "steering_joint_names: %zu, wheel_joint_names: %zu,"
      " module_x_offsets: %zu, module_y_offsets: %zu, module_angle_offsets: %zu,"
      " module_steering_limit_lower: %zu, module_steering_limit_upper: %zu, "
      "module_wheel_speed_limit_lower: %zu, module_wheel_speed_limit_upper: %zu, "
      "steering_to_wheel_y_offsets: %zu",
      steering_joint_names_.size(), wheel_joint_names_.size(),
      module_x_offsets_.size(), module_y_offsets_.size(),
      module_angle_offsets_.size(),
      module_steering_limit_lower_.size(),
      module_steering_limit_upper_.size(),
      module_wheel_speed_limit_lower_.size(), module_wheel_speed_limit_upper_.size(),
      steering_to_wheel_y_offsets_.size());
    return CallbackReturn::ERROR;
  }
  if (wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(logger, "'wheel_radius' must be positive.");
    return CallbackReturn::ERROR;
  }

  // --- Setup Subscriber ---
  cmd_vel_subscriber_ = get_node()->create_subscription<CmdVelMsg>(
    cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&SwerveDriveController::reference_callback, this, std::placeholders::_1)
  );

  // Initialize the buffer
  auto initial_cmd = std::make_shared<CmdVelMsg>();
  reset_controller_reference_msg(initial_cmd);
  cmd_vel_buffer_.initRT(initial_cmd);
  last_cmd_vel_time_ = get_node()->now();

  RCLCPP_DEBUG(
    logger, "Subscribed to %s (expecting geometry_msgs/msg/Twist)",
    cmd_vel_topic_.c_str());

  // Publisher
  try {
    odom_s_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<OdomStatePublisher>(odom_s_publisher_);

    // Initialize publisher for visualizing Trajectory in swerve motion planning
    trajectory_publisher_ = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/planned_trajectory", rclcpp::SystemDefaultsQoS());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Exception during publisher creation: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger, "odom_s_publisher_ created successfully");

  // swerve motion planning
  std::vector<Point> module_positions(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    module_positions[i] = {module_x_offsets_[i], module_y_offsets_[i]};
  }

  // Create a list of joint names for the JointTrajectory message (order is important!)
  std::vector<std::string> joint_names;
  // First, add the steering joint names in order
  for (const auto & name : steering_joint_names_) {
    joint_names.push_back(name);
  }
  // Then, add the driving joint names in order
  for (const auto & name : wheel_joint_names_) {
    joint_names.push_back(name);
  }

  // Pass settings to the Planner
  // Declare and get steering/wheel parameters for easy yaml configuration
  if (!get_node()->has_parameter("steering.max_velocity")) {
    get_node()->declare_parameter(
      "steering.max_velocity",
      params_.antbot_swerve_controller.steering.max_velocity);
  }
  if (!get_node()->has_parameter("steering.max_acceleration")) {
    get_node()->declare_parameter(
      "steering.max_acceleration",
      params_.antbot_swerve_controller.steering.max_acceleration);
  }
  if (!get_node()->has_parameter("wheel.max_acceleration")) {
    get_node()->declare_parameter(
      "wheel.max_acceleration",
      params_.antbot_swerve_controller.wheel.max_acceleration);
  }

  double steering_max_velocity = params_.antbot_swerve_controller.steering.max_velocity;
  double steering_max_acceleration =
    params_.antbot_swerve_controller.steering.max_acceleration;
  double wheel_max_acceleration = params_.antbot_swerve_controller.wheel.max_acceleration;

  get_node()->get_parameter_or(
    "steering.max_velocity", steering_max_velocity,
    params_.antbot_swerve_controller.steering.max_velocity);
  get_node()->get_parameter_or(
    "steering.max_acceleration", steering_max_acceleration,
    params_.antbot_swerve_controller.steering.max_acceleration);
  get_node()->get_parameter_or(
    "wheel.max_acceleration", wheel_max_acceleration,
    params_.antbot_swerve_controller.wheel.max_acceleration);
  params_.antbot_swerve_controller.steering.max_velocity = steering_max_velocity;
  params_.antbot_swerve_controller.steering.max_acceleration = steering_max_acceleration;
  params_.antbot_swerve_controller.wheel.max_acceleration = wheel_max_acceleration;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[Steering Profile Config] max_velocity=%.3f, max_acceleration=%.3f, wheel_max_acc=%.3f",
    steering_max_velocity, steering_max_acceleration, wheel_max_acceleration);

  motion_planner_.configure(
    num_modules_, wheel_radius_, module_positions, steering_to_wheel_y_offsets_,
    module_angle_offsets_, joint_names,
    steering_max_velocity,
    steering_max_acceleration,
    wheel_max_acceleration,
    module_steering_limit_lower_, module_steering_limit_upper_,
    non_coaxial_ik_iterations_, realigning_angle_threshold_,
    discontinuous_motion_steering_tolerance_, velocity_deadband_);


  // Initialize state variables
  last_received_cmd_vel_ = geometry_msgs::msg::Twist();  // Initialize to zero
  has_new_command_ = true;  // Set to true to process the first command at controller start
  trajectory_start_time_ = get_node()->now();  // Initialize with the current time

  // initialize odometry and set parameters
  try {
    odometry_.init(get_node()->now());
    odometry_.setModuleParams(
      module_x_offsets_, module_y_offsets_, steering_to_wheel_y_offsets_,
      wheel_radius_);
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);

    OdomSolverMethod solver_method_enum = OdomSolverMethod::SVD;
    if (odom_solver_method_str_ == "pseudo_inverse") {
      solver_method_enum = OdomSolverMethod::PSEUDO_INVERSE;
    } else if (odom_solver_method_str_ == "qr") {
      solver_method_enum = OdomSolverMethod::QR_DECOMPOSITION;
    } else if (odom_solver_method_str_ == "svd") {
      solver_method_enum = OdomSolverMethod::SVD;
    } else {
      RCLCPP_WARN(
        logger,
        "Invalid 'odom_solver_method' parameter: %s. Using SVD by default.",
        odom_solver_method_str_.c_str());
    }
    // Set the solver method
    odometry_.setSolverMethod(solver_method_enum);

    OdomIntegrationMethod integration_method_enum = OdomIntegrationMethod::EULER_METHOD;
    if (odom_integration_method_str_ == "euler") {
      integration_method_enum = OdomIntegrationMethod::EULER_METHOD;
    } else if (odom_integration_method_str_ == "rk2") {
      integration_method_enum = OdomIntegrationMethod::RK2_METHOD;
    } else if (odom_integration_method_str_ == "rk4") {
      integration_method_enum = OdomIntegrationMethod::RK4_METHOD;
    } else if (odom_integration_method_str_ == "analytic_swerve") {
      integration_method_enum = OdomIntegrationMethod::ANALYTIC_SWERVE_METHOD;
    } else {
      RCLCPP_WARN(
        logger,
        "Invalid 'odom_solver_method' parameter: %s. Using EULER_METHOD by default.",
        odom_integration_method_str_.c_str());
    }
    odometry_.setIntegrationMethod(integration_method_enum);

    // Direct Joint Commands
    enable_direct_joint_commands_ =
      get_node()->get_parameter("enable_direct_joint_commands").as_bool();
    direct_joint_command_topic_ =
      get_node()->get_parameter("direct_joint_command_topic").as_string();
    direct_joint_command_timeout_sec_ =
      get_node()->get_parameter("direct_joint_command_timeout_sec").as_double();

    direct_joint_cmd_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      direct_joint_command_topic_,
      rclcpp::SystemDefaultsQoS(),      // Use appropriate QoS
      std::bind(
        &SwerveDriveController::direct_joint_command_callback, this,
        std::placeholders::_1)
    );
    direct_joint_cmd_buffer_.initRT(nullptr);    // Initialize buffer with null
    last_direct_joint_cmd_time_ = get_node()->now() -
      rclcpp::Duration::from_seconds(direct_joint_command_timeout_sec_ + 1.0);
    // Start in timed-out state
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Direct joint command interface enabled on topic: %s",
      direct_joint_command_topic_.c_str());

    if (num_modules_ > 0) {
      direct_steering_cmd_targets_.resize(num_modules_, 0.0);
      direct_wheel_vel_cmd_targets_.resize(num_modules_, 0.0);
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Error initializing odometry: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger, "odometry_ created successfully");

  // --- Realtime Odom State Publisher ---
  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = odom_frame_id_;
  rt_odom_state_publisher_->msg_.child_frame_id = base_frame_id_;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

  constexpr size_t NUM_DIMENSIONS = 6;
  auto & pose_covariance = rt_odom_state_publisher_->msg_.pose.covariance;
  auto & twist_covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  for (size_t index = 0; index < NUM_DIMENSIONS; ++index) {
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    pose_covariance[diagonal_index] = pose_covariance_diagonal_[index];
    twist_covariance[diagonal_index] = twist_covariance_diagonal_[index];
  }

  rt_odom_state_publisher_->unlock();

  // --- TF State Publisher ---
  try {
    // Tf State publisher
    tf_odom_s_publisher_ =
      get_node()->create_publisher<TfStateMsg>("/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<TfStatePublisher>(tf_odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = base_frame_id_;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  RCLCPP_DEBUG(logger, "Subscribed to %s", cmd_vel_topic_.c_str());
  RCLCPP_DEBUG(logger, "Publishing odometry to ~/odometry");

  // ***** joint commander publisher *****
  commanded_joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "joint_commanders", rclcpp::SystemDefaultsQoS());
  rt_commanded_joint_state_publisher_ = std::make_unique<CommandedJointStatePublisher>(
    commanded_joint_state_publisher_);
  RCLCPP_DEBUG(logger, "Publishing joint commands to /joint_commanders");
  // ***** realtime joint commander publisher *****
  if (rt_commanded_joint_state_publisher_) {
    rt_commanded_joint_state_publisher_->lock();
    auto & msg = rt_commanded_joint_state_publisher_->msg_;
    msg.name.reserve(num_modules_ * 2);
    msg.position.resize(num_modules_ * 2, std::numeric_limits<double>::quiet_NaN());
    msg.velocity.resize(num_modules_ * 2, std::numeric_limits<double>::quiet_NaN());
    // enroll the joint names
    for (size_t i = 0; i < num_modules_; ++i) {
      msg.name.push_back(steering_joint_names_[i]);
    }
    for (size_t i = 0; i < num_modules_; ++i) {
      msg.name.push_back(wheel_joint_names_[i]);
    }
    rt_commanded_joint_state_publisher_->unlock();
  }

  // ---publish_limited_velocity  ---
  if (publish_limited_velocity_) {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>("limited_cmd_vel", rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }
  const Twist empty_twist;
  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);

  wheel_saturation_scale_factor_ = 1.0;

  RCLCPP_DEBUG(logger, "Configuration successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Activating Swerve Drive Controller...");

  // Reset internal state variables
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_wz_ = 0.0;
  last_cmd_vel_time_ = get_node()->now();
  is_halted_ = false;

  // --- Get and organize hardware interface handles ---
  module_handles_.clear();
  module_handles_.reserve(num_modules_);

  RCLCPP_DEBUG(get_node()->get_logger(), "Attempting to claim %zu modules.", num_modules_);
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Available command interfaces (%zu):", command_interfaces_.size());
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Available state interfaces (%zu):",
    state_interfaces_.size());

  for (size_t i = 0; i < num_modules_; ++i) {
    // ***** find the steering and wheel joint names according to the module index *****
    const auto & steering_joint = steering_joint_names_[i];
    const auto & wheel_joint = wheel_joint_names_[i];
    RCLCPP_DEBUG(
      get_node()->get_logger(), "Processing module %zu:"
      " Expected Steering='%s', Expected Wheel='%s'",
      i, steering_joint.c_str(), wheel_joint.c_str());
    // **********************************

    // --- Find state interface for Steering Position
    const hardware_interface::LoanedStateInterface * steering_state_pos_ptr = nullptr;
    const std::string expected_steering_state_name = steering_joint;
    const std::string expected_steering_state_if_name = HW_IF_POSITION;

    // -- Find the state interface for wheel velocity state
    const hardware_interface::LoanedStateInterface * wheel_state_vel_ptr = nullptr;

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Searching for State Interface: Joint='%s', Type='%s'",
      expected_steering_state_name.c_str(), expected_steering_state_if_name.c_str());
    bool state_found = false;
    const std::string expected_position_state_name = steering_joint_names_[i] + "/" +
      HW_IF_POSITION;
    const std::string expected_speed_state_name = wheel_joint_names_[i] + "/" + HW_IF_VELOCITY;

    // Find the state interface for steering joint state
    // w.r.t position and wheel joint state w.r.t joint velocity
    for (const auto & state_if : state_interfaces_) {
      if (state_if.get_name() == expected_position_state_name) {
        steering_state_pos_ptr = &state_if;
        state_found = true;
      }
      if (state_if.get_name() == expected_speed_state_name) {
        wheel_state_vel_ptr = &state_if;
        state_found = true;
      }
    }
    if (!state_found) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "State interface '%s' not found in state_interfaces_ list during update.",
        expected_position_state_name.c_str());
      return CallbackReturn::ERROR;
    }

    // --- Find command interface (Steering Position) ---
    hardware_interface::LoanedCommandInterface * steering_cmd_pos_ptr = nullptr;
    const std::string expected_steering_cmd_name = steering_joint_names_[i] + "/" + HW_IF_POSITION;
    const std::string expected_steering_cmd_if_name = HW_IF_POSITION;

    RCLCPP_DEBUG(
      get_node()->get_logger(), "  Searching for Command Interface: Joint='%s', Type='%s'",
      expected_steering_cmd_name.c_str(), expected_steering_cmd_if_name.c_str());

    bool cmd_steering_found = false;
    for (auto & cmd_if : command_interfaces_) {
      // compare the name of the command interface with the expected name
      if (cmd_if.get_name() == expected_steering_cmd_name) {
        steering_cmd_pos_ptr = &cmd_if;
        cmd_steering_found = true;
        break;
      }
    }
    if (!cmd_steering_found) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Command interface '%s' not found in command_interfaces_ list during update.",
        expected_steering_cmd_name.c_str());
      return CallbackReturn::ERROR;
    }

    // --- Find command interface (Steering Velocity) - Optional ---
    hardware_interface::LoanedCommandInterface * steering_cmd_vel_ptr = nullptr;
    if (use_steering_velocity_command_) {
      const std::string expected_steering_vel_cmd_name =
        steering_joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;

      for (auto & cmd_if : command_interfaces_) {
        if (cmd_if.get_name() == expected_steering_vel_cmd_name) {
          steering_cmd_vel_ptr = &cmd_if;
          RCLCPP_INFO(
            get_node()->get_logger(),
            "Found steering velocity command interface for module %zu", i);
          break;
        }
      }
      if (steering_cmd_vel_ptr == nullptr) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Steering velocity command interface not found for module %zu, disabling", i);
      }
    }

    // --- Find command interface (Steering Acceleration) - Optional ---
    hardware_interface::LoanedCommandInterface * steering_cmd_acc_ptr = nullptr;
    if (use_steering_acceleration_command_) {
      const std::string expected_steering_acc_cmd_name =
        steering_joint_names_[i] + "/" + hardware_interface::HW_IF_ACCELERATION;

      for (auto & cmd_if : command_interfaces_) {
        if (cmd_if.get_name() == expected_steering_acc_cmd_name) {
          steering_cmd_acc_ptr = &cmd_if;
          RCLCPP_INFO(
            get_node()->get_logger(),
            "Found steering acceleration command interface for module %zu", i);
          break;
        }
      }
      if (steering_cmd_acc_ptr == nullptr) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Steering acceleration command interface not found for module %zu, disabling", i);
      }
    }

    // --- Find command interface (Wheel Velocity) ---
    hardware_interface::LoanedCommandInterface * wheel_cmd_vel_ptr = nullptr;
    const std::string expected_wheel_cmd_name = wheel_joint_names_[i] + "/" + HW_IF_VELOCITY;
    const std::string expected_wheel_cmd_if_name = HW_IF_VELOCITY;

    // Find the command interface for wheel velocity
    bool cmd_wheel_found = false;
    RCLCPP_DEBUG(
      get_node()->get_logger(), "  Searching for Command Interface: Joint='%s', Type='%s'",
      expected_wheel_cmd_name.c_str(), expected_wheel_cmd_if_name.c_str());

    for (auto & cmd_if : command_interfaces_) {
      // compare the name of the command interface with the expected name
      if (cmd_if.get_name() == expected_wheel_cmd_name) {
        wheel_cmd_vel_ptr = &cmd_if;
        cmd_wheel_found = true;
        break;
      }
    }

    if (!cmd_wheel_found) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Command interface '%s' not found in command_interfaces_ list during activation.",
        expected_wheel_cmd_name.c_str());
      module_handles_.clear();
      return CallbackReturn::ERROR;
    }

    // --- Find command interface (Wheel Acceleration - optional) ---
    hardware_interface::LoanedCommandInterface * wheel_cmd_acc_ptr = nullptr;
    {
      const std::string expected_wheel_acc_cmd_name =
        wheel_joint_names_[i] + "/" + hardware_interface::HW_IF_ACCELERATION;
      for (auto & cmd_if : command_interfaces_) {
        if (cmd_if.get_name() == expected_wheel_acc_cmd_name) {
          wheel_cmd_acc_ptr = &cmd_if;
          RCLCPP_INFO(
            get_node()->get_logger(),
            "Found wheel acceleration command interface for module %zu", i);
          break;
        }
      }
      if (wheel_cmd_acc_ptr == nullptr) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Wheel acceleration command interface not found for module %zu", i);
      }
    }

    // --- Add found handles and params to module_handles_ vector ---
    try {
      module_handles_.emplace_back(
        ModuleHandles{
            std::cref(*steering_state_pos_ptr),
            std::ref(*steering_cmd_pos_ptr),
            steering_cmd_vel_ptr,
            steering_cmd_acc_ptr,
            std::cref(*wheel_state_vel_ptr),
            std::ref(*wheel_cmd_vel_ptr),
            wheel_cmd_acc_ptr,
            module_x_offsets_[i],
            module_y_offsets_[i],
            module_angle_offsets_[i],
            module_steering_limit_lower_[i],
            module_steering_limit_upper_[i]
          });
      RCLCPP_DEBUG(
        get_node()->get_logger(), "Successfully processed interfaces for module %zu.",
        i);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Exception while adding module handles for module %zu: %s", i,
        e.what());
      module_handles_.clear();
      return CallbackReturn::ERROR;
    }
  }
  // End of module loop

  // Direct Joint Commands
  direct_joint_cmd_buffer_.reset();    // Clear the buffer
  // It might be good to initialize direct_cmd_targets_ with the current joint state
  for (size_t i = 0; i < num_modules_; ++i) {
    if (!module_handles_.empty() && i < module_handles_.size()) {
      try {
        direct_steering_cmd_targets_[i] = module_handles_[i].steering_state_pos.get().get_value();
        direct_wheel_vel_cmd_targets_[i] = 0.0;          // Wheel speed is 0 at the start
      } catch (...) {
      }
    }
  }

  // ... (Final check and Activation successful log) ...
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivating Swerve Drive Controller...");
  // Stop the robot
  std::vector<double> zero_steering_cmds(num_modules_, 0.0);
  std::vector<double> zero_wheel_cmds(num_modules_, 0.0);
  for (size_t i = 0; i < num_modules_; ++i) {
    try {
      // Use module_handles if available and initialized correctly
      if (!module_handles_.empty() && i < module_handles_.size()) {
        // Optionally set steering to current pos
        double current_pos = module_handles_[i].steering_state_pos.get().get_value();
        zero_steering_cmds[i] = current_pos;
      } else {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Module handles not available during deactivation for index %zu."
          " Attempting direct interface access.",
          i);
        // Fallback (less safe)
        for (auto & iface : command_interfaces_) {
          // Check if wheel_joint_names_ is still valid and index is within bounds
          if (i < wheel_joint_names_.size() && iface.get_name() == wheel_joint_names_[i] &&
            iface.get_interface_name() == HW_IF_VELOCITY)
          {
            iface.set_value(0.0);
            break;
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Error setting command interface values during deactivation for module %zu: %s", i,
        e.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unknown error setting command interface values during deactivation for module %zu",
        i);
    }
  }
  command_steerings_and_wheels(zero_steering_cmds, zero_wheel_cmds);
  module_handles_.clear();
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivation successful");
  return CallbackReturn::SUCCESS;
}

void SwerveDriveController::direct_joint_control(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  // Direct Joint Commands
  std::shared_ptr<sensor_msgs::msg::JointState> current_direct_joint_cmd_ptr = nullptr;

  current_direct_joint_cmd_ptr = *direct_joint_cmd_buffer_.readFromRT();
  bool direct_cmd_timeout = (time - last_direct_joint_cmd_time_).seconds() >
    direct_joint_command_timeout_sec_;

  if (!direct_cmd_timeout) {
    // read the last joint command
    if (current_direct_joint_cmd_ptr) {
      for (size_t i = 0; i < current_direct_joint_cmd_ptr->name.size(); ++i) {
        for (size_t j = 0; j < num_modules_; ++j) {
          if (current_direct_joint_cmd_ptr->name[i] == steering_joint_names_[j]) {
            if (i < current_direct_joint_cmd_ptr->position.size()) {
              direct_steering_cmd_targets_[j] = current_direct_joint_cmd_ptr->position[i];
            }
          } else if (current_direct_joint_cmd_ptr->name[i] == wheel_joint_names_[j]) {
            if (i < current_direct_joint_cmd_ptr->velocity.size()) {
              direct_wheel_vel_cmd_targets_[j] = current_direct_joint_cmd_ptr->velocity[i];
            }
          }
        }
      }
    }
  } else if (direct_cmd_timeout) {
    direct_joint_active_ = false;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Direct joint command timed out. Reverting to /cmd_vel if available.");
  }

  // Update the final commands
  command_steerings_and_wheels(direct_steering_cmd_targets_, direct_wheel_vel_cmd_targets_);

  // update odometry
  if (!odometry_.update(direct_steering_cmd_targets_, direct_wheel_vel_cmd_targets_, dt)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "Odometry update failed, dt might be too small (%.6f).",
      dt);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getYaw());
  if (rt_odom_state_publisher_ && rt_odom_state_publisher_->trylock()) {
    rt_odom_state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.getX();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.getY();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.getVx();
    rt_odom_state_publisher_->msg_.twist.twist.linear.y = odometry_.getVy();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.getWz();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 2000,
    "[direct_joint_control] enable_odom_tf_: %s, rt_tf_odom_state_publisher_ valid: %s",
    enable_odom_tf_ ? "true" : "false",
    rt_tf_odom_state_publisher_ ? "true" : "false");
  if (enable_odom_tf_ && rt_tf_odom_state_publisher_->trylock()) {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp =
      get_node()->get_clock()->now();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x = odometry_.getX();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y = odometry_.getY();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Check for runtime parameter updates and apply them (hybrid approach)
  if (param_listener_->is_old(params_)) {
    auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "Detected parameter changes, updating runtime parameters...");

    try {
      // Get updated structural parameters from parameter library (if needed)
      params_ = param_listener_->get_params();

      // Update runtime-changeable parameters from ROS parameter server
      double timeout_sec = get_node()->get_parameter("cmd_vel_timeout").as_double();
      ref_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);

      enabled_steering_flip_ = get_node()->get_parameter("enabled_steering_flip").as_bool();
      steering_alignment_angle_error_threshold_ = get_node()->get_parameter(
        "steering_alignment_angle_error_threshold").as_double();
      enabled_wheel_saturation_scaling_ = get_node()->get_parameter(
        "enabled_wheel_saturation_scaling").as_bool();

      // Update direct joint command settings
      enable_direct_joint_commands_ =
        get_node()->get_parameter("enable_direct_joint_commands").as_bool();
      direct_joint_command_timeout_sec_ = get_node()->get_parameter(
        "direct_joint_command_timeout_sec").as_double();

      // Update steering and velocity limits and tolerances
      realigning_angle_threshold_ =
        get_node()->get_parameter("realigning_angle_threshold").as_double();
      discontinuous_motion_steering_tolerance_ = get_node()->get_parameter(
        "discontinuous_motion_steering_tolerance").as_double();
      velocity_deadband_ = get_node()->get_parameter("velocity_deadband").as_double();

      // Update steering scrub compensator
      enable_steering_scrub_compensator_ = get_node()->get_parameter(
        "enable_steering_scrub_compensator").as_bool();
      steering_scrub_compensator_scale_factor_ = get_node()->get_parameter(
        "steering_scrub_compensator_scale_factor").as_double();

      // Update odometry settings
      enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Failed to update runtime parameters: %s", e.what());
    }
  }

  // Process cmd_vel: timeout/NaN guards, speed limits, synchronized setpoint
  process_cmd_and_limits(time, period);

  // 1. Read current states (aligned with offsets) once ---
  std::vector<double> current_wheel_velocities;
  std::vector<double> current_steering_positions;
  current_wheel_velocities.reserve(num_modules_);
  current_steering_positions.reserve(num_modules_);

  if (previous_steering_commands_.empty()) {
    previous_steering_commands_.resize(num_modules_);
    for (size_t i = 0; i < num_modules_; ++i) {
      previous_steering_commands_[i] =
        module_handles_[i].steering_state_pos.get().get_value();
    }
  }

  // Use helper to read states
  if (!read_current_module_states(current_steering_positions, current_wheel_velocities)) {
    // If state reading failed, bail out safely this cycle
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to read all module states; skipping command computation this cycle.");
    return controller_interface::return_type::OK;
  }

  // --- Safety Check: Steering limits violation detection & recovery ---
  check_and_recover_steering_limit_violations(current_steering_positions);

  // 2. Synchronized setpoint (optional motion profiling of chassis speeds)
  if (params_.antbot_swerve_controller.enabled_synchronized_setpoint) {
    apply_synchronized_setpoint(current_steering_positions, period);
  }

  // --- 3. calculate the wheel velocities and steering angles based on the inverse kinematics
  // (Synchronized Motion Profile or not) ---
  std::vector<double> final_steering_commands(num_modules_);
  std::vector<double> final_wheel_velocity_commands(num_modules_);

  bool stop_flag = (std::abs(target_vx_) < 1e-4 &&
    std::abs(target_vy_) < 1e-4 && std::abs(target_wz_) < 1e-4);
  // Maintain previous steering angles and set wheel speeds to zero if halted or cmd_vel is zero
  if (enable_direct_joint_commands_ && direct_joint_active_) {
    direct_joint_control(time, period);
    return controller_interface::return_type::OK;
  } else if (is_halted_ || stop_flag) {
    final_steering_commands = previous_steering_commands_;
    final_wheel_velocity_commands.resize(num_modules_, 0.0);
    command_steerings_and_wheels(final_steering_commands, final_wheel_velocity_commands);

  } else {
    // Use synchronized motion profile for swerve control
    run_synchronized_motion_profile(
      time, period, current_steering_positions, current_wheel_velocities,
      final_steering_commands, final_wheel_velocity_commands);
  }   // --- 4. update the odometry ---
  calculate_odometry(time, period, current_steering_positions, current_wheel_velocities);

  // Publish joint commands in order to compare the actual joint states
  if (rt_commanded_joint_state_publisher_ && rt_commanded_joint_state_publisher_->trylock()) {
    auto & msg = rt_commanded_joint_state_publisher_->msg_;
    msg.header.stamp = time;

    for (size_t i = 0; i < num_modules_; ++i) {
      msg.position[i] = final_steering_commands[i];
      msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();

      msg.position[i + num_modules_] = std::numeric_limits<double>::quiet_NaN();
      msg.velocity[i + num_modules_] = final_wheel_velocity_commands[i];
    }
    rt_commanded_joint_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void SwerveDriveController::calculate_odometry(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period,
  const std::vector<double> & current_steering_positions,
  const std::vector<double> & current_wheel_velocities)
{
  const double dt = period.seconds();
  // Compute odometry using feedback from steering and wheel velocities
  if (!odometry_.update(current_steering_positions, current_wheel_velocities, dt)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "Odometry update failed, dt might be too small (%.6f).",
      dt);
  }


  // Cache odometry values for publishing
  current_odometry_.pose.pose.position.x = odometry_.getX();
  current_odometry_.pose.pose.position.y = odometry_.getY();
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getYaw());
  current_odometry_.pose.pose.orientation = tf2::toMsg(orientation);
  current_odometry_.twist.twist.linear.x = odometry_.getVx();
  current_odometry_.twist.twist.linear.y = odometry_.getVy();
  current_odometry_.twist.twist.angular.z = odometry_.getWz();

  // Publish odometry message
  if (rt_odom_state_publisher_ && rt_odom_state_publisher_->trylock()) {
    rt_odom_state_publisher_->msg_ = current_odometry_;
    rt_odom_state_publisher_->msg_.child_frame_id = base_frame_id_;
    rt_odom_state_publisher_->msg_.header.frame_id = odom_frame_id_;
    rt_odom_state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  if (enable_odom_tf_ && rt_tf_odom_state_publisher_ &&
    rt_tf_odom_state_publisher_->trylock())
  {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp =
      get_node()->get_clock()->now();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x =
      current_odometry_.pose.pose.position.x;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y =
      current_odometry_.pose.pose.position.y;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }
}

geometry_msgs::msg::Twist SwerveDriveController::generate_synchronized_setpoint(
  const geometry_msgs::msg::Twist & last_setpoint,
  const geometry_msgs::msg::Twist & desired_setpoint,
  const std::vector<double> & current_steering_angles,
  double dt)
{
  // 1. Calculate target module states from final goal (inverse kinematics)
  std::vector<double> desired_wheel_speeds(num_modules_);
  std::vector<double> desired_steering_angles(num_modules_);

  for (size_t i = 0; i < num_modules_; ++i) {
    const double module_x = module_handles_[i].x_offset;
    const double module_y = module_handles_[i].y_offset;
    const double angle_offset = module_handles_[i].angle_offset;

    double wheel_vel_x = desired_setpoint.linear.x - desired_setpoint.angular.z * module_y;
    double wheel_vel_y = desired_setpoint.linear.y + desired_setpoint.angular.z * module_x;

    desired_wheel_speeds[i] = std::sqrt(wheel_vel_x * wheel_vel_x + wheel_vel_y * wheel_vel_y);
    double target_steering_angle_robot = std::atan2(wheel_vel_y, wheel_vel_x + 1e-9);
    desired_steering_angles[i] = normalize_angle(target_steering_angle_robot - angle_offset);

    // Steering flip logic
    if (enabled_steering_flip_) {
      double angle_diff = shortest_angular_distance(
        current_steering_angles[i],
        desired_steering_angles[i]);
      if (std::abs(angle_diff) > M_PI / 2.0) {
        desired_steering_angles[i] = normalize_angle(desired_steering_angles[i] + M_PI);
        desired_wheel_speeds[i] *= -1.0;
      }
    }
  }

  // 2. Calculate previous module states (for acceleration/jerk limiting)
  std::vector<double> last_wheel_speeds(num_modules_);
  std::vector<double> last_steering_angles(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    const double module_x = module_handles_[i].x_offset;
    const double module_y = module_handles_[i].y_offset;
    const double angle_offset = module_handles_[i].angle_offset;

    double wheel_vel_x = last_setpoint.linear.x - last_setpoint.angular.z * module_y;
    double wheel_vel_y = last_setpoint.linear.y + last_setpoint.angular.z * module_x;

    last_wheel_speeds[i] = std::sqrt(wheel_vel_x * wheel_vel_x + wheel_vel_y * wheel_vel_y);
    double target_steering_angle_robot = std::atan2(wheel_vel_y, wheel_vel_x + 1e-9);
    last_steering_angles[i] = normalize_angle(target_steering_angle_robot - angle_offset);

    // Apply flip logic to previous states as well
    if (enabled_steering_flip_) {
      double angle_diff = shortest_angular_distance(
        current_steering_angles[i],
        last_steering_angles[i]);
      if (std::abs(angle_diff) > M_PI / 2.0) {
        last_steering_angles[i] = normalize_angle(last_steering_angles[i] + M_PI);
        last_wheel_speeds[i] *= -1.0;
      }
    }
  }

  // 3. Calculate scaling factors for all constraints
  double overall_scale_factor = 1.0;

  for (size_t i = 0; i < num_modules_; ++i) {
    // Steering velocity constraint
    double steering_change = shortest_angular_distance(
      last_steering_angles[i],
      desired_steering_angles[i]);
    double required_steering_velocity = std::abs(steering_change) / dt;
    if (required_steering_velocity > params_.antbot_swerve_controller.steering.max_velocity) {
      overall_scale_factor = std::min(
        overall_scale_factor,
        params_.antbot_swerve_controller.steering.max_velocity /
        required_steering_velocity);
    }

    // Driving acceleration constraint
    double speed_change = desired_wheel_speeds[i] - last_wheel_speeds[i];
    double required_drive_acceleration = std::abs(speed_change) / dt;
    if (required_drive_acceleration >
      params_.antbot_swerve_controller.wheel.max_acceleration)
    {
      overall_scale_factor = std::min(
        overall_scale_factor,
        params_.antbot_swerve_controller.wheel.max_acceleration /
        required_drive_acceleration);
    }
  }

  // 4. Generate scaled next chassis speed
  geometry_msgs::msg::Twist next_setpoint;
  next_setpoint.linear.x = last_setpoint.linear.x +
    (desired_setpoint.linear.x - last_setpoint.linear.x) * overall_scale_factor;
  next_setpoint.linear.y = last_setpoint.linear.y +
    (desired_setpoint.linear.y - last_setpoint.linear.y) * overall_scale_factor;
  next_setpoint.angular.z = last_setpoint.angular.z +
    (desired_setpoint.angular.z - last_setpoint.angular.z) * overall_scale_factor;

  return next_setpoint;
}

void SwerveDriveController::reference_callback(const std::shared_ptr<CmdVelMsg> msg)
{
  // Directly store the Twist message shared pointer
  last_cmd_vel_time_ = this->get_node()->now();
  // Only write if not halted (prevents buffer filling while stopped)
  cmd_vel_buffer_.writeFromNonRT(msg);

  RCLCPP_DEBUG(
    get_node()->get_logger(), "Received new command: vx=%.2f, vy=%.2f, wz=%.2f",
    msg->linear.x, msg->linear.y, msg->angular.z);
}

void SwerveDriveController::direct_joint_command_callback(
  const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  if (!enable_direct_joint_commands_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Direct joint command received but enable_direct_joint_commands is false. Ignoring.");
    return;
  }
  direct_joint_active_ = true;

  if (msg->name.size() != msg->position.size() || msg->name.size() != msg->velocity.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000,
      "Received JointState for direct command with mismatched name/position/velocity "
      "array sizes. Ignoring.");
    return;
  }

  auto direct_cmd_ptr = std::make_shared<sensor_msgs::msg::JointState>(*msg);
  direct_joint_cmd_buffer_.writeFromNonRT(direct_cmd_ptr);
  last_direct_joint_cmd_time_ = get_node()->now();

  RCLCPP_DEBUG(get_node()->get_logger(), "Received new direct joint command.");
}

void SwerveDriveController::command_steerings_and_wheels(
  const std::vector<double> & final_steering_commands,
  const std::vector<double> & final_wheel_velocity_commands,
  const std::vector<double> & steering_velocities,
  const std::vector<double> & steering_accelerations)
{
  // Check if velocity/acceleration vectors are provided and valid
  const bool has_velocities = !steering_velocities.empty() &&
    steering_velocities.size() == num_modules_;
  const bool has_accelerations = !steering_accelerations.empty() &&
    steering_accelerations.size() == num_modules_;

  // Use parameter values as default when vectors are empty or invalid
  const double default_velocity = params_.antbot_swerve_controller.steering.max_velocity;
  const double default_acceleration =
    params_.antbot_swerve_controller.steering.max_acceleration;

  // Wheel acceleration: check for emergency stop condition
  // Emergency stop when: (vx, vy, wz) are all near zero, or linear.z < 0
  const bool is_emergency_stop = (target_vz_ < -1e-6);
  const double wheel_acceleration = is_emergency_stop ?
    params_.antbot_swerve_controller.wheel.emergency_stop_acceleration :
    params_.antbot_swerve_controller.wheel.max_acceleration;

  for (size_t i = 0; i < num_modules_; ++i) {
    // Position command (always required)
    module_handles_[i].steering_cmd_pos.get().set_value(final_steering_commands[i]);
    module_handles_[i].wheel_cmd_vel.get().set_value(final_wheel_velocity_commands[i]);

    // Velocity command (optional)
    if (module_handles_[i].steering_cmd_vel != nullptr) {
      const double raw_velocity = has_velocities ? steering_velocities[i] : default_velocity;
      const double steering_velocity = std::min(raw_velocity, default_velocity);
      module_handles_[i].steering_cmd_vel->set_value(steering_velocity);
    }

    // Steering acceleration command (optional)
    if (module_handles_[i].steering_cmd_acc != nullptr) {
      const double raw_acceleration =
        has_accelerations ? steering_accelerations[i] : default_acceleration;
      const double steering_acceleration = std::min(raw_acceleration, default_acceleration);
      module_handles_[i].steering_cmd_acc->set_value(steering_acceleration);
    }

    // Wheel acceleration command (always claimed)
    // When use_wheel_acceleration_command_ is false, send default max_acceleration
    if (module_handles_[i].wheel_cmd_acc != nullptr) {
      if (use_wheel_acceleration_command_) {
        module_handles_[i].wheel_cmd_acc->set_value(wheel_acceleration);
      } else {
        module_handles_[i].wheel_cmd_acc->set_value(
          params_.antbot_swerve_controller.wheel.max_acceleration);
      }
    }
  }
}

void SwerveDriveController::process_cmd_and_limits(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 1) Check cmd_vel timeout and set halt state/targets
  bool timeout = false;
  if (ref_timeout_.seconds() > 0.0 && (time - last_cmd_vel_time_) > ref_timeout_) {
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 50000,
      "time: %.3f, last_cmd_vel_time_: %.3f, ref_timeout_: %.3f",
      time.seconds(), last_cmd_vel_time_.seconds(), ref_timeout_.seconds());
    timeout = true;
  }

  if (timeout) {
    if (!is_halted_) {
      RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "cmd_vel timed out (%.3f s), stopping robot.", cmd_vel_timeout_);
      is_halted_ = true;
    }
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;

    // Reset odometry accumulators on timeout
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Resetting odometry accumulators due to timeout.");
    odometry_.resetAccumulators();
  } else {
    // Read the latest command from realtime buffer
    auto current_cmd_vel_ptr = cmd_vel_buffer_.readFromRT();
    if (current_cmd_vel_ptr && *current_cmd_vel_ptr) {
      const auto & current_cmd_vel = **current_cmd_vel_ptr;
      target_vx_ = current_cmd_vel.linear.x;
      target_vy_ = current_cmd_vel.linear.y;
      target_wz_ = current_cmd_vel.angular.z;
      target_vz_ = current_cmd_vel.linear.z;  // For emergency stop detection
      is_halted_ = false;  // valid command clears halt

      // collision stop policy: if z linear velocity is negative, halt the robot
      if (current_cmd_vel.linear.z < 0.0) {
        command_steerings_and_wheels(
          previous_steering_commands_,
          std::vector<double>(num_modules_, 0.0));
        is_halted_ = true;
        target_vx_ = 0.0;
        target_vy_ = 0.0;
        target_wz_ = 0.0;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Halt command received (linear.z < 0). Stopping robot.");
      }
      // last_cmd_vel_time_ is updated in the subscriber callback
    } else if (is_halted_) {
      // Keep targets at zero when halted
      target_vx_ = 0.0;
      target_vy_ = 0.0;
      target_wz_ = 0.0;
    }
  }

  // 2) NaN guard
  if (std::isnan(target_vx_) || std::isnan(target_vy_) || std::isnan(target_wz_)) {
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Received NaN in target velocity (vx:%.2f, vy:%.2f, wz:%.2f). Setting targets to zero.",
      target_vx_, target_vy_, target_wz_);
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;
    is_halted_ = true;
  }

  // 3) Apply speed limiting (keeps a short history for jerk/accel limiting)
  if (enabled_speed_limits_) {
    // T-1 and T-2 commands from history
    Twist previous_cmd{};
    Twist pprevious_cmd{};

    if (previous_commands_.size() >= 2) {
      previous_cmd = previous_commands_.back();
      pprevious_cmd = previous_commands_.front();
    } else if (previous_commands_.size() == 1) {
      previous_cmd = previous_commands_.back();
      pprevious_cmd = previous_cmd;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Speed limiter: Not enough previous commands in history. "
        "Assuming zero for past velocities.");
    }

    limiter_linear_x_.limit(
      target_vx_, previous_cmd.linear.x, pprevious_cmd.linear.x,
      period.seconds());
    limiter_linear_y_.limit(
      target_vy_, previous_cmd.linear.y, pprevious_cmd.linear.y,
      period.seconds());
    limiter_angular_z_.limit(
      target_wz_, previous_cmd.angular.z, pprevious_cmd.angular.z,
      period.seconds());

    // Maintain history size of 2
    if (previous_commands_.size() >= 2) {
      previous_commands_.pop();
    }
    Twist current_limited_cmd_obj{};
    current_limited_cmd_obj.linear.x = target_vx_;
    current_limited_cmd_obj.linear.y = target_vy_;
    current_limited_cmd_obj.angular.z = target_wz_;
    previous_commands_.emplace(current_limited_cmd_obj);

    if (publish_limited_velocity_ && realtime_limited_velocity_publisher_ &&
      realtime_limited_velocity_publisher_->trylock())
    {
      auto & limited_velocity_msg = realtime_limited_velocity_publisher_->msg_;
      limited_velocity_msg.linear.x = target_vx_;
      limited_velocity_msg.linear.y = target_vy_;
      limited_velocity_msg.angular.z = target_wz_;
      realtime_limited_velocity_publisher_->unlockAndPublish();
    }
  }
}

bool SwerveDriveController::read_current_module_states(
  std::vector<double> & current_steering_positions,
  std::vector<double> & current_wheel_velocities)
{
  for (size_t i = 0; i < num_modules_; ++i) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Reading state for module %zu", i);
    try {
      // Validate module_handles_ before accessing it
      RCLCPP_DEBUG(get_node()->get_logger(), "Module handles size: %zu", module_handles_.size());
      if (module_handles_.empty() || i >= module_handles_.size()) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "Module handles not ready for index %zu in state reading.",
          i);
        return false;
      }

      RCLCPP_DEBUG(
        get_node()->get_logger(), "Steering='%s': %f",
        module_handles_[i].steering_state_pos.get().get_name().c_str(),
        module_handles_[i].steering_state_pos.get().get_value());
      RCLCPP_DEBUG(
        get_node()->get_logger(), " Wheel='%s': %f",
        module_handles_[i].wheel_state_vel.get().get_name().c_str(),
        module_handles_[i].wheel_state_vel.get().get_value());

      double current_steering_position = module_handles_[i].steering_state_pos.get().get_value();
      // Consider the angle offset
      current_steering_positions.push_back(
        current_steering_position + module_handles_[i].angle_offset);
      current_wheel_velocities.push_back(module_handles_[i].wheel_state_vel.get().get_value());
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception reading state for module %zu during odometry update: %s", i, e.what());
      return false;
    }
  }
  return true;
}

void SwerveDriveController::apply_synchronized_setpoint(
  const std::vector<double> & current_steering_positions_robot_frame,
  const rclcpp::Duration & period,
  bool log_info)
{
  // Basic validation
  if (current_steering_positions_robot_frame.size() != num_modules_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "apply_synchronized_setpoint: size mismatch (got %zu, expected %zu). Skipping.",
      current_steering_positions_robot_frame.size(), num_modules_);
    return;
  }

  geometry_msgs::msg::Twist desired_chassis_speeds;
  desired_chassis_speeds.linear.x = target_vx_;
  desired_chassis_speeds.linear.y = target_vy_;
  desired_chassis_speeds.angular.z = target_wz_;

  // Convert robot-frame steering angles (offset applied) to joint-frame angles
  std::vector<double> current_joint_steering_angles(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    current_joint_steering_angles[i] = normalize_angle(
      current_steering_positions_robot_frame[i] - module_handles_[i].angle_offset);
  }

  // Compute synchronized setpoint
  auto synchronized = generate_synchronized_setpoint(
    previous_chassis_speeds_, desired_chassis_speeds, current_joint_steering_angles,
    period.seconds());

  // Update memory of last setpoint
  previous_chassis_speeds_ = synchronized;

  if (log_info) {
    RCLCPP_INFO(
      get_node()->get_logger(), "original: vx=%.2f, vy=%.2f, wz=%.2f",
      target_vx_, target_vy_, target_wz_);
  }

  // Apply synchronized target back to controller targets
  target_vx_ = synchronized.linear.x;
  target_vy_ = synchronized.linear.y;
  target_wz_ = synchronized.angular.z;

  if (log_info) {
    RCLCPP_INFO(
      get_node()->get_logger(), "synchronized: vx=%.2f, vy=%.2f, wz=%.2f",
      target_vx_, target_vy_, target_wz_);
  }

  // NOTE: Extensibility hooks can be added here (e.g., custom weighting, flip policy overrides,
  //       per-module constraints, diagnostics), without changing the update() logic.
}


bool SwerveDriveController::check_and_recover_steering_limit_violations(
  const std::vector<double> & current_steering_positions)
{
  bool steering_limit_violation = false;
  std::vector<size_t> violating_indices;
  for (size_t i = 0; i < num_modules_; ++i) {
    if (i >= module_steering_limit_lower_.size() || i >= module_steering_limit_upper_.size()) {
      continue;  // already validated earlier
    }
    double lower = module_steering_limit_lower_[i];
    double upper = module_steering_limit_upper_[i];
    double angle = current_steering_positions[i];
    bool outside = false;
    if (lower <= upper) {
      outside = (angle<lower || angle> upper);
    } else {  // wrap-around
      outside = (angle > upper && angle < lower);
    }
    if (outside) {
      steering_limit_violation = true;
      violating_indices.push_back(i);
    }
  }
  if (!steering_limit_violation) {
    return false;
  }
  RCLCPP_ERROR(
    get_node()->get_logger(),
    "Steering limit violation detected on %zu module(s). "
    "Forcing safe reset (steering=0, wheels=0).",
    violating_indices.size());
  for (auto idx : violating_indices) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "  Module %zu angle=%.3f outside [%.3f, %.3f]", idx,
      current_steering_positions[idx], module_steering_limit_lower_[idx],
      module_steering_limit_upper_[idx]);
  }
  std::vector<double> reset_steering(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    double lower = module_steering_limit_lower_[i];
    double upper = module_steering_limit_upper_[i];
    if (lower <= upper) {
      reset_steering[i] = std::clamp(0.0, lower, upper);
    } else {
      // wrap-around: check if 0.0 is in the valid range
      if (0.0 > upper && 0.0 < lower) {
        reset_steering[i] = (std::abs(lower) < std::abs(upper)) ? lower : upper;
      } else {
        reset_steering[i] = 0.0;
      }
    }
  }
  std::vector<double> reset_wheels(num_modules_, 0.0);
  command_steerings_and_wheels(reset_steering, reset_wheels);
  previous_steering_commands_ = reset_steering;
  target_vx_ = 0.0; target_vy_ = 0.0; target_wz_ = 0.0;
  last_received_cmd_vel_ = geometry_msgs::msg::Twist();
  return true;
}

void SwerveDriveController::run_synchronized_motion_profile(
  const rclcpp::Time & time,
  const rclcpp::Duration & period,
  const std::vector<double> & current_steering_positions,
  const std::vector<double> & current_wheel_velocities,
  std::vector<double> & final_steering_commands,
  std::vector<double> & final_wheel_velocity_commands)
{
  // --- 1. First, check if the current trajectory has finished ---
  bool plan_is_finished = current_trajectory_.points.empty() ||
    (time - trajectory_start_time_) > rclcpp::Duration(
    current_trajectory_.points.back().time_from_start);

  // --- 2. Check and release re-alignment state ---
  // If it was re-aligning and that trajectory has finished, release the re-alignment state.
  if (is_realigning_ && plan_is_finished) {
    is_realigning_ = false;     // Re-alignment complete! New commands can now be accepted.
    RCLCPP_DEBUG(
      get_node()->get_logger(), "Re-alignment trajectory finished. Accepting new commands.");
  }

  // --- 3. Process new commands and plan trajectory (only runs when not re-aligning) ---
  if (!is_realigning_) {
    // 3.1. Read external target velocity (cmd_vel) and detect changes
    geometry_msgs::msg::Twist limited_cmd_vel;
    limited_cmd_vel.linear.x = target_vx_;
    limited_cmd_vel.linear.y = target_vy_;
    limited_cmd_vel.angular.z = target_wz_;

    double speed_variation = 1e-3;
    if (std::abs(limited_cmd_vel.linear.x - last_received_cmd_vel_.linear.x) > speed_variation ||
      std::abs(limited_cmd_vel.linear.y - last_received_cmd_vel_.linear.y) > speed_variation ||
      std::abs(limited_cmd_vel.angular.z - last_received_cmd_vel_.angular.z) > speed_variation)
    {
      last_received_cmd_vel_ = limited_cmd_vel;
      has_new_command_ = true;
    }

    // 3.2. Plan a new trajectory (if there's a new command or the previous
    // 'normal' trajectory has finished)
    if (has_new_command_ || plan_is_finished) {
      PlannedMotion motion = motion_planner_.plan(
        last_received_cmd_vel_,
        current_steering_positions,
        current_wheel_velocities);

      // [Core] Check the type of the planned motion.
      if (motion.type == RobotState::DISCONTINUOUS_RE_ALIGNMENT) {
        is_realigning_ = true;         // Set re-alignment flag!
        RCLCPP_DEBUG(
          get_node()->get_logger(),
          "Discontinuous re-alignment detected. "
          "Locking new commands until trajectory is complete.");
      }

      current_trajectory_ = motion.trajectory;
      // Save steering profile parameters for command_steerings_and_wheels
      current_steering_peak_velocities_ = motion.steering_peak_velocities;
      current_steering_accelerations_ = motion.steering_accelerations;

      // If a valid trajectory was generated, record the start time and
      // publish a visualization message
      if (!current_trajectory_.points.empty()) {
        has_new_command_ = false;         // Reset flag
        trajectory_start_time_ = time;
        current_trajectory_.header.stamp = time;
        trajectory_publisher_->publish(current_trajectory_);
        RCLCPP_DEBUG(get_node()->get_logger(), "New trajectory received from motion planner.");
      }
    }
  }

  // --- 4. Calculate intermediate target values for the current time (linear interpolation) ---
  if (!current_trajectory_.points.empty()) {
    double delay = trajectory_delay_time_;
    double elapsed_time = (time - trajectory_start_time_).seconds() + delay;

    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Calculating target positions for elapsed time: %.2f seconds, "
      "Final trajectory duration: %.2f seconds",
      elapsed_time,
      rclcpp::Duration(current_trajectory_.points.back().time_from_start).seconds()
    );

    // Find the two appropriate points in the trajectory
    size_t p_prev_idx = 0;
    size_t p_next_idx = 0;
    for (size_t i = 0; i < current_trajectory_.points.size() - 1; ++i) {
      p_prev_idx = i;
      p_next_idx = i + 1;
      if (rclcpp::Duration(current_trajectory_.points[p_next_idx].time_from_start).seconds() >=
        elapsed_time)
      {
        break;
      }
    }

    const auto & p_prev = current_trajectory_.points[p_prev_idx];
    const auto & p_next = current_trajectory_.points[p_next_idx];

    double t_prev = rclcpp::Duration(p_prev.time_from_start).seconds();
    double t_next = rclcpp::Duration(p_next.time_from_start).seconds();
    double segment_duration = t_next - t_prev;

    double alpha = (segment_duration > 1e-6) ? (elapsed_time - t_prev) / segment_duration : 1.0;
    alpha = std::clamp(alpha, 0.0, 1.0);

    // Calculate the target value for the current time with linear interpolation
    for (size_t i = 0; i < num_modules_; ++i) {
      // Steering angle interpolation (same as before, very well implemented)
      double angle_diff = shortest_angular_distance(p_prev.positions[i], p_next.positions[i]);
      final_steering_commands[i] = normalize_angle(p_prev.positions[i] + alpha * angle_diff);

      // Bug fix: Interpolate driving speed from the positions field, not velocities
      final_wheel_velocity_commands[i] = p_prev.positions[i + num_modules_] +
        alpha * (p_next.positions[i + num_modules_] - p_prev.positions[i + num_modules_]);
    }

  } else {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "No valid trajectory points available for interpolation.");
    // If the trajectory is empty, reset
    previous_steering_commands_.assign(num_modules_, 0.0);
    final_steering_commands = previous_steering_commands_;
    std::vector<double> zero_drive_speeds(num_modules_, 0.0);
    final_wheel_velocity_commands = zero_drive_speeds;
  }

  // --- 5. Calculate final command and send to hardware ---
  for (size_t i = 0; i < num_modules_; ++i) {
    // ★★★ 5.2. Calculate and apply steering scrub compensation
    // (Feed-Forward) ★★★
    if (enable_steering_scrub_compensator_) {
      double steering_angle_change = shortest_angular_distance(
        previous_steering_commands_[i],
        final_steering_commands[i]);

      double estimated_steering_velocity =
        steering_angle_change / period.seconds();

      double scrub_linear_velocity = estimated_steering_velocity *
        steering_to_wheel_y_offsets_[i];

      double scrub_compensation_speed_rad_per_sec = scrub_linear_velocity / wheel_radius_;

      bool is_point_turn =
        (std::abs(target_vx_) < 1e-3 && std::abs(target_vy_) < 1e-3 &&
        std::abs(target_wz_) > 1e-3);

      if (is_point_turn) {
        final_wheel_velocity_commands[i] -= scrub_compensation_speed_rad_per_sec;
      } else {
        final_wheel_velocity_commands[i] -= scrub_compensation_speed_rad_per_sec *
          steering_scrub_compensator_scale_factor_;
      }

      RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Module %zu: Steering scrub compensation applied. Estimated Vel: %.4f rad/s",
        i, scrub_compensation_speed_rad_per_sec);
    }
  }

  // Update the final commands for logging and next iteration
  // Use saved steering profile parameters from current trajectory
  command_steerings_and_wheels(
    final_steering_commands, final_wheel_velocity_commands,
    current_steering_peak_velocities_, current_steering_accelerations_);

  // Save the previous steering commands for the next iteration
  previous_steering_commands_ = final_steering_commands;
}

}   // namespace swerve_drive_controller
}   // namespace antbot

// Pluginlib export macro
PLUGINLIB_EXPORT_CLASS(
  antbot::swerve_drive_controller::SwerveDriveController,
  controller_interface::ControllerInterface)
