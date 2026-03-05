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

#ifndef ANTBOT_SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define ANTBOT_SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "antbot_swerve_controller/odometry.hpp"
#include "antbot_swerve_controller/speed_limiter.hpp"
#include "antbot_swerve_controller/swerve_motion_control.hpp"
#include <antbot_swerve_controller/swerve_drive_controller_parameter.hpp>

namespace antbot
{

namespace swerve_drive_controller
{

// Define aliases for convenience
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
// Use Twist directly as the command message type
using CmdVelMsg = geometry_msgs::msg::Twist;
using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using OdomStateMsg = nav_msgs::msg::Odometry;
using TfStateMsg = tf2_msgs::msg::TFMessage;
using OdomStatePublisher = realtime_tools::RealtimePublisher<OdomStateMsg>;
using TfStatePublisher = realtime_tools::RealtimePublisher<TfStateMsg>;

// Structure to hold module information for easier access
struct ModuleHandles
{
  // Use reference_wrapper for safe access in RT loop
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> steering_state_pos;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> steering_cmd_pos;
  // Optional steering velocity and acceleration command interfaces (non-owning)
  hardware_interface::LoanedCommandInterface * steering_cmd_vel = nullptr;
  hardware_interface::LoanedCommandInterface * steering_cmd_acc = nullptr;
  // Wheel state might be needed for odometry (not implemented here)
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> wheel_state_vel;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> wheel_cmd_vel;
  // Optional wheel acceleration command interface (non-owning)
  hardware_interface::LoanedCommandInterface * wheel_cmd_acc = nullptr;
  // Store parameters associated with this module
  double x_offset;
  double y_offset;
  double angle_offset;
  double steering_limit_lower;
  double steering_limit_upper;
};

class SwerveDriveController : public controller_interface::ControllerInterface
{
public:
  SwerveDriveController();

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void reference_callback(const std::shared_ptr<CmdVelMsg> msg);

  static double normalize_angle(double angle_rad);

  static double shortest_angular_distance(double from, double to);

  // Parameters from ROS for swerve drive controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

protected:
  // Parameters (defaults match swerve_drive_controller_parameter.yaml)
  double wheel_radius_{0.103};
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;
  std::vector<double> module_x_offsets_;
  std::vector<double> module_y_offsets_;
  std::vector<double> module_angle_offsets_;
  std::vector<double> steering_to_wheel_y_offsets_;
  std::vector<double> module_steering_limit_lower_;
  std::vector<double> module_steering_limit_upper_;
  std::vector<double> module_wheel_speed_limit_lower_;
  std::vector<double> module_wheel_speed_limit_upper_;

  bool enabled_steering_flip_{true};
  bool enabled_wheel_saturation_scaling_{true};
  bool enable_steering_scrub_compensator_{true};
  double steering_scrub_compensator_scale_factor_{1.0};
  double wheel_saturation_scale_factor_{1.0};
  double steering_alignment_angle_error_threshold_{6.29};
  std::string odom_solver_method_str_{"svd"};
  std::string odom_integration_method_str_{"rk4"};

  int non_coaxial_ik_iterations_{0};
  double realigning_angle_threshold_{0.087266};
  double discontinuous_motion_steering_tolerance_{1.0};
  double velocity_deadband_{0.01};
  double trajectory_delay_time_{0.50};

  // Steering velocity/acceleration command interface options
  bool use_steering_velocity_command_{true};
  bool use_steering_acceleration_command_{true};
  // Wheel acceleration command interface option
  bool use_wheel_acceleration_command_{true};

  std::string cmd_vel_topic_{"/cmd_vel"};
  bool use_stamped_cmd_vel_{false};
  double cmd_vel_timeout_{0.50};

  // Store number of modules
  size_t num_modules_;

  // Realtime buffer for incoming Twist commands
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdVelMsg>> cmd_vel_buffer_;
  // Subscriber
  rclcpp::Subscription<CmdVelMsg>::SharedPtr cmd_vel_subscriber_ = nullptr;

  // Interface handles organized by module
  std::vector<ModuleHandles> module_handles_;

  // Internal state or variables for control logic
  double target_vx_ = 0.0;
  double target_vy_ = 0.0;
  double target_wz_ = 0.0;
  double target_vz_ = 0.0;  // Used for emergency stop flag (linear.z < 0)
  rclcpp::Time last_cmd_vel_time_;
  // Flag to check if stopping due to timeout
  bool is_halted_ = false;


  // ***** Odometry *****
  OdomStateMsg current_odometry_;
  Odometry odometry_;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_s_publisher_ = nullptr;
  std::unique_ptr<OdomStatePublisher> rt_odom_state_publisher_ = nullptr;

  // joint commander publisher
  using CommandedJointStatePublisher =
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commanded_joint_state_publisher_ =
    nullptr;
  std::unique_ptr<CommandedJointStatePublisher> rt_commanded_joint_state_publisher_ = nullptr;

  // Odometry Parameters
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  bool enable_odom_tf_{true};
  std::vector<double> pose_covariance_diagonal_;
  std::vector<double> twist_covariance_diagonal_;
  int velocity_rolling_window_size_{1};

  // TF Broadcaster
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_odom_s_publisher_;
  std::unique_ptr<TfStatePublisher> rt_tf_odom_state_publisher_;

  // speed limiters
  bool enabled_speed_limits_{false};
  bool publish_limited_velocity_{true};
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> limited_velocity_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
  realtime_limited_velocity_publisher_ =
    nullptr;
  std::queue<Twist> previous_commands_;
  std::vector<double> previous_steering_commands_;

  SpeedLimiter limiter_linear_x_;
  SpeedLimiter limiter_linear_y_;
  SpeedLimiter limiter_angular_z_;

  // Direct Joint Commands
  bool enable_direct_joint_commands_{true};
  bool direct_joint_active_{false};  // runtime flag: set by callback, cleared by timeout
  std::string direct_joint_command_topic_{"~/direct_joint_commands"};
  double direct_joint_command_timeout_sec_{5.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr direct_joint_cmd_subscriber_ =
    nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>
  direct_joint_cmd_buffer_;
  std::vector<double> direct_steering_cmd_targets_;
  std::vector<double> direct_wheel_vel_cmd_targets_;
  rclcpp::Time last_direct_joint_cmd_time_;
  void direct_joint_command_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
  void direct_joint_control(const rclcpp::Time & time, const rclcpp::Duration & period);

  // Synchronized Motion Profiling
  geometry_msgs::msg::Twist previous_chassis_speeds_;
  geometry_msgs::msg::Twist generate_synchronized_setpoint(
    const geometry_msgs::msg::Twist & last_setpoint,
    const geometry_msgs::msg::Twist & desired_setpoint,
    const std::vector<double> & current_steering_angles,
    double dt);

  // motion planning
  SwerveMotionControl motion_planner_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  // the currently active trajectory and its planned start time
  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  rclcpp::Time trajectory_start_time_;
  // Steering profile parameters from current trajectory (Global Sync result)
  std::vector<double> current_steering_peak_velocities_;
  std::vector<double> current_steering_accelerations_;

  // new command flag
  geometry_msgs::msg::Twist last_received_cmd_vel_;
  bool has_new_command_{true};
  bool is_realigning_ = false;
  void command_steerings_and_wheels(
    const std::vector<double> & final_steering_commands,
    const std::vector<double> & final_wheel_velocity_commands,
    const std::vector<double> & steering_velocities = {},
    const std::vector<double> & steering_accelerations = {});

  // Odometry
  void calculate_odometry(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    const std::vector<double> & current_steering_positions,
    const std::vector<double> & current_wheel_velocities);

private:
  // Helper: process timeout/NaN, apply speed limits, and synchronized setpoint
  void process_cmd_and_limits(const rclcpp::Time & time, const rclcpp::Duration & period);

  bool read_current_module_states(
    std::vector<double> & current_steering_positions,
    std::vector<double> & current_wheel_velocities);

  void apply_synchronized_setpoint(
    const std::vector<double> & current_steering_positions_robot_frame,
    const rclcpp::Duration & period,
    bool log_info = false);

  bool check_and_recover_steering_limit_violations(
    const std::vector<double> & current_steering_positions);

  void run_synchronized_motion_profile(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    const std::vector<double> & current_steering_positions,
    const std::vector<double> & current_wheel_velocities,
    std::vector<double> & final_steering_commands,
    std::vector<double> & final_wheel_velocity_commands);

  rclcpp::Duration ref_timeout_;
};

// Utility function prototype (global scope)
void reset_controller_reference_msg(
  const std::shared_ptr<geometry_msgs::msg::Twist> & msg);
}  // namespace swerve_drive_controller

}  // namespace antbot

#endif  // ANTBOT_SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
