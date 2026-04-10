# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Jaehong Oh

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('antbot_navigation')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        choices=['sim', 'real'],
        description='Operating mode: sim (Gazebo) or real (physical robot)')

    odom_integration_method_arg = DeclareLaunchArgument(
        'odom_integration_method',
        default_value='rk4',
        choices=['euler', 'rk2', 'rk4', 'analytic_swerve'],
        description='Odometry integration method: euler, rk2, rk4, or analytic_swerve')

    mode = LaunchConfiguration('mode')
    odom_integration_method = LaunchConfiguration('odom_integration_method')

    # Resolve config paths based on mode: config/sim/ or config/real/
    config_dir = PathJoinSubstitution([pkg_dir, 'config', mode])
    ekf_params_file = PathJoinSubstitution([config_dir, 'ekf.yaml'])
    slam_params_file = PathJoinSubstitution(
        [config_dir, 'slam_toolbox_params.yaml'])

    # use_sim_time is derived from mode: sim=true, real=false
    use_sim_time = PythonExpression(["'", mode, "' == 'sim'"])

    # Set odometry integration method for swerve controller
    # Delayed 2s to ensure the swerve controller is fully loaded first.
    set_odom_integration_method = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'param', 'set',
                 '/antbot_swerve_controller', 'odom_integration_method',
                 odom_integration_method],
            output='screen')])

    # =========================================================================
    # REAL MODE ARCHITECTURE (Simplified):
    # - Odometry: Swerve controller only (no EKF, no IMU fusion)
    # - TF: Swerve controller publishes odom→base_link directly (enable_odom_tf=true by default)
    # - LiDAR: Front LiDAR (/scan_0) only (no merger, no back LiDAR)
    #
    # This configuration prioritizes stability and simplicity over sensor fusion.
    # =========================================================================

    # Scan fix relay (real mode only)
    # COIN D4 driver publishes angle_increment for N points but only N-1 ranges.
    # This relay recalculates metadata to match the actual ranges count.
    scan_fix_relay_node = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
        package='antbot_navigation',
        executable='scan_fix_relay.py',
        name='scan_fix_relay',
        output='screen',
        parameters=[{
            'input_topic': '/scan_0',
            'output_topic': '/scan_0_fixed',
        }])

    # SLAM Toolbox
    # - In real mode: subscribes to /scan_0_fixed (metadata-corrected)
    # - In sim mode: subscribes to /scan_0 directly
    # - Publishes map→odom TF
    # - Performs online SLAM with loop closure
    scan_topic = PythonExpression([
        "'/scan_0_fixed' if '", mode, "' == 'real' else '/scan_0'"])
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
        }])

    return LaunchDescription([
        mode_arg,
        odom_integration_method_arg,
        set_odom_integration_method,  # Sets RK4 integration after 2s
        scan_fix_relay_node,
        slam_toolbox_node,
    ])
