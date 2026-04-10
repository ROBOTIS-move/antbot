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

    mode = LaunchConfiguration('mode')

    # Resolve config paths based on mode: config/sim/ or config/real/
    config_dir = PathJoinSubstitution([pkg_dir, 'config', mode])
    ekf_params_file = PathJoinSubstitution([config_dir, 'ekf.yaml'])
    slam_params_file = PathJoinSubstitution(
        [config_dir, 'slam_toolbox_params.yaml'])

    # use_sim_time is derived from mode: sim=true, real=false
    use_sim_time = PythonExpression(["'", mode, "' == 'sim'"])

    # Disable swerve controller's odom TF so the EKF publishes it instead.
    # Delayed 3s to ensure the swerve controller is fully active first.
    disable_odom_tf = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'param', 'set',
                 '/antbot_swerve_controller', 'enable_odom_tf', 'false'],
            output='screen')])

    # EKF node for sensor fusion (wheel odom + IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}])

    # SLAM Toolbox (provides map -> odom TF instead of AMCL)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}])

    return LaunchDescription([
        mode_arg,
        disable_odom_tf,
        ekf_node,
        slam_toolbox_node,
    ])
