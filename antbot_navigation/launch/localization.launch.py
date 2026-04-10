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

    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to the map YAML file')

    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')

    # Resolve config paths based on mode
    config_dir = PathJoinSubstitution([pkg_dir, 'config', mode])
    nav2_params_file = PathJoinSubstitution([config_dir, 'nav2_params.yaml'])
    ekf_params_file = PathJoinSubstitution([config_dir, 'ekf.yaml'])

    use_sim_time = PythonExpression(["'", mode, "' == 'sim'"])

    # Disable swerve controller's odom TF so the EKF publishes it instead.
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

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file,
                    {'yaml_filename': map_yaml,
                     'use_sim_time': use_sim_time}])

    # AMCL localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Lifecycle manager for localization nodes
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Delay localization nodes to allow EKF to start publishing TF
    delayed_localization = TimerAction(
        period=2.0,
        actions=[
            map_server_node,
            amcl_node,
            lifecycle_manager_localization,
        ])

    return LaunchDescription([
        mode_arg,
        map_arg,
        disable_odom_tf,
        ekf_node,
        delayed_localization,
    ])
