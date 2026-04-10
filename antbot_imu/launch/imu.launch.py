#!/usr/bin/env python3
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
# Authors: Daun Jeong

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('antbot_imu')
    config_dir = os.path.join(pkg_dir, 'config')

    control_table_path = LaunchConfiguration('control_table_path')
    imu_param = LaunchConfiguration('imu_param')

    declare_control_table = DeclareLaunchArgument(
        'control_table_path',
        default_value=os.path.join(config_dir, 'control_table.xml'),
        description='Full path to IMU control table XML file')

    declare_param = DeclareLaunchArgument(
        'imu_param',
        default_value=os.path.join(config_dir, 'imu.yaml'),
        description='Full path to IMU parameter file')

    imu_node = Node(
        package='antbot_imu',
        executable='imu_node',
        name='imu_node',
        parameters=[
            imu_param,
            {'control_table_path': control_table_path}
        ],
        output='screen')

    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_node,
            on_exit=[Shutdown(reason='IMU node has exited.')]))

    ld = LaunchDescription()
    ld.add_action(declare_control_table)
    ld.add_action(declare_param)
    ld.add_action(imu_node)
    ld.add_action(shutdown_handler)
    return ld
