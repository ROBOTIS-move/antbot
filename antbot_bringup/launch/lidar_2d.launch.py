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
# Author: Daun Jeong

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    coin_d4_driver_dir = get_package_share_directory('coin_d4_driver')
    default_params = os.path.join(
        coin_d4_driver_dir, 'params', 'multi_lidar_node.yaml')

    multi_lidar_node = Node(
        package='coin_d4_driver',
        executable='multi_coin_d4_node',
        output='screen',
        parameters=[
            default_params,
            {'lidar_0.port': '/dev/ttyUSB3',
             'lidar_0.reverse': False,
             'lidar_0.frame_id': 'lidar_2d_front_link',
             'lidar_1.port': '/dev/ttyUSB2',
             'lidar_1.reverse': False,
             'lidar_1.frame_id': 'lidar_2d_back_link'}],
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=15),
        sigkill_timeout=LaunchConfiguration('sigkill_timeout', default=15))

    return LaunchDescription([
        multi_lidar_node,
    ])
