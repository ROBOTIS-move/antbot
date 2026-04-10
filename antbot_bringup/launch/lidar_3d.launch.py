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
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('antbot_bringup')
    config_path = os.path.join(bringup_dir, 'config', 'lidar_3d.yaml')

    vanjee_lidar_node = Node(
        namespace='vanjee_lidar_sdk',
        package='vanjee_lidar_sdk',
        executable='vanjee_lidar_sdk_node',
        output='screen',
        parameters=[{'config_path': config_path}])

    return LaunchDescription([
        vanjee_lidar_node,
    ])
