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
# Authors: Jaehun Park

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('antbot_camera')
    default_config = os.path.join(pkg_share, 'param', 'camera_test_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('output_dir', default_value='/tmp/antbot_camera_test'),
        DeclareLaunchArgument('save_interval_sec', default_value='1.0'),

        Node(
            package='antbot_camera',
            executable='camera_test_node',
            name='camera_test_node',
            namespace='',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'test.output_dir': LaunchConfiguration('output_dir'),
                    'test.save_interval_sec': LaunchConfiguration('save_interval_sec'),
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
