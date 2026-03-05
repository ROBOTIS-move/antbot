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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory('antbot_camera')
    default_config = os.path.join(pkg_share, 'param', 'camera_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('gemini336l_serial_number', default_value=''),
        DeclareLaunchArgument('gemini336l_usb_port', default_value=''),

        Node(
            package='antbot_camera',
            executable='antbot_camera_node',
            name='antbot_camera_node',
            namespace='',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
            emulate_tty=True,
        ),

        GroupAction(
            actions=[
                PushRosNamespace('sensor/camera/gemini336l'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('orbbec_camera'),
                            'launch', 'gemini_330_series.launch.py'
                        )
                    ),
                    launch_arguments={
                        'camera_name': 'front',
                        'serial_number': LaunchConfiguration('gemini336l_serial_number'),
                        'usb_port': LaunchConfiguration('gemini336l_usb_port'),
                        'depth_registration': 'true',
                        'align_mode': 'HW',
                        'enable_color': 'true',
                        'enable_depth': 'true',
                        'color_width': '640',
                        'color_height': '480',
                        'color_fps': '15',
                        'depth_width': '640',
                        'depth_height': '480',
                        'depth_fps': '15',
                        'enable_point_cloud': 'false',
                        'enable_colored_point_cloud': 'false',
                    }.items()
                ),
            ]
        ),
    ])
