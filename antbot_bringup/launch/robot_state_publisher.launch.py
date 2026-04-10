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
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('antbot_description'),
        'urdf',
        'antbot.xacro')

    calibration_yaml_path = os.path.expanduser('~/ANTBOT/calibration.yaml')
    mappings = {}
    if os.path.isfile(calibration_yaml_path):
        mappings['calibration_yaml_path'] = calibration_yaml_path
    robot_description_config = xacro.process_file(urdf_path, mappings=mappings)
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}, robot_description])

    shutdown_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[Shutdown(
                reason='robot_state_publisher_node has exited')]))

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(shutdown_event_handler)

    return ld
