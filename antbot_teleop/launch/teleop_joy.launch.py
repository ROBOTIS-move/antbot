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
# Author: Daun Jeong

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'autorepeat_rate': 20.0,
            'deadzone': 0.05,
        }])

    teleop_joystick_node = Node(
        package='antbot_teleop',
        executable='teleop_joystick',
        name='teleop_joystick',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_joystick_node)

    return ld
