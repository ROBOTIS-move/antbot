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
    hw_interface_dir = get_package_share_directory('antbot_hw_interface')
    description_dir = get_package_share_directory('antbot_description')

    # URDF
    urdf_path = os.path.join(description_dir, 'urdf', 'antbot.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Hardware parameters
    hw_params = os.path.join(
        hw_interface_dir, 'config', 'board_params.yaml')
    control_table_path = os.path.join(
        hw_interface_dir, 'config', 'control_table.xml')

    # Controller parameters
    swerve_controller_dir = get_package_share_directory(
        'antbot_swerve_controller')
    controller_params = os.path.join(
        swerve_controller_dir, 'config', 'swerve_drive_controller.yaml')

    # ros2_control_node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            hw_params,
            controller_params,
            robot_description,
            {'control_table_path': control_table_path}],
        remappings=[
            ('controller_manager/robot_description', 'robot_description')])

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen')

    swerve_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['antbot_swerve_controller'],
        output='screen')

    shutdown_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[Shutdown(
                reason='controller_manager_node has exited')]))

    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_spawner,
        shutdown_event_handler,
    ])
