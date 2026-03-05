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
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('antbot_description')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    use_joint_state_publisher_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='false',
        description='Launch joint state publisher')

    use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='true',
        description='Launch joint state publisher with GUI')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    use_joint_state_publisher_gui = LaunchConfiguration(
        'use_joint_state_publisher_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # Process URDF via xacro
    urdf_path = os.path.join(pkg_dir, 'urdf', 'antbot.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description])

    # Joint State Publisher (without GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
        condition=IfCondition(use_joint_state_publisher))

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
        condition=IfCondition(use_joint_state_publisher_gui))

    # RViz
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'antbot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz))

    # Shutdown when robot_state_publisher exits
    shutdown_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[Shutdown(reason='robot_state_publisher_node has exited')]))

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_joint_state_publisher_arg)
    ld.add_action(use_joint_state_publisher_gui_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    ld.add_action(shutdown_event_handler)

    return ld
