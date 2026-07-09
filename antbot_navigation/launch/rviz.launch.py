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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _qt_platform_env(context):
    qt_platform = LaunchConfiguration('qt_platform').perform(context).strip()

    if qt_platform.lower() in ('', 'inherit', 'none', 'off'):
        return {}

    if qt_platform.lower() != 'auto':
        return {'QT_QPA_PLATFORM': qt_platform}

    if os.environ.get('QT_QPA_PLATFORM'):
        return {}

    is_wayland = (
        os.environ.get('XDG_SESSION_TYPE', '').lower() == 'wayland' or
        bool(os.environ.get('WAYLAND_DISPLAY')))
    if is_wayland and os.environ.get('DISPLAY'):
        return {'QT_QPA_PLATFORM': 'xcb'}

    return {}


def _launch_rviz(context, *args, **kwargs):
    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool),
            }],
            additional_env=_qt_platform_env(context))
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('antbot_navigation')
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation.rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time')

    qt_platform_arg = DeclareLaunchArgument(
        'qt_platform',
        default_value='auto',
        choices=['auto', 'xcb', 'wayland', 'inherit'],
        description='Qt platform for RViz: auto, xcb, wayland, or inherit')

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        qt_platform_arg,
        OpaqueFunction(function=_launch_rviz),
    ])
