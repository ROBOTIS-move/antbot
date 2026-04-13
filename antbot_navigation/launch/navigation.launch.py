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
# Author: Jaehong Oh

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _resolve_map_and_launch(context, *args, **kwargs):
    """Resolve map path from world name or map argument, then launch Nav2."""
    pkg_dir = get_package_share_directory('antbot_navigation')
    mode = LaunchConfiguration('mode').perform(context)
    map_value = LaunchConfiguration('map').perform(context)
    world_value = LaunchConfiguration('world').perform(context)

    # Resolve map path
    if map_value:
        # Explicit map path provided
        map_yaml = map_value
    elif world_value:
        # Resolve from worlds.yaml
        maps_dir = os.path.join(pkg_dir, 'maps')
        worlds_yaml = os.path.join(maps_dir, 'worlds.yaml')

        if os.path.isfile(worlds_yaml):
            with open(worlds_yaml, 'r') as f:
                config = yaml.safe_load(f) or {}
            worlds = config.get('worlds', {})
            if world_value in worlds and worlds[world_value].get('map'):
                map_yaml = os.path.join(maps_dir, worlds[world_value]['map'])
            else:
                raise ValueError(
                    f"World '{world_value}' has no map defined in worlds.yaml")
        else:
            raise FileNotFoundError(f'worlds.yaml not found at {worlds_yaml}')
    else:
        raise ValueError("Either 'world' or 'map' argument must be provided")

    if not os.path.isfile(map_yaml):
        raise FileNotFoundError(f'Map file not found: {map_yaml}')

    # Scan fix relay (real mode only)
    # COIN D4 driver publishes variable-length scans with mismatched metadata.
    # This relay normalizes to fixed 400 points for Nav2 and AMCL.
    scan_fix_relay_node = None
    if mode == 'real':
        scan_fix_relay_node = Node(
            package='antbot_navigation',
            executable='scan_fix_relay.py',
            name='scan_fix_relay',
            output='screen',
            parameters=[{
                'input_topic': '/scan_0',
                'output_topic': '/scan_0_fixed',
            }])

    # Include localization launch (EKF, map_server, AMCL, disable_odom_tf)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization.launch.py')),
        launch_arguments={
            'mode': mode,
            'map': map_yaml,
        }.items())

    # Resolve config paths based on mode
    config_dir = os.path.join(pkg_dir, 'config', mode)
    nav2_params_file = os.path.join(config_dir, 'nav2_params.yaml')

    use_sim_time = (mode == 'sim')

    # Planner server (NavFn A*)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Smoother server (smooths global path before controller follows it)
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Controller server (MPPI holonomic)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Behavior server (spin, backup, wait)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Lifecycle manager for navigation nodes
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Delay Nav2 nodes to allow localization to initialize first
    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[
            planner_server_node,
            smoother_server_node,
            controller_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_navigation,
        ])

    result = [localization_launch, delayed_nav2]
    if scan_fix_relay_node is not None:
        result.insert(0, scan_fix_relay_node)
    return result


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        choices=['sim', 'real'],
        description='Operating mode: sim (Gazebo) or real (physical robot)')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='World name (from worlds.yaml) to auto-resolve map path')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map YAML file (overrides world argument)')

    return LaunchDescription([
        mode_arg,
        world_arg,
        map_arg,
        OpaqueFunction(function=_resolve_map_and_launch),
    ])
