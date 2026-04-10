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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('antbot_bringup')
    imu_dir = get_package_share_directory('antbot_imu')
    camera_dir = get_package_share_directory('antbot_camera')
    ublox_gps_dir = get_package_share_directory('ublox_gps')
    teleop_dir = get_package_share_directory('antbot_teleop')

    # Robot State Publisher
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'robot_state_publisher.launch.py')))

    # Controller (ros2_control_node + spawners)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'controller.launch.py')))

    # IMU
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            imu_dir, 'launch', 'imu.launch.py')))

    # 2D LiDAR
    lidar_2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'lidar_2d.launch.py')))

    # 3D LiDAR
    lidar_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'lidar_3d.launch.py')))

    # GNSS
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ublox_gps_dir, 'launch', 'ublox_gps_node-launch.py')))

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            camera_dir, 'launch', 'camera.launch.py')))

    # Joystick teleop
    teleop_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            teleop_dir, 'launch', 'teleop_joy.launch.py')))

    return LaunchDescription([
        robot_state_publisher_launch,
        controller_launch,
        imu_launch,
        lidar_2d_launch,
        lidar_3d_launch,
        gnss_launch,
        camera_launch,
        teleop_joy_launch,
    ])
