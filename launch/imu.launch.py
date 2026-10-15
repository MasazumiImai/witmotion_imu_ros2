# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('witmotion_imu_driver')) / 'config/imu.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(config)),
        DeclareLaunchArgument('namespace', default_value=''),
        Node(package='witmotion_imu_driver', executable='witmotion_imu_node',
             name='witmotion_imu', namespace=LaunchConfiguration('namespace'),
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
