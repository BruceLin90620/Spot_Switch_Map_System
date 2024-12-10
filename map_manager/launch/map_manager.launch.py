import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_manager',
            executable='map_manager_node',
            name='map_manager_node',
            output='screen'
        ),
    ])