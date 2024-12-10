from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Create launch argument declarations
    declare_current_map_id = DeclareLaunchArgument(
        'current_map_id',
        default_value='0',
        description='Initial map ID'
    )

    declare_current_area = DeclareLaunchArgument(
        'current_area',
        default_value="'121315'",
        description='Initial area'
    )

    declare_tags_poses_file = DeclareLaunchArgument(
        'tags_poses_file',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_map_system_bringup/config/tags_position.yaml',
        description='Path to tags poses file'
    )

    declare_config_path = DeclareLaunchArgument(
        'config_path',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_map_system_bringup/config/map_path.yaml',
        description='Path to map configuration file'
    )

    # Create node
    switch_map_node = Node(
        package='switch_pcd_map',
        executable='switch_pcd_map_node',
        name='switch_pcd_map_node',
        parameters=[{
            'current_map_id': LaunchConfiguration('current_map_id'),
            'current_area': LaunchConfiguration('current_area'),
            'tags_poses_file': LaunchConfiguration('tags_poses_file'),
            'config_path': LaunchConfiguration('config_path')
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_current_map_id,
        declare_current_area,
        declare_tags_poses_file,
        declare_config_path,
        switch_map_node
    ])