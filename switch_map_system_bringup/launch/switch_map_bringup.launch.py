import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    switch_pcd_map_dir = FindPackageShare('switch_pcd_map')
    map_manager_dir = FindPackageShare('map_manager')

    switch_pcd_map_launch = PathJoinSubstitution(
        [switch_pcd_map_dir, 'launch', 'switch_pcd_map.launch.py']
    )
    map_manager_launch = PathJoinSubstitution(
        [map_manager_dir, 'launch', 'map_manager.launch.py']
    )

    # Declare all launch arguments
    current_map_id = LaunchConfiguration('current_map_id')
    current_area = LaunchConfiguration('current_area')
    tags_poses_file = LaunchConfiguration('tags_poses_file')
    config_path = LaunchConfiguration('config_path')

    # Create launch argument declarations
    declare_current_map_id = DeclareLaunchArgument(
        'current_map_id',
        default_value='0',
        description='Initial map ID'
    )
    declare_current_area = DeclareLaunchArgument(
        'current_area',
        default_value="'5152'",
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

    def launch_setup(context, *args, **kwargs):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(switch_pcd_map_launch),
                launch_arguments={
                    'current_map_id': current_map_id,
                    'current_area': current_area,
                    'tags_poses_file': tags_poses_file,
                    'config_path': config_path
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(map_manager_launch),
            ),
        ]

    return LaunchDescription([
        # Declare all arguments
        declare_current_map_id,
        declare_current_area,
        declare_tags_poses_file,
        declare_config_path,
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])