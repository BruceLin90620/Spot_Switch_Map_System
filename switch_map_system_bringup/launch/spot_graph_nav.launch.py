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
    spot_graph_nav_dir = FindPackageShare('spot_graph_nav')
    switch_pcd_map_dir = FindPackageShare('switch_pcd_map')
    map_manager_dir = FindPackageShare('map_manager')
    bringup_pkg_dir = get_package_share_directory('switch_map_system_bringup')

    # Get the path to the config file
    config_file = os.path.join(bringup_pkg_dir, 'config', 'switch_map_system.yaml')

    # Get launch files
    spot_graph_nav_launch = PathJoinSubstitution(
        [spot_graph_nav_dir, 'launch', 'spot_graph_nav.launch.py']
    )
    switch_pcd_map_launch = PathJoinSubstitution(
        [switch_pcd_map_dir, 'launch', 'switch_pcd_map.launch.py']
    )
    map_manager_launch = PathJoinSubstitution(
        [map_manager_dir, 'launch', 'map_manager.launch.py']
    )

    # Declare all launch arguments
    # Switch PCD Map arguments
    current_map_id = LaunchConfiguration('current_map_id')
    current_area = LaunchConfiguration('current_area')
    tags_poses_file = LaunchConfiguration('tags_poses_file')
    config_path = LaunchConfiguration('config_path')
    
    # Spot Graph Nav arguments
    map_config_path = LaunchConfiguration('map_config_path')
    spot_tags_poses_file = LaunchConfiguration('spot_tags_poses_file')

    # Create launch argument declarations
    # Switch PCD Map declarations
    declare_current_map_id = DeclareLaunchArgument(
        'current_map_id',
        default_value='0',
        description='Initial map ID'
    )
    declare_current_area = DeclareLaunchArgument(
        'current_area',
        default_value='5152',
        description='Initial area'
    )
    declare_tags_poses_file = DeclareLaunchArgument(
        'tags_poses_file',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/tags_position.yaml',
        description='Path to tags poses file for Switch PCD Map'
    )
    declare_config_path = DeclareLaunchArgument(
        'config_path',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/map_path.yaml',
        description='Path to map configuration file'
    )

    # Spot Graph Nav declarations
    declare_map_config_path = DeclareLaunchArgument(
        'map_config_path',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/map_path.yaml',
        description='Path to map configuration file for Spot Graph Nav'
    )
    declare_spot_tags_poses_file = DeclareLaunchArgument(
        'spot_tags_poses_file',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml',
        description='Path to tags poses file for Spot Graph Nav'
    )

    def launch_setup(context, *args, **kwargs):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spot_graph_nav_launch),
                launch_arguments={
                    'map_config_path': map_config_path,
                    'tags_poses_file': spot_tags_poses_file
                }.items()
            ),
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
        # Switch PCD Map arguments
        declare_current_map_id,
        declare_current_area,
        declare_tags_poses_file,
        declare_config_path,
        # Spot Graph Nav arguments
        declare_map_config_path,
        declare_spot_tags_poses_file,
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])