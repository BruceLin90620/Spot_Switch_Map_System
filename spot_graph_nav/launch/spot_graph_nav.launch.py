from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('spot_graph_nav')
    params_file = os.path.join(pkg_share, 'config', 'spot_graph_nav_params.yaml')

    # Declare launch arguments
    map_config_path = LaunchConfiguration('map_config_path')
    tags_poses_file = LaunchConfiguration('tags_poses_file')
    hostname = LaunchConfiguration('hostname')

    declare_map_config_path = DeclareLaunchArgument(
        'map_config_path',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_map_system_bringup/config/map_path.yaml',
        description='Path to map configuration file'
    )

    declare_tags_poses_file = DeclareLaunchArgument(
        'tags_poses_file',
        default_value='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml',
        description='Path to tags poses file'
    )

    declare_hostname = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.50.3',
        description='Hostname/IP of the Spot robot'
    )

    # Create node
    spot_nav_node = Node(
        package='spot_graph_nav',
        executable='spot_graph_nav_node',
        name='spot_graph_nav_node',
        parameters=[{
            'map_config_path': map_config_path,
            'tags_poses_file': tags_poses_file,
        }],
        # Pass hostname as a positional argument
        arguments=[hostname],
        output='screen'
    )

    return LaunchDescription([
        declare_map_config_path,
        declare_tags_poses_file,
        declare_hostname,
        spot_nav_node
    ])