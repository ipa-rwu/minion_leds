import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MINION_MODEL = "tank"
PKG = "minion_leds"
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    param_file_name = MINION_MODEL + '_client.yaml'
    client_param = LaunchConfiguration(
        'client_param',
        default=os.path.join(
            get_package_share_directory(PKG),
            'params',
            param_file_name))
    
    remappings = [('/leds_server', '/minion_eyes')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'client_param',
            default_value=client_param,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package=PKG,
            executable='minion_leds_client',
            name='minion_leds_client',
            parameters=[client_param],
            remappings=remappings),
            # output='screen'),
    ])