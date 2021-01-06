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

    param_file_name = MINION_MODEL + '_all.yaml'
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory(PKG),
            'params',
            param_file_name))

    leds_launch_dir =LaunchConfiguration(
        'leds_launch_dir',
        default=os.path.join(
            get_package_share_directory(PKG),
            'launch'))
    minion_ultrasonic_launch_dir =LaunchConfiguration(
        'minion_ultrasonic_launch_dir',
        default=os.path.join(
            get_package_share_directory("minion_ultrasonic"),
            'launch'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([leds_launch_dir, '/minion_leds_client.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params': param_dir}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([leds_launch_dir, '/minion_leds_server.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params': param_dir}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([minion_ultrasonic_launch_dir, '/minion_ultrasonic_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time}.items(),
        ),
    ])