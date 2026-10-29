from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = str(Path(get_package_share_directory('lab1_control')) / 'config' / 'limits.yaml')
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim', output='screen'),
        Node(package='lab1_control', executable='lab1_server', name='lab1_server',
             parameters=[config], output='screen')])
