from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    return LaunchDescription([
        LaunchArg('namespace', default_value=['low_level_control'], description='Namespace for node'),

        Node(
            package='control',
            executable='low_level_control_node.py',
            name='low_level_control_node',
            namespace= LaunchConfig('namespace'),
        )
    
    ])

