from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig


def generate_launch_description():

    return LaunchDescription([
        LaunchArg('namespace', default_value=['control'], description='Namespace for node'),
        LaunchArg('path', default_value=['/path'], description='Path message topic'),
        LaunchArg('odom', default_value=['/odom'], description='Odom message topic'),
        LaunchArg('control', default_value=['control'], description='Control message topic'),
        LaunchArg('T', default_value=['0.01'], description='Sampling period'),
        Node(
            package='ros2_control',
            executable='control_node.py',
            name='control_node',
            namespace= LaunchConfig('namespace'),
            remappings=[('path', LaunchConfig('path')),
                        ('odom', LaunchConfig('odom')),
                        ('control', LaunchConfig('control'))],
            parameters=[{'T': LaunchConfig('T')}]
        )
    ])