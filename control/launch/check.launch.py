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
        LaunchArg('/control_command', default_value=['/control_command'], description='Path message topic'),
        Node(
            package='control',
            executable='check_node',
            name='check_node',
            remappings=[('/control_command', LaunchConfig('/control_command'))]
            #parameters=[{'T': LaunchConfig('T')}]
        )
    ])