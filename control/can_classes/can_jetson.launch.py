from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_bus',
            executable='ros_can_publisher.py',
            name='can_to_ros_node',
            output='screen',
            remappings=[
            ])
    ])