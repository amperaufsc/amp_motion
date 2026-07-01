from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('control'),
        'config',
        'control_parameters.yaml'
    )

    control_node = LifecycleNode(
        package='control',
        executable='control_node_life.py',
        name='control_node',
        namespace=LaunchConfig('namespace'),
        output='screen',

        remappings=[
            ('path_concatenated', LaunchConfig('path_concatenated')),
            ('odom', LaunchConfig('odom')),
            ('control', LaunchConfig('control'))
        ],

        parameters=[parameters_file],
    )

    return LaunchDescription([

        LaunchArg(
            'namespace',
            default_value='control',
            description='Namespace for node'
        ),

        LaunchArg(
            'path',
            default_value='/path',
            description='Path message topic'
        ),

        LaunchArg(
            'odom',
            default_value='/odom',
            description='Odom message topic'
        ),

        LaunchArg(
            'control',
            default_value='control',
            description='Control message topic'
        ),

        LaunchArg(
            'path_concatenated',
            default_value='path_concatenated',
            description='Control message topic'
        ),

        control_node,
    ])