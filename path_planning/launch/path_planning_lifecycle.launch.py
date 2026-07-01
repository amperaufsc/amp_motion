from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig

from launch_ros.actions import LifecycleNode


def generate_launch_description():

    planning_node = LifecycleNode(
        package='path_planning',
        executable='path_node_life.py',
        name='planning_node',
        namespace=LaunchConfig('namespace'),
        output='screen',

        remappings=[
            ('path', LaunchConfig('path')),
            ('path_concatenated', LaunchConfig('path_concatenated')),
            ('odom', LaunchConfig('odom')),
            ('go', LaunchConfig('go')),
            ('track', LaunchConfig('track')),
            ('track_pointcloud', LaunchConfig('track_pointcloud'))
        ],

        parameters=[
            {'max_angle_change_gain': LaunchConfig('max_angle_change_gain')},
            {'std_dvt_track_width_gain': LaunchConfig('std_dvt_track_width_gain')},
            {'std_dvt_left_right_cones': LaunchConfig('std_dvt_left_right_cones')},
            {'max_wrong_color_gain': LaunchConfig('max_wrong_color_gain')},
            {'sqd_diff_path_len_sensor_range': LaunchConfig('sqd_diff_path_len_sensor_range')},
            {'T': LaunchConfig('T')},
            {'frame_id': LaunchConfig('frame_id')},
            {'max_acceleration': LaunchConfig('max_acceleration')},
            {'braking_acceleration': LaunchConfig('braking_acceleration')},
            {'lateral_acceleration': LaunchConfig('lateral_acceleration')},
            {'max_speed': LaunchConfig('max_speed')}
        ]
    )

    return LaunchDescription([

        LaunchArg('namespace', default_value='path'),
        LaunchArg('path', default_value='path'),
        LaunchArg('path_concatenated', default_value='path_concatenated'),
        LaunchArg('track_pointcloud', default_value='track_pointcloud'),

        LaunchArg('odom', default_value='odom'),
        LaunchArg('track', default_value='track'),
        LaunchArg('go', default_value='/AGORAVOCEVAICORNO'),

        LaunchArg('max_angle_change_gain', default_value='5.0'),
        LaunchArg('std_dvt_track_width_gain', default_value='0.0'),
        LaunchArg('std_dvt_left_right_cones', default_value='0.0'),
        LaunchArg('max_wrong_color_gain', default_value='20.0'),
        LaunchArg('sqd_diff_path_len_sensor_range', default_value='0.0'),

        LaunchArg('T', default_value='0.01'),

        LaunchArg('max_acceleration', default_value='0.5'),
        LaunchArg('braking_acceleration', default_value='0.5'),
        LaunchArg('lateral_acceleration', default_value='0.5'),
        LaunchArg('max_speed', default_value='2.5'),

        LaunchArg('track_pointcloud', default_value='track_pointcloud'),
        LaunchArg('frame_id', default_value='fsds/map'),

        planning_node,
    ])