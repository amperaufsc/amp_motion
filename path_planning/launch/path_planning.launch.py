from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        return LaunchDescription([
            LaunchArg('namespace', default_value=[''], description='namespace'),
            LaunchArg('path', default_value=['path'], description='path msg'),
            LaunchArg('odom', default_value=['/orbslam/odom'], description='odom msg'),
            LaunchArg('track_pub', default_value=['track'], description='track msg'),
            LaunchArg('go', default_value=['/as_amp/mission_select/go'], description='go msg'),
            LaunchArg('max_angle_change_gain', default_value=['5.0'], description='max_angle_change_gain msg'),
            LaunchArg('std_dvt_track_width_gain', default_value=['0.0'], description='std_dvt_track_width_gain msg'),
            LaunchArg('std_dvt_left_right_cones', default_value=['0.0'], description='std_dvt_left_right_cones msg'),
            LaunchArg('max_wrong_color_gain', default_value=['20.0'], description='max_wrong_color_gain msg'),
            LaunchArg('sqd_diff_path_len_sensor_range', default_value=['0.0'], description='sqd_diff_path_len_sensor_range msg'),
            LaunchArg('T', default_value=['0.01'], description='T msg'),
            LaunchArg('max_acceleration', default_value=['0.5'], description='max_acceleration msg'),
            LaunchArg('braking_acceleration', default_value=['0.5'], description='braking_acceleration msg'),
            LaunchArg('lateral_acceleration', default_value=['0.5'], description='lateral_acceleration msg'),
            LaunchArg('max_speed', default_value=['2.5'], description='max_speed msg'),

            LaunchArg('track_pointcloud', default_value=['track_pointcloud'], description='track msg pointcloud'),
            LaunchArg('frame_id', default_value = ['map'], description = 'frame_id msg'),
            
            Node(
                package='path_planning',
                executable='path_node_life.py',
                name='path_node',
                namespace=LaunchConfig('namespace'),
                remappings=[
                    ('path', LaunchConfig('path')),
                    ('odom', LaunchConfig('odom')),
                    ('go', LaunchConfig('go')),
                    ('track_pub', LaunchConfig('track_pub'))
                    ],
                parameters = [{'max_angle_change_gain': LaunchConfig('max_angle_change_gain')},
                    {'std_dvt_track_width_gain': LaunchConfig('std_dvt_track_width_gain')},
                    {'std_dvt_left_right_cones': LaunchConfig('std_dvt_left_right_cones')},
                    {'max_wrong_color_gain': LaunchConfig('max_wrong_color_gain')},
                    {'sqd_diff_path_len_sensor_range': LaunchConfig('sqd_diff_path_len_sensor_range')},
                    {'T': LaunchConfig('T')},
                    {'frame_id': LaunchConfig('frame_id')}]
                
            )
        
    ])