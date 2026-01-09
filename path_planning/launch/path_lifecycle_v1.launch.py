from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessExit

class Respawn:
    
    def callback(self, event, context):
        return [
            LogInfo(msg='Enviando shutdown pro MAPPER.'),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == lifecycle_node,
                    transition_id=Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
                )
            )
                
        ]
    
def generate_launch_description():
        global lifecycle_node
        lifecycle_node = LifecycleNode(
            package='ros2_path_planning',
            executable='lifecycle_path_node_v1.py',
            name='path_node',
            namespace='',
            respawn = True,
            output='screen',
            parameters = [{'max_angle_change_gain': LaunchConfig('max_angle_change_gain')},
                    {'std_dvt_track_width_gain': LaunchConfig('std_dvt_track_width_gain')},
                    {'std_dvt_left_right_cones': LaunchConfig('std_dvt_left_right_cones')},
                    {'max_wrong_color_gain': LaunchConfig('max_wrong_color_gain')},
                    {'sqd_diff_path_len_sensor_range': LaunchConfig('sqd_diff_path_len_sensor_range')},
                    {'T': LaunchConfig('T')},
                    {'frame_id': LaunchConfig('frame_id')}]
            
        )

        counter = Respawn()

        configure_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda node: node == lifecycle_node,
                transition_id=Transition.TRANSITION_CONFIGURE
            )
        )

        activate_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda node: node == lifecycle_node,
                transition_id=Transition.TRANSITION_ACTIVATE
            )
        )

        on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_node,
            on_exit=counter.callback
        )
    )
        return LaunchDescription([
            LaunchArg('namespace', default_value=['path'], description='namespace'),
            LaunchArg('path', default_value=['path'], description='path msg'),
            LaunchArg('odom', default_value=['/odom'], description='odom msg'),
            LaunchArg('track', default_value=['/track'], description='track msg'),
            LaunchArg('go', default_value=['go'], description='go msg'),
            LaunchArg('max_angle_change_gain', default_value=['5.0'], description='max_angle_change_gain msg'),
            LaunchArg('std_dvt_track_width_gain', default_value=['0.0'], description='std_dvt_track_width_gain msg'),
            LaunchArg('std_dvt_left_right_cones', default_value=['0.0'], description='std_dvt_left_right_cones msg'),
            LaunchArg('max_wrong_color_gain', default_value=['20.0'], description='max_wrong_color_gain msg'),
            LaunchArg('sqd_diff_path_len_sensor_range', default_value=['0.0'], description='sqd_diff_path_len_sensor_range msg'),
            LaunchArg('T', default_value=['3.0'], description='T msg'),
            LaunchArg('track_pointcloud', default_value=['track_pointcloud'], description='track msg pointcloud'),
            LaunchArg('frame_id', default_value = ['fsds/map'], description = 'frame_id msg'),
            lifecycle_node,
            TimerAction(period=8.0, actions=[configure_event, LogInfo(msg='Configurando Path')]),
            TimerAction(period=12.0, actions=[activate_event, LogInfo(msg='Ativando Path')]),
            on_exit_handler     
    ])