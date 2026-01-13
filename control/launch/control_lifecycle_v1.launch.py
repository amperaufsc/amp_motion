from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, EmitEvent, RegisterEventHandler
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
        package='control',  
        executable='lifecycle_control_v1.py',  
        name='control_node',
        namespace='',
        output='screen',
        parameters=[
            {
                'Kp': 0.05,
                'Ki': 0.01,
                'Kd': 0.0,
                'T': 0.01,
            }
        ]
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
        LaunchArg('namespace', default_value=['control'], description='Namespace for node'),
        LaunchArg('path', default_value=['/path'], description='Path message topic'),
        LaunchArg('odom', default_value=['/odom'], description='Odom message topic'),
        LaunchArg('control', default_value=['control'], description='Control message topic'),
        LaunchArg('T', default_value=['30.0'], description='Sampling period'),
        lifecycle_node,
        TimerAction(period=13.0, actions=[configure_event, LogInfo(msg='Configurando ControlNode...')]),
        TimerAction(period=16.0, actions=[activate_event, LogInfo(msg='Ativando ControlNode...')]),
        on_exit_handler
    ])