from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    lifecycle_node = LifecycleNode(
        package='control',  
        executable='lifecycle_control_node.py',  
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

    return LaunchDescription([
        lifecycle_node,
        TimerAction(period=10.0, actions=[configure_event, LogInfo(msg='Configurando ControlNode...')]),
        TimerAction(period=13.0, actions=[activate_event, LogInfo(msg='Ativando ControlNode...')])
    ])
