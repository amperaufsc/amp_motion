from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    lifecycle_node = LifecycleNode(
        package='ros2_path_planning',
        executable='lifecycle_path_node',
        name='lifecycle_path_node',
        namespace='lifecycle_path_node',
        output='screen',
        parameters=[{
            'max_angle_change_gain': 5.0,
            'std_dvt_track_width_gain': 0.0,
            'std_dvt_left_right_cones': 0.0,
            'max_wrong_color_gain': 20.0,
            'sqd_diff_path_len_sensor_range': 0.0,
            'T': 0.01,
            'max_acceleration': 0.5,
            'braking_acceleration': 0.5,
            'lateral_acceleration': 0.5,
            'max_speed': 4.0,
            'frame_id': 'frame_id',
        }]
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
        TimerAction(period=6.0, actions=[configure_event, LogInfo(msg='Configurando Path')]),
        TimerAction(period=9.0, actions=[activate_event, LogInfo(msg='Ativando Path')])
    ])
