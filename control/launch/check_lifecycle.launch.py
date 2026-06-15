import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import LogInfo

def generate_launch_description():
    # Cria a descrição do Launch e adiciona o nó à lista de execução
    return LaunchDescription([
        LogInfo(msg="[Launch] Iniciando o nó Float Publisher em modo Lifecycle (Unconfigured)..."),
        LifecycleNode(
        package='control',          # Nome do seu pacote ROS 2
        executable='check_node_lifecycle',  # Nome do executável definido no seu CMakeLists.txt
        name='float_publisher',             # Nome que o nó terá em tempo de execução
        namespace='',                       # Namespace (opcional)
        output='screen',                    # Garante que os logs (RCLCPP_INFO) apareçam no terminal
        remappings=[('/control_command', '/control_command')]  # Remapeamento do tópico, se necessário
        )
    ])