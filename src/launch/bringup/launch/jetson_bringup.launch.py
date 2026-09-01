from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_emulator_arg = DeclareLaunchArgument(
        'use_emulator',
        default_value='false',
        description='Se True, utiliza o emulador sintético STM32 para testes sem hardware conectado'
    )

    serial_params = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'stm32_serial.yaml'
    ])

    return LaunchDescription([
        use_emulator_arg,

        # Nó Driver Serial STM32 (Palmilha Esquerda / STM32 Master)
        Node(
            package='serial_driver_node',
            executable='serial_driver_node',
            name='serial_driver_left',
            output='screen',
            parameters=[serial_params],
            condition=None
        ),

        # Nó de Sincronização Temporal Multissensorial
        Node(
            package='sync_node',
            executable='sync_node',
            name='sync_node',
            output='screen'
        ),

        # Nó de Estimação de Marcha (IA em C++)
        Node(
            package='gait_estimation_pkg',
            executable='gait_estimation_node',
            name='gait_estimation_node',
            output='screen'
        ),

        # Nó de Visualização e Ponte CoppeliaSim
        Node(
            package='gait_estimation_pkg',
            executable='visualization_node',
            name='visualization_node',
            output='screen'
        )
    ])
