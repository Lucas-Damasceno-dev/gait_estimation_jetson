import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Argumentos de inicialização
    start_sensors_arg = DeclareLaunchArgument(
        'start_sensors',
        default_value='false',
        description='Se True, inicia também os drivers de hardware dos sensores (IMU e Palmilhas)'
    )

    open_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Se True, abre a interface gráfica do RViz2 com o perfil configurado'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame de referência global para renderização no RViz'
    )

    # Caminho para o perfil RViz
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'rviz',
        'sensors_monitor.rviz'
    ])

    # Launch opcional dos sensores (se start_sensors:=true)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'sensors_only.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_sensors'))
    )

    # Nó C++ Visualizador e Calculador de Hz
    sensor_visualizer_node = Node(
        package='gait_estimation_pkg',
        executable='sensor_visualizer_node',
        name='sensor_visualizer_node',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate_hz': 25.0,
            'max_pressure_value': 3500.0
        }]
    )

    # Nó do RViz2 com arquivo de visualização pré-carregado
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sensors',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        start_sensors_arg,
        open_rviz_arg,
        frame_id_arg,
        sensors_launch,
        sensor_visualizer_node,
        rviz2_node
    ])
