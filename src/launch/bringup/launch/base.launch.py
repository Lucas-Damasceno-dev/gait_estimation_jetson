from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    imu_params = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'imu.yaml'
    ])

    actuator_params = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'actuator.yaml'
    ])

    pressure_params = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'pressure.yaml'
    ])

    return LaunchDescription([
        Node(
            package='imu_node',
            executable='imu_node',
            output='screen',
            parameters=[imu_params]
        ),

        Node(
            package='pressure_node',
            executable='pressure_node',
            output='screen',
            parameters=[pressure_params]
        ),
        
        Node(
            package='mg8008e_node',
            executable='actuator_node',
            output='screen',
            parameters=[actuator_params]
        ),

        Node(
            package='sync_node',
            executable='sync_node',
            output='screen',
        )
    ])
