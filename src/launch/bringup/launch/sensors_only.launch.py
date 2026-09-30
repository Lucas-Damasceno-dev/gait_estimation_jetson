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

    pressure_params = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'pressure.yaml'
    ])

    return LaunchDescription([
        # Nó da IMU (BNO055 via I2C nativo Linux)
        Node(
            package='imu_node',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[imu_params]
        ),

        # Nó das Palmilhas (2x STMicroelectronics USB Serial /dev/ttyACM*)
        Node(
            package='pressure_node',
            executable='pressure_node',
            name='pressure_node',
            output='screen',
            parameters=[pressure_params]
        )
    ])
