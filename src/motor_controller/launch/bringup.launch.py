import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_sentra_pkg = get_package_share_directory('model_sentra')
    motor_controller_pkg = get_package_share_directory('motor_controller')

    # Lanzar el modelo en Gazebo
    launch_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(model_sentra_pkg, 'launch', 'launch_gazebo.py')
        )
    )

    # Spawner del joint_state_broadcaster
    spawner_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        launch_model,
        spawner_joint_state_broadcaster
    ])
