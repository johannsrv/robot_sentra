import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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

    # Lista de controladores
    controllers = [
        'front_left_shoulder_position_controller',
        'front_left_leg_position_controller',
        'front_left_foot_position_controller',
        'front_right_shoulder_position_controller',
        'front_right_leg_position_controller',
        'front_right_foot_position_controller',
        'rear_left_shoulder_position_controller',
        'rear_left_leg_position_controller',
        'rear_left_foot_position_controller',
        'rear_right_shoulder_position_controller',
        'rear_right_leg_position_controller',
        'rear_right_foot_position_controller',
    ]

    # Crear TimerAction para spawnear cada controlador con pequeños offsets
    spawner_actions = []
    base_delay = 2.0
    for i, ctrl in enumerate(controllers):
        t = base_delay + i * 0.2
        spawner = TimerAction(
            period=t,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=[ctrl],
                output='screen'
            )]
        )
        spawner_actions.append(spawner)

    # Nodo para mover el motor de prueba (motor_tester)
    # Lo lanzamos con un pequeño retraso para asegurar que los controladores estén activos
    motor_tester_node = TimerAction(
        period=base_delay + len(controllers) * 0.2 + 2.0,  # espera extra
        actions=[Node(
            package='motor_controller',
            executable='motor_tester',
            name='motor_tester',
            output='screen'
        )]
    )

    # Agregar todo al LaunchDescription
    ld_items = [launch_model, spawner_joint_state_broadcaster] + spawner_actions + [motor_tester_node]
    return LaunchDescription(ld_items)
