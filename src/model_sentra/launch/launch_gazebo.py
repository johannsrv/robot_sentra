import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('model_sentra')
    urdf_installed = os.path.join(pkg_share, 'urdf', 'model_sentra.urdf')

    # read URDF and replace package://model_sentra with absolute path
    with open(urdf_installed, 'r') as f:
        urdf_text = f.read()
    urdf_text_abs = urdf_text.replace('package://model_sentra', pkg_share)

    # write to a temporary file for spawn
    tmp_urdf = '/tmp/model_sentra_for_spawn.urdf'
    with open(tmp_urdf, 'w') as f:
        f.write(urdf_text_abs)

    # optional: set model path parent (safe)
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_MODEL_PATH',
        value=os.environ.get('GZ_MODEL_PATH', '') + ':' + os.path.dirname(pkg_share)
    )
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + os.path.dirname(pkg_share)
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': urdf_text_abs
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', tmp_urdf, '-entity', 'model_sentra', '-x', '0', '-y', '0', '-z', '1'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_model_path,
        set_gazebo_model_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity
    ])