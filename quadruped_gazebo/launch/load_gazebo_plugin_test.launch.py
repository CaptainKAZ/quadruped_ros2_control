from distutils.spawn import spawn
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.events import Shutdown


def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory(
        'quadruped_description'), 'urdf', 'quadruped.urdf.xacro')
    urdf = xacro.process_file(xacro_path, mappings={
                              'hung_up': 'False', 'gazebo_control': 'True', 'gazebo_use_cheetah_interface': 'True'})

    # Gazebo classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'verbose': 'true',
                          'pause': 'false', 'physics': 'bullet'}.items()
    )
    # Spawn
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description',
                            '-entity', 'world',
                            '-z', '0.5'],
                 output='screen')
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf.toxml(),
                'use_sim_time': True
            }]),
        spawn,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='log'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_controller],
            )
        ),
        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=[EmitEvent(event=Shutdown())],)),
    ])
