from distutils.spawn import spawn
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory(
        'quadruped_description'), 'urdf', 'quadruped.urdf.xacro')
    # xacro_path = os.path.join(get_package_share_directory(
    #     'quadruped_description'), 'urdf', 'mini_cheetah.urdf')
    urdf = xacro.process_file(xacro_path,mappings={'hung_up':'False'})

    # Gazebo classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'verbose': 'true','pause':'true'}.items()
    )
    # Spawn
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'world',
                                   '-z','0.5'],
                        output='screen')

    return LaunchDescription([
        
        gazebo,
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
