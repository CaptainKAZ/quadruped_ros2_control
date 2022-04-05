import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory(
        'quadruped_description'), 'urdf', 'quadruped.urdf.xacro')
    # xacro_path = os.path.join(get_package_share_directory(
    #     'quadruped_description'), 'urdf', 'mini_cheetah.urdf')
    return LaunchDescription([
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
                'robot_description': xacro.process_file(xacro_path).toxml()
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
