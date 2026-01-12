import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'dolphin_arm_description'
    share = get_package_share_directory(pkg)
    xacro_file = os.path.join(share, 'urdf', 'robot_arm_moveit_urdf.urdf.xacro')
    rviz_cfg  = os.path.join(share, 'config', 'config.rviz')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )
    return LaunchDescription([rsp, jsp, rviz])
