import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = 'dolphin_arm_description'
    share = FindPackageShare(package=pkg).find(pkg)

    # 你的 xacro
    xacro_path = os.path.join(share, 'urdf', 'robot_arm_urdf.urdf.xacro')

    # 用 xacro 库在 Python 里展开
    doc = xacro.process_file(xacro_path)
    robot_desc = {'robot_description': doc.toxml()}

    # 1) 启动 Gazebo（Classic），显式加载 ROS 插件
    gazebo = ExecuteProcess(
        cmd=[
            'bash','-lc',
            'export GAZEBO_MODEL_DATABASE_URI=; '
            'export ROS_NAMESPACE=/gazebo_ros2_control; '
            'gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        ],
        output='screen'
    )


    # 2) RSP：发布 /robot_description 和 TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, robot_desc, {'publish_frequency': 15.0}],
        output='screen'
    )

    # 3) spawn 到世界里（抬起一点，防止埋地面）
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'dolphin_arm',
                   '-x', '0', '-y', '0', '-z', '0.30',
                   '-timeout', '60'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        # chain1, chain2,
    ])
