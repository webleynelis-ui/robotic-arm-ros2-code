# joy_inspector_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动joy节点读取USB摇杆
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js1',  # 指定摇杆设备
                'deadzone': 0.1,
                'autorepeat_rate': 20.0
            }]
        ),
        # 启动监听器
        Node(
            package='dolphin_arm_joy',
            executable='joy_inspector',
            name='joy_inspector'
        )
    ])