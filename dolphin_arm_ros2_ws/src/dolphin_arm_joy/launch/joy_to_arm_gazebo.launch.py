from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # === 启动 joy 节点 ===
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js1',     # 根据你系统的手柄设备调整
                'autorepeat_rate': 80.0,  # 每秒80次，12.5ms一帧
                'deadzone': 0.05,
                'coalesce_interval_ms': 5
            }],
            output='screen'
        ),

        # === 启动机械臂控制节点 ===
        Node(
            package='dolphin_arm_joy',
            executable='joy_to_arm',
            name='joy_to_arm',
            parameters=[{'use_sim_time': True}],  #仿真时间同步
            output='screen'
        ),

        # === 启动夹爪控制节点 ===
        Node(
            package='dolphin_arm_joy',
            executable='joy_to_gripper',
            name='joy_to_gripper',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
