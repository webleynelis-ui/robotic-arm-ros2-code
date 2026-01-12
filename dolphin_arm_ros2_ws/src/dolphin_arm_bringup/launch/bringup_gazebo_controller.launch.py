from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_pkg = get_package_share_directory("dolphin_arm_description")
    ctrl_pkg = get_package_share_directory("dolphin_arm_controller")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, "launch", "gazebo.launch.py")
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, "launch", "controller.launch.py")
        )
    )

    delayed_controller = TimerAction(period=6.0, actions=[controller_launch])

    return LaunchDescription([
        gazebo_launch,
        delayed_controller,
    ])
