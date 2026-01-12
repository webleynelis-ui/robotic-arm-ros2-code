from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("dolphin_arm_moveitassis")

    moveit_config = (
        MoveItConfigsBuilder("Robot_Arm_Urdf", package_name="dolphin_arm_moveitassis")
        .robot_description(file_path=os.path.join(pkg_share, "config", "Robot_Arm_Urdf.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(pkg_share, "config", "Robot_Arm_Urdf.srdf"))
        .trajectory_execution(file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml"))
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
