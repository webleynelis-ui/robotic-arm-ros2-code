from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    启动 MoveIt 的 move_group 节点与 RViz2 可视化界面。
    - 支持 Gazebo 仿真时间 (use_sim_time=True)
    - 兼容 dolphin_arm_moveitassis 的配置路径
    """

    # 获取配置包路径
    pkg_share = get_package_share_directory("dolphin_arm_moveitassis")

    # 构建 MoveIt 配置对象（URDF + SRDF + 控制器）
    moveit_config = (
        MoveItConfigsBuilder("Robot_Arm_Urdf", package_name="dolphin_arm_moveitassis")
        .robot_description(file_path=os.path.join(pkg_share, "config", "Robot_Arm_Urdf.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(pkg_share, "config", "Robot_Arm_Urdf.srdf"))
        .trajectory_execution(file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml"))
        .to_moveit_configs()
    )

    # 创建 LaunchDescription
    ld = LaunchDescription()

    # 启动 move_group
    my_generate_move_group_launch(ld, moveit_config)

    # 启动 RViz2
    my_generate_moveit_rviz_launch(ld, moveit_config)

    return ld


def my_generate_move_group_launch(ld, moveit_config):
    """
    启动 MoveIt 核心节点 /move_group
    """

    # --- Launch 参数声明 ---
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # --- MoveIt 运行配置 ---
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": True},  # Gazebo 仿真时间同步
    ]

    # --- 启动 move_group 节点 ---
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": ":0"},
    )
    return ld


def my_generate_moveit_rviz_launch(ld, moveit_config):
    """
    启动带 MoveIt 插件的 RViz2
    """

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        {"use_sim_time": True},  # 同步仿真时间
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="screen",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )
    return ld
