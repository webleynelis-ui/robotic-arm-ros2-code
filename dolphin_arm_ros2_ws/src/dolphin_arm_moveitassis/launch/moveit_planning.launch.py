from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("Robot_Arm_Urdf", package_name="dolphin_arm_moveitassis")
        .robot_description(file_path="config/Robot_Arm_Urdf.urdf.xacro")
        .robot_description_semantic(file_path="config/Robot_Arm_Urdf.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=moveit_config.package_path / "config/moveit.rviz"
    )

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation (Gazebo) clock"),
        DeclareLaunchArgument("rviz_config", default_value=str(rviz_config),
                              description="RViz configuration file")
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
    )

    nodes_to_start = [
        *declared_arguments,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
