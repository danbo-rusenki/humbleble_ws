from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="実機=false / シミュレータ=true",
    )

    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .robot_description_semantic(file_path="config/amir_mecanum3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    condition_id_arg = DeclareLaunchArgument(
        "condition_id",
        default_value="cond_single",
        description="実験条件ID（CSVのcondition_id列に記録される）",
    )

    pick_place_node = Node(
        package="amir_operation",
        executable="pick_place_humble_10",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"condition_id": LaunchConfiguration("condition_id")},
        ],
    )

    return LaunchDescription([
        sim_time_arg,
        condition_id_arg,
        pick_place_node,
    ])
