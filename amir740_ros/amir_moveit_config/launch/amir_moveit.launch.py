# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.actions import ExecuteProcess
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder


# def generate_launch_description():

#     declared_arguments = []
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "rviz_config",
#             default_value="amir_moveit.rviz",
#             description="RViz configuration file",
#         )
#     )

#     return LaunchDescription(
#         declared_arguments + [OpaqueFunction(function=launch_setup)]
#     )


# def launch_setup(context, *args, **kwargs):

#     moveit_config = (
#         MoveItConfigsBuilder("amir")
#         .robot_description(file_path="config/amir.urdf.xacro")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_scene_monitor(
#             publish_robot_description=True, publish_robot_description_semantic=True
#         )
#         .planning_pipelines(
#             pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
#         )
#         .to_moveit_configs()
#     )

#     # Start the actual move_group node/action server
#     run_move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict()],
#     )

#     rviz_base = LaunchConfiguration("rviz_config")
#     rviz_config = PathJoinSubstitution(
#         [FindPackageShare("amir_moveit_config"), "launch", rviz_base]
#     )

#     # RViz
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             moveit_config.planning_pipelines,
#             moveit_config.joint_limits,
#         ],
#     )

#     # Static TF
#     static_tf = Node(
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         name="static_transform_publisher",
#         output="log",
#         arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0_1"],
#     )

#     # Publish TF
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[moveit_config.robot_description],
#     )

#     # ros2_control using FakeSystem as hardware
#     ros2_controllers_path = os.path.join(
#         get_package_share_directory("amir_moveit_config"),
#         "config",
#         "ros2_controllers.yaml",
#     )
#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[moveit_config.robot_description, ros2_controllers_path],
#         output="both",
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager-timeout",
#             "300",
#             "--controller-manager",
#             "/controller_manager",
#         ],
#     )

#     arm_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["arm_controller", "-c", "/controller_manager"],
#     )

#     hand_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["gripper_controller", "-c", "/controller_manager"],
#     )
#     nodes_to_start = [
#         static_tf,
#         robot_state_publisher,
#         ros2_control_node,
#         joint_state_broadcaster_spawner,
#         arm_controller_spawner,
#         hand_controller_spawner,
#         run_move_group_node,
#         rviz_node,
#     ]

#     return nodes_to_start

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="amir_moveit.rviz",
            # default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value="amir_hardware",
            description="Hardware interface type for ros2_control",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("amir")
        .robot_description(file_path="config/amir.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .robot_description_semantic(file_path="config/amir.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Manual xacro for ros2_control with hardware type mapping
    hardware_type = LaunchConfiguration("ros2_control_hardware_type").perform(context)
    xacro_file = os.path.join(
        get_package_share_directory("amir_moveit_config"),
        "config",
        "amir.urdf.xacro",
    )
    doc = xacro.process_file(xacro_file, mappings={"ros2_control_hardware_type": hardware_type})
    robot_description = {"robot_description": doc.toxml()}

    # Start the move_group action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz node
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("amir_moveit_config"), "launch", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static transform publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0_1"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("amir_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # Delay spawner until ros2_control_node is ready
    delay_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                hand_controller_spawner,
            ],
        )
    )

    return [
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        delay_spawner,
        run_move_group_node,
        rviz_node,
    ]
