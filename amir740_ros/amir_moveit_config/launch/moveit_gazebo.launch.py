from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "octomap_enabled",
            default_value="true",
            description="起動時に octomap 点群リレーを ON にするか (実行中は /octomap_enable で切替)",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # Gazebo シミュレーション用 MoveIt2 launch
    # 前提: gazebo_bringup.launch.py が起動済みで以下が稼働中
    #   - robot_state_publisher (use_sim_time=true)
    #   - gazebo_ros2_control (controller_manager)
    #   - joint_state_broadcaster, wheel_velocity_controller
    #   - arm_controller (JointTrajectoryController)
    #   - gripper_controller (GripperActionController)
    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .robot_description_semantic(file_path="config/amir_mecanum3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # OctoMap (3D障害物) パラメータ。
    #   octomap_frame: 地図を固定するフレーム。world≡odom (下の static TF world→odom)
    #     なので odom にすると土台が走っても障害物がワールド座標に残る。
    octomap_params = {
        "octomap_frame": "odom",
        "octomap_resolution": 0.05,
    }

    # octomap 点群リレー (frame補正 + ON/OFF ゲート)
    #   gz の /d435/points を d435_depth_frame に補正して /moveit_cloud へ再配信。
    #   octomap_enabled=false で起動するとOFF。実行中は /octomap_enable で切替。
    octomap_enabled = LaunchConfiguration("octomap_enabled").perform(context)
    octomap_cloud_relay = Node(
        package="amir_moveit_config",
        executable="octomap_cloud_relay.py",
        name="octomap_cloud_relay",
        output="log",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/d435/points",
            "output_topic": "/moveit_cloud",
            "override_frame": "d435_depth_frame",
            "enabled": octomap_enabled.lower() in ("true", "1", "yes"),
        }],
    )

    # gazebo_ros2_control/GazeboSystem がミミックジョイントの ODE 拘束として
    # "<joint>_mimic" を生成し /joint_states に流す。これらは URDF に存在しないため
    # move_group が ERROR を出し続ける。フィルターノードで除外した
    # /joint_states_filtered を move_group に購読させる。
    joint_state_filter = Node(
        package="amir_moveit_config",
        executable="joint_state_filter.py",
        name="joint_state_filter",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            octomap_params,
            {"use_sim_time": True},
        ],
        remappings=[("joint_states", "joint_states_filtered")],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("amir_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        remappings=[("joint_states", "joint_states_filtered")],
    )

    # Static TF: world → odom (world≡odom)
    # 旧構成は world→base_footprint を静的固定していたが、それだと土台が動いても
    # MoveIt は原点扱いのまま。代わりに world→odom を identity で固定し、
    # odom→base_footprint は gazebo_bringup の odom_bridge が真値で動的配信する。
    # これで土台が走っても planning frame(world) 上で正しく追従し、octomap も
    # odom(=world) 固定で崩れない。
    # 注意: base_footprint の親が二重にならないよう、ここでは base_footprint に
    #       直接 TF を出さないこと (odom_bridge が odom→base_footprint を出す)。
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    return [
        joint_state_filter,
        static_tf,
        octomap_cloud_relay,
        run_move_group_node,
        rviz_node,
    ]
