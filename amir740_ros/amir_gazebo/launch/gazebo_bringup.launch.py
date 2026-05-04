import os
import re
import shutil
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _safe_kill_gazebo(context, *args, **kwargs):
    has_gzserver = subprocess.run(
        ["pgrep", "gzserver"], stdout=subprocess.DEVNULL
    ).returncode == 0
    has_gzclient = subprocess.run(
        ["pgrep", "gzclient"], stdout=subprocess.DEVNULL
    ).returncode == 0
    if has_gzserver or has_gzclient:
        os.system("killall -9 gzserver gzclient 2>/dev/null")
    os.system("rm -rf /dev/shm/fastrtps_port* 2>/dev/null")
    os.system("rm -f /tmp/.gazebo/lock")
    return [LogInfo(msg="[amir_gazebo] Safe kill done")]


def _make_world(context, *args, **kwargs):
    physics_type = LaunchConfiguration("physics").perform(context)
    if physics_type not in ("ode", "bullet"):
        physics_type = "ode"

    world_xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <physics type="{physics_type}">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <publish_model_state>true</publish_model_state>
      <publish_link_state>true</publish_link_state>
      <update_rate>30</update_rate>
    </plugin>
  </world>
</sdf>
"""
    fd, path = tempfile.mkstemp(prefix="amir_world_", suffix=".world")
    with os.fdopen(fd, "w") as f:
        f.write(world_xml)
    return [
        LogInfo(msg=f"[amir_gazebo] generated temp world: {path}"),
        SetLaunchConfiguration("world_path", path),
    ]


def _create_urdf_and_rsp(context, *args, **kwargs):
    xacro_file = os.path.join(
        get_package_share_directory("amir_description"),
        "urdf",
        "amir_mecanum3_sim.xacro",
    )
    xacro_exe = shutil.which("xacro") or "xacro"

    fd_urdf, urdf_path = tempfile.mkstemp(prefix="amir_mecanum3_", suffix=".urdf")
    os.close(fd_urdf)
    subprocess.run([xacro_exe, xacro_file, "-o", urdf_path], check=True)
    with open(urdf_path) as f:
        raw = f.read()
    # gazebo_ros2_control passes robot_description as --param key:=value to rcl,
    # which parses the value as a YAML plain scalar. Two things break YAML:
    #   1. raw newlines (terminate the scalar)
    #   2. ': ' (colon-space) patterns inside XML comments (treated as mapping)
    # Stripping XML comments and collapsing newlines makes the value safely parseable.
    urdf_content = re.sub(r'<!--.*?-->', '', raw, flags=re.DOTALL).replace('\n', ' ')

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": urdf_content,
                "use_sim_time": True,
            }
        ],
    )

    return [
        SetLaunchConfiguration("urdf_path", urdf_path),
        LogInfo(msg=f"[amir_gazebo] generated temp urdf: {urdf_path}"),
        rsp,
    ]


def _cleanup_temp_files(context, *args, **kwargs):
    world_path = LaunchConfiguration("world_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    for p in (world_path, urdf_path):
        try:
            if p and os.path.exists(p):
                os.remove(p)
        except Exception:
            pass
    return [LogInfo(msg="[amir_gazebo] Cleaned up temp files")]


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world_path = LaunchConfiguration("world_path")
    urdf_path = LaunchConfiguration("urdf_path")

    arm_controllers_yaml = os.path.join(
        get_package_share_directory("amir_gazebo"),
        "config",
        "arm_controllers.yaml",
    )

    mecanum_description_dir = get_package_share_directory("mecanumrover_description")
    amir_description_dir = get_package_share_directory("amir_description")

    safe_kill = OpaqueFunction(function=_safe_kill_gazebo)
    make_world = OpaqueFunction(function=_make_world)
    create_urdf_and_rsp = OpaqueFunction(function=_create_urdf_and_rsp)

    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "gazebo_ros", "gazebo.launch.py",
            ["gui:=", gui],
            ["world:=", world_path],
        ],
        additional_env={
            "GAZEBO_MODEL_PATH": (
                f"{os.path.dirname(mecanum_description_dir)}:"
                f"{os.path.dirname(amir_description_dir)}:"
                f"{os.environ.get('GAZEBO_MODEL_PATH', '')}"
            )
        },
        output="screen",
    )

    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", "amir_mecanum3",
            "-file", urdf_path,
            "-z", "0.03",
        ],
        output="screen",
    )

    # コントローラーは controller_manager のシングルスレッドサービスを
    # 同時に叩くとタイムアウト/二重ロードが発生するため順番に起動する
    # jsb → wheel → arm → gripper の順に OnProcessExit で連鎖させる
    jsb = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    wheel = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "wheel_velocity_controller",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    arm = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "arm_controller",
            "-t", "joint_trajectory_controller/JointTrajectoryController",
            "-p", arm_controllers_yaml,
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    gripper = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "gripper_controller",
            "-t", "position_controllers/GripperActionController",
            "-p", arm_controllers_yaml,
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    wheel_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[wheel])
    )
    arm_after_wheel = RegisterEventHandler(
        OnProcessExit(target_action=wheel, on_exit=[arm])
    )
    gripper_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm, on_exit=[gripper])
    )

    # メカナムローバー twist リレーノード
    rover_twist_relay = Node(
        package="mecanumrover_description",
        executable="rover_twist_relay.py",
        name="rover_twist_relay",
        output="screen",
        parameters=[
            os.path.join(mecanum_description_dir, "scripts", "rover_twist_relay.yaml"),
            {"rover": "mecanum3", "use_sim_time": True},
        ],
    )

    # Gazebo odometry → /odom ブリッジ
    gazebo_odom_bridge = Node(
        package="mecanumrover3_bringup",
        executable="gazebo_odom_bridge",
        name="gazebo_odom_bridge",
        parameters=[
            {"model_name": "amir_mecanum3"},
            {"base_link_name": "base_footprint"},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    on_shutdown_cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_temp_files)])
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("physics", default_value="ode"),
        safe_kill,
        make_world,
        create_urdf_and_rsp,
        gazebo,
        spawn,
        jsb,
        wheel_after_jsb,
        arm_after_wheel,
        gripper_after_arm,
        rover_twist_relay,
        gazebo_odom_bridge,
        on_shutdown_cleanup,
    ])
