import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

_temp_sdf_path = None


def _create_spawn_actions(context, *args, **kwargs):
    global _temp_sdf_path

    mecanumrover3_gazebo_dir = get_package_share_directory("mecanumrover3_gazebo")
    models_dir = os.path.join(mecanumrover3_gazebo_dir, "models")
    original_sdf = os.path.join(models_dir, "small_coke_can", "model.sdf")

    with open(original_sdf, "r") as f:
        sdf_content = f.read()

    # model:// を file:// 絶対パスに置換してGazebo側でも解決できるようにする
    sdf_content = sdf_content.replace(
        "model://small_coke_can",
        f"file://{models_dir}/small_coke_can",
    )

    fd, tmp_path = tempfile.mkstemp(prefix="coke_can_", suffix=".sdf")
    with os.fdopen(fd, "w") as f:
        f.write(sdf_content)
    _temp_sdf_path = tmp_path

    def make_spawn(name, x, y, z):
        return Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-file", tmp_path,
                "-name", name,
                "-x", x, "-y", y, "-z", z,
                "-allow_renaming", "true",
            ],
        )

    return [
        make_spawn("coke_can_1", "0.5",  "-0.2",  "0.163"),
        make_spawn("coke_can_2", "2.0",  "1.0",  "0.163"),
        make_spawn("coke_can_3", "1.0",  "-1.0", "0.163"),
        make_spawn("coke_can_4", "2.0",  "-1.0", "0.163"),
    ]


def _cleanup(context, *args, **kwargs):
    global _temp_sdf_path
    if _temp_sdf_path and os.path.exists(_temp_sdf_path):
        os.remove(_temp_sdf_path)
    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_create_spawn_actions),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
    ])
