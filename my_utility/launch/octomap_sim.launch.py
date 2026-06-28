# =============================================================================
#  octomap_sim.launch.py
#
#  ★★★ シミュレータ(ign-gazebo)用 / 方式A ★★★
#  gz が自前生成する点群 /d435/points を直接 OctoMap 化する。
#  convertpc_(深度→点群の自作変換)は通さない。
#  理由: sim の CameraInfo(内部パラメータ)が解像度と矛盾しており
#        (1280x720 なのに cx=160,cy=120,fx=277 = 320x240相当)、
#        自作逆投影だと点群が歪む。gz 純正点群はその問題を回避できる。
#
#  前提: 先に Gazebo を起動しておくこと
#      ros2 launch amir_gazebo gazebo_bringup.launch.py
#    さらに /d435/points を ROS に出すブリッジが必要:
#      ros2 launch amir_gazebo d435_bridge.launch.py
#    （gazebo_bringup.launch.py 内の points ブリッジはコメントアウトされているため）
#
#  データ流路:
#      /d435/points → compc_oct → octomap_out
#
#  使い方:
#      ros2 launch my_utility octomap_sim.launch.py
#  別の点群トピックを使う場合:
#      ros2 launch my_utility octomap_sim.launch.py cloud_topic:=/your/points
# =============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    target_frame = LaunchConfiguration("target_frame")
    source_frame = LaunchConfiguration("source_frame")
    resolution = LaunchConfiguration("resolution")
    max_range = LaunchConfiguration("max_range")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # gz 純正点群 (compc_oct が購読する /converted_pointcloud にリマップする)
        DeclareLaunchArgument("cloud_topic", default_value="/d435/points"),
        # 地図を固定するフレーム (台車含めワールド固定なら odom にする)
        DeclareLaunchArgument("target_frame", default_value="base_footprint"),
        # ★ /d435/points は gz規約(X前方/Z上)なので frame_id を d435_depth_frame で上書き。
        #    (frame_id の d435_depth_optical_frame をそのまま使うと 90°倒れて縦になる)
        DeclareLaunchArgument("source_frame", default_value="d435_depth_frame"),
        # ボクセルサイズ[m] (起動時のみ有効)
        DeclareLaunchArgument("resolution", default_value="0.05"),
        # センサからの採用範囲[m] (各軸±)。深度 far クリップ(5.0m)に合わせるなら 5.0
        DeclareLaunchArgument("max_range", default_value="5.0"),

        # 点群 → OctoMap (target_frame へ TF 変換してから蓄積)
        Node(
            package="my_utility",
            executable="compc_oct",
            name="pc_to_octomap_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": target_frame,
                "source_frame": source_frame,
                "resolution": resolution,
                "max_range": max_range,
            }],
            remappings=[("/converted_pointcloud", cloud_topic)],
        ),
    ])
