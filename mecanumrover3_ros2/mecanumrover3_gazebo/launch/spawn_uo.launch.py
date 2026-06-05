import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 起動引数の定義
    # デフォルトで visualize_lidar.sdf を読み込むように設定
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='visualize_lidar.sdf',
        description='World file name to load (e.g., visualize_lidar.sdf)'
    )

    # 2. ros_gz_sim パッケージの gz_sim.launch.py をインクルード
    # Ignition Gazebo を起動するための公式Launchファイルです。
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        # gz_args に起動オプションを渡す (-r: 自動再生, -v 4: 詳細ログ)
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')]
        }.items()
    )

    return LaunchDescription([
        world_arg,
        gz_sim_launch
    ])