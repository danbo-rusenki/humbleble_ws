import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    ns = LaunchConfiguration('namespace').perform(context)
    this_dir = get_package_share_directory('mecanum_navigation2')
    slam_params = os.path.join(this_dir, 'config', 'mapper_params_online_async.yaml')

    # slam_toolbox は scan_topic を絶対名化するため、namespace 付きの
    # 絶対トピック (/amir1/scan) を明示指定する。map→odom TF はロボットごとの
    # /<ns>/tf に載せる (tf remap)。map/odom/base_footprint フレーム名は標準のまま。
    scan_topic = ('/' + ns + '/scan') if ns else 'scan'

    return [Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=ns,
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time == 'true',
             'scan_topic': scan_topic},
        ],
        # slam_toolbox は map / map_metadata を絶対名 /map で publish するため、
        # namespace 配下 (/<ns>/map) へ remap する。これをしないと global_costmap の
        # static_layer (map_topic: map → /<ns>/map) が地図を受け取れず、グローバル
        # プランナが経路を出せない (distance_remaining=0 のまま動かない)。
        # tf も同様にロボットごとの /<ns>/tf へ。
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('/map', 'map'), ('/map_metadata', 'map_metadata')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=_setup),
    ])
