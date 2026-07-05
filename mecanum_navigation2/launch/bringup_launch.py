import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def _rewrite_costmap_topics(params_file, ns):
    """コストマップの sensor/map トピックを絶対名 /<ns>/... に書き換えた一時 params
    を生成する。

    nav2 の costmap は相対トピック (scan / map) をコストマップ・サブ namespace
    (/<ns>/local_costmap/scan, /<ns>/global_costmap/map) 配下に解決してしまい、
    実際の /<ns>/scan・/<ns>/map と一致しない。その結果コストマップが地図/スキャン
    を受け取れず "current" にならず、プランナ/コントローラがハングして動かない。
    絶対名にすると costmap はそのまま購読する。
    """
    prefix = '/' + ns
    with open(params_file) as f:
        cfg = yaml.safe_load(f)

    # local_costmap: obstacle_layer の観測源 (scan) を /<ns>/scan へ
    try:
        obl = cfg['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
        for src in str(obl.get('observation_sources', '')).split():
            if isinstance(obl.get(src), dict) and 'topic' in obl[src]:
                obl[src]['topic'] = prefix + '/' + str(obl[src]['topic']).lstrip('/')
    except (KeyError, TypeError):
        pass

    # global_costmap: static_layer の地図 (map) を /<ns>/map へ
    try:
        stl = cfg['global_costmap']['global_costmap']['ros__parameters']['static_layer']
        if 'map_topic' in stl:
            stl['map_topic'] = prefix + '/' + str(stl['map_topic']).lstrip('/')
    except (KeyError, TypeError):
        pass

    fd, path = tempfile.mkstemp(prefix='nav2_%s_' % ns, suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


def _bringup_group(context, *args, **kwargs):
    """navigation_launch を包む。namespace 付きのときは costmap トピックを
    書き換えた params を渡す (二重ネスト回避)。"""
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    ns = LaunchConfiguration('namespace').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    if ns:
        params_file = _rewrite_costmap_topics(params_file, ns)

    return [GroupAction([
        PushRosNamespace(
            condition=IfCondition(LaunchConfiguration('use_namespace')),
            namespace=LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'autostart': LaunchConfiguration('autostart').perform(context),
                'params_file': params_file,
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),
    ])]


def _nav_rviz_config(base_path, ns):
    """nav2 既定 rviz 設定の絶対 topic を相対名にした一時設定を生成する。

    nav2_default_view.rviz は costmap/scan/plan 等を絶対ルート名 (/global_costmap/
    costmap 等) で持ち <robot_namespace> 置換もされないため、namespace 起動でも
    RViz は root を購読し実データ (/<ns>/*) と繋がらず costmap 等が表示されない
    (map/tf/goal_pose だけは rviz_launch が remap するので地図は出る)。
    Topic を相対名にすると namespace 化された RViz ノード配下 (/<ns>/...) に解決される。
    """
    import re
    with open(base_path) as f:
        text = f.read()
    text = re.sub(r'(?m)^(\s*Value: )/', r'\1', text)
    fd, path = tempfile.mkstemp(prefix='nav2_rviz_%s_' % ns, suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    return path


def _rviz_cmd(context, *args, **kwargs):
    """RViz を起動。namespace 付きのときは topic を相対名化した設定を渡し、
    costmap 等も /<ns>/ 配下を購読させる。"""
    if LaunchConfiguration('use_rviz').perform(context).lower() not in ('true', '1', 'yes'):
        return []
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    ns = LaunchConfiguration('namespace').perform(context)
    cfg = LaunchConfiguration('rviz_config_file').perform(context)
    if ns:
        cfg = _nav_rviz_config(cfg, ns)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            # namespace 起動時は RViz も /<ns>/ 配下へ入れる (topic 相対名を解決)。
            'use_namespace': LaunchConfiguration('use_namespace').perform(context),
            'rviz_config': cfg,
        }.items(),
    )]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(bringup_dir, 'launch')

    this_dir = get_package_share_directory('mecanum_navigation2')
    this_launch_dir = os.path.join(this_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(this_dir, 'params', 'amir.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz'
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RViz config file'
    )

    # slam_toolbox: online async mapping (builds map while driving)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_launch_dir, 'slam_toolbox_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items(),
    )

    # nav2: planner + controller + behavior (no localization — slam_toolbox handles TF)
    # costmap の sensor/map トピック二重ネストを避けるため OpaqueFunction 内で
    # namespace に応じた params を生成して渡す (_bringup_group / _rewrite_costmap_topics)。
    bringup_cmd_group = OpaqueFunction(function=_bringup_group)

    # Relay Nav2 /cmd_vel output to /rover_twist (→ mecanum drive controller)
    cmd_vel_relay = Node(
        package='mecanum_navigation2',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz は OpaqueFunction 内で生成 (namespace 付きは topic を相対名化した設定を渡す)。
    # false だと RViz がルートの /map /scan /global_costmap/costmap 等を掴み、実データ
    # (/<ns>/*) と繋がらず costmap 等が表示されない。
    rviz_cmd = OpaqueFunction(function=_rviz_cmd)

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(slam_toolbox_launch)
    ld.add_action(bringup_cmd_group)
    ld.add_action(cmd_vel_relay)
    ld.add_action(rviz_cmd)

    return ld
