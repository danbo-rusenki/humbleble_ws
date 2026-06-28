import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gen_sdf_and_spawn(context, *args, **kwargs):
    base_entity = LaunchConfiguration('entity').perform(context)
    shape = LaunchConfiguration('shape').perform(context)

    z = float(LaunchConfiguration('z').perform(context))
    roll = float(LaunchConfiguration('roll').perform(context))
    pitch = float(LaunchConfiguration('pitch').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))
    mass = float(LaunchConfiguration('mass').perform(context))

    # 生成したい座標
    positions = [
        (1.0, 1.0),
        (2.0, 1.0),
        (1.0, -1.0),
        (2.0, -1.0),
    ]

    kp = 1e6
    kd = 1e3
    mu = 3.0

    surface_xml = f"""
        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu}</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>{kp}</kp>
              <kd>{kd}</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>"""

    if shape == 'cylinder':
        radius = float(LaunchConfiguration('radius').perform(context))
        height = float(LaunchConfiguration('size_z').perform(context))

        ixx = iyy = (1.0 / 12.0) * mass * (3.0 * radius**2 + height**2)
        izz = 0.5 * mass * radius**2

        # 物理判定用の形状（単一の円柱）
        geom_xml_collision = f"""<cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>"""

        # YOLOv5 bottle認識向けビジュアル（キャップ付きシルエットハック）
        label_height = height * 0.35
        label_radius = radius * 1.02
        
        # ボトルの首（キャップ）部分のサイズと位置
        cap_radius = radius * 0.4
        cap_height = 0.02
        cap_z_offset = (height / 2) + (cap_height / 2)

        visual_xml = f"""
          <visual name="visual_base">
            <geometry>
              <cylinder>
                <radius>{radius}</radius>
                <length>{height}</length>
              </cylinder>
            </geometry>
            <material>
              <ambient>0.02 0.15 0.05 1.0</ambient>
              <diffuse>0.02 0.15 0.05 1.0</diffuse>
            </material>
          </visual>

          <visual name="visual_label">
            <geometry>
              <cylinder>
                <radius>{label_radius}</radius>
                <length>{label_height}</length>
              </cylinder>
            </geometry>
            <material>
              <ambient>0.95 0.95 0.95 1.0</ambient>
              <diffuse>0.95 0.95 0.95 1.0</diffuse>
            </material>
          </visual>
          
          <visual name="visual_cap">
            <pose>0 0 {cap_z_offset} 0 0 0</pose>
            <geometry>
              <cylinder>
                <radius>{cap_radius}</radius>
                <length>{cap_height}</length>
              </cylinder>
            </geometry>
            <material>
              <ambient>0.9 0.9 0.9 1.0</ambient>
              <diffuse>0.9 0.9 0.9 1.0</diffuse>
            </material>
          </visual>
        """
        link_name = 'cylinder_link'

    else:
        size_x = float(LaunchConfiguration('size_x').perform(context))
        size_y = float(LaunchConfiguration('size_y').perform(context))
        size_z = float(LaunchConfiguration('size_z').perform(context))

        ixx = (1.0 / 12.0) * mass * (size_y**2 + size_z**2)
        iyy = (1.0 / 12.0) * mass * (size_x**2 + size_z**2)
        izz = (1.0 / 12.0) * mass * (size_x**2 + size_y**2)

        geom_xml_collision = f"""<box>
              <size>{size_x} {size_y} {size_z}</size>
            </box>"""

        # 箱の場合は段ボールを模した茶色に設定
        visual_xml = f"""
          <visual name="visual">
            <geometry>
              {geom_xml_collision}
            </geometry>
            <material>
              <ambient>0.6 0.4 0.2 1.0</ambient>
              <diffuse>0.6 0.4 0.2 1.0</diffuse>
            </material>
          </visual>
        """
        link_name = 'box_link'

    actions = []
    tmpdir = tempfile.mkdtemp(prefix='spawn_obj_')

    for i, (current_x, current_y) in enumerate(positions):
        entity_name = f"{base_entity}_{i}"

        sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{entity_name}">
    <static>false</static>
    <link name="{link_name}">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          {geom_xml_collision}
        </geometry>
        {surface_xml}
      </collision>

      {visual_xml}
    </link>
  </model>
</sdf>
'''

        sdf_path = os.path.join(tmpdir, f'{entity_name}.sdf')
        with open(sdf_path, 'w') as f:
            f.write(sdf)

        delete_cmd = ExecuteProcess(
            cmd=[
                'ign', 'service',
                '-s', '/world/default/remove',
                '--reqtype', 'ignition.msgs.Entity',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "{entity_name}" type: MODEL',
            ],
            output='log',
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', sdf_path,
                '-name', entity_name,
                '-x', str(current_x),
                '-y', str(current_y),
                '-z', str(z),
                '-R', str(roll),
                '-P', str(pitch),
                '-Y', str(yaw),
                '-allow_renaming', 'false',
            ],
            output='screen',
        )

        actions.append(delete_cmd)
        actions.append(TimerAction(period=0.5, actions=[spawn_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='target_obj'),
        DeclareLaunchArgument('shape', default_value='cylinder', description="'cylinder' or 'box'"),

        # 円柱：半径2.5cm、高さ10cm
        DeclareLaunchArgument('radius', default_value='0.025'),
        DeclareLaunchArgument('size_z', default_value='0.10'),

        # box用
        DeclareLaunchArgument('size_x', default_value='0.05'),
        DeclareLaunchArgument('size_y', default_value='0.05'),

        # 高さ10cmの円柱なら z=0.05 で地面に接する
        DeclareLaunchArgument('z', default_value='1'),

        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mass', default_value='0.1'),

        OpaqueFunction(function=_gen_sdf_and_spawn),
    ])
