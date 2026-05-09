import math
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gen_sdf_and_spawn(context, *args, **kwargs):
    entity = LaunchConfiguration('entity').perform(context)
    shape  = LaunchConfiguration('shape').perform(context)   # 'cylinder' or 'box'

    x     = float(LaunchConfiguration('x').perform(context))
    y     = float(LaunchConfiguration('y').perform(context))
    z     = float(LaunchConfiguration('z').perform(context))
    roll  = float(LaunchConfiguration('roll').perform(context))
    pitch = float(LaunchConfiguration('pitch').perform(context))
    yaw   = float(LaunchConfiguration('yaw').perform(context))
    mass  = float(LaunchConfiguration('mass').perform(context))

    # ── 物理パラメータ (接触剛性・摩擦) ──────────────────────────────
    # kp: 接触バネ定数 [N/m]  大きいほど「硬い」物体
    # kd: 接触ダンパ   [N・s/m]
    # mu: 摩擦係数
    #
    # Gazebo の実効摩擦 = min(gripper_mu, object_mu)
    # グリッパー指は mu=3.0 に設定済みのため、物体も 3.0 に合わせる。
    # これにより実効摩擦が 1.0 → 3.0 に改善される。
    kp = 1e6   # 高剛性: 物体すり抜け防止に必須
    kd = 1e3   # 高制動: 接触振動を素早く減衰させる (旧値 1e2 → 10倍に増加)
    mu = 3.0

    surface_xml = f"""
        <surface>
          <contact>
            <ode>
              <kp>{kp}</kp>
              <kd>{kd}</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu}</mu2>
            </ode>
          </friction>
        </surface>"""

    if shape == 'cylinder':
        radius = float(LaunchConfiguration('radius').perform(context))
        height = float(LaunchConfiguration('size_z').perform(context))
        # 円柱の慣性モーメント
        ixx = iyy = (1.0 / 12.0) * mass * (3.0 * radius**2 + height**2)
        izz = 0.5 * mass * radius**2
        geom_xml = f"""<cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>"""
        link_name = 'cylinder_link'
    else:
        # box (後方互換)
        size_x = float(LaunchConfiguration('size_x').perform(context))
        size_y = float(LaunchConfiguration('size_y').perform(context))
        size_z = float(LaunchConfiguration('size_z').perform(context))
        ixx = (1.0 / 12.0) * mass * (size_y**2 + size_z**2)
        iyy = (1.0 / 12.0) * mass * (size_x**2 + size_z**2)
        izz = (1.0 / 12.0) * mass * (size_x**2 + size_y**2)
        geom_xml = f"""<box>
              <size>{size_x} {size_y} {size_z}</size>
            </box>"""
        link_name = 'box_link'

    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{entity}">
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
          {geom_xml}
        </geometry>
        {surface_xml}
      </collision>
      <visual name="visual">
        <geometry>
          {geom_xml}
        </geometry>
        <material>
          <ambient>0.9 0.5 0.1 1.0</ambient>
          <diffuse>0.9 0.5 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

    tmpdir  = tempfile.mkdtemp(prefix='spawn_obj_')
    sdf_path = os.path.join(tmpdir, f'{entity}.sdf')
    with open(sdf_path, 'w') as f:
        f.write(sdf)

    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', entity,
                '-file',   sdf_path,
                '-x', str(x), '-y', str(y), '-z', str(z),
                '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='target_obj'),

        # ── 形状: 'cylinder'(デフォルト) または 'box' ──────────────
        DeclareLaunchArgument('shape',  default_value='cylinder',
                              description="'cylinder' or 'box'"),

        # ── 位置: ロボット前方 x=0.4、地面(z=物体中心高さ) ──────────
        DeclareLaunchArgument('x',     default_value='0.4'),
        DeclareLaunchArgument('y',     default_value='0.0'),
        DeclareLaunchArgument('z',     default_value='0.05'),   # 地面 + 高さ/2
        DeclareLaunchArgument('roll',  default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),

        # ── 円柱パラメータ ────────────────────────────────────────
        DeclareLaunchArgument('radius',  default_value='0.025',
                              description='Cylinder radius [m] (diameter=5cm)'),
        DeclareLaunchArgument('size_z',  default_value='0.10',
                              description='Height [m]'),

        # ── 直方体パラメータ (shape:=box の場合) ──────────────────
        DeclareLaunchArgument('size_x',  default_value='0.05'),
        DeclareLaunchArgument('size_y',  default_value='0.05'),

        # ── 共通 ────────────────────────────────────────────────
        DeclareLaunchArgument('mass',    default_value='0.1'),

        OpaqueFunction(function=_gen_sdf_and_spawn),
    ])
