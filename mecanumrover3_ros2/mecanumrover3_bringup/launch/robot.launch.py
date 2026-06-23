from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 実機構成メモ:
    #   - robot_state_publisher は amir_bringup 側が (amir+台車の合体URDFで)
    #     /tf・/robot_description を配信するため、台車側では起動しない (二重起動は競合)。
    #   - joint_state は ros2_control の joint_state_broadcaster が /joint_states に出すため
    #     joint_state_publisher も不要。
    #   - rviz は実機本体では不要 (見るなら操作PC側で起動)。
    #   → ここでは台車のオドメトリ/twist転送ノード (pub_odom) のみ起動する。
    pub_odom_node = Node(
        package='mecanumrover3_bringup',
        executable='pub_odom',
        name='pub_odom',
    )

    return LaunchDescription([
        pub_odom_node,
    ])
