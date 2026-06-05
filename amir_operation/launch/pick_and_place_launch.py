from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pick_server_node = Node(
        package='amir_operation',
        executable='pick_server',
        output='screen',
    )

    return LaunchDescription([
        pick_server_node,
    ])
