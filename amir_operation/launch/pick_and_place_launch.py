from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pick_server_node = Node(
        package='amir_operation',
        executable='pick_server',
        output='screen',
    )

    move_meca_server_node = Node(
        package='amir_operation',
        executable='move_meca_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    place_server_node = Node(
        package='amir_operation',
        executable='place_server',
        output='screen',
    )

    return LaunchDescription([
        pick_server_node,
        move_meca_server_node,
        place_server_node,
    ])
