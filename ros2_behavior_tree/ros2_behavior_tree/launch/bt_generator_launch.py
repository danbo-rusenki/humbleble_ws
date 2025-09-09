import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ld = LaunchDescription()

    bt_generator_node = Node(
        package='failure_detection',
        executable='bt_generator_service',
        name='bt_generator_service',
        namespace='robot1',
        # parameters=[{'robot_name': 'mecanum1'}],
        output='screen',
        emulate_tty=True  #これがないと表示されない。foxy以降はemulate_tty=True
    )
    ld.add_action(bt_generator_node)
    return ld