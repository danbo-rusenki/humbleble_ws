import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    # kinematics_file = os.path.join(
    #     os.path.dirname(__file__),
    #     'kinematics.yaml'
    # )
    # kinematics_file = os.path.join(os.path.dirname(__file__), 'kinematics.yaml')

    
    pickplace_node = Node(
        package='amir_operation',
        executable='pick_place_hum',
        output='screen',
        # parameters=[kinematics_file],
    )

    return LaunchDescription([
        
        pickplace_node
    ])

