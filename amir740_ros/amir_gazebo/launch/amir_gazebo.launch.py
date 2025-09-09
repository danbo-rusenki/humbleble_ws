# amir_gazebo
# いったんmoveitはなしでやるっぴ

import os

from ament_index_python.packages import get_package_share_directory
from amir_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():

    world_file = os.path.join(
        get_package_share_directory('amir_gazebo'), 'worlds', 'table.sdf')
    gui_config = os.path.join(
        get_package_share_directory('amir_gazebo'), 'gui', 'gui.config')

    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
            output='screen',
            shell=True
        )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'amir',
                   '-z', '1.015',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    # description_loader.gz_control_config_package = 'crane_x7_control'
    description_loader.gz_control_config_file_path = 'arm_controllers.yaml'
    description = description_loader.load()
    


    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen'
        )


    return LaunchDescription([
       ign_gazebo,
       ignition_spawn_entity,
       bridge
    ])