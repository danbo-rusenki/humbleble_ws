from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="実機=false / シミュレータ=true",
    )

    sim_time_param = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    pick_server_node = Node(
        package='amir_operation',
        executable='pick_server',
        output='screen',
        parameters=[sim_time_param],
    )

    move_meca_server_node = Node(
        package='amir_operation',
        executable='move_meca_server',
        output='screen',
        parameters=[sim_time_param],
    )

    place_server_node = Node(
        package='amir_operation',
        executable='place_server',
        output='screen',
        parameters=[sim_time_param],
    )

    return LaunchDescription([
        sim_time_arg,
        pick_server_node,
        move_meca_server_node,
        place_server_node,
    ])
