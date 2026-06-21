from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Nav2(slam_toolbox / amcl) を併用する場合は map→odom はそちらが publish するため
    # ここでは出さない (既定 false)。単体テレオプ等で map を固定したいときのみ true。
    publish_map_odom = LaunchConfiguration('publish_map_odom')
    declare_publish_map_odom = DeclareLaunchArgument(
        'publish_map_odom', default_value='false',
        description='static map→odom TF を出すか。Nav2併用時は false(既定)、'
                    '単体テレオプ時のみ true。'
    )

    # start_static_tf_node = 
    #       Node(
    #           package = 'tf2_ros',
    #           executalble='static_transform_publisher',
    #           arguments=['0','1.5','0','3.14159','3.14159','0','base_link','horizontal_laser_link'],
    #         #   namespace='mecanum2',
    #         #   remapping=[('/tf_static','/mecanum2/tf_static')]
    #         ),
              
              
    return LaunchDescription([

        declare_publish_map_odom,

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0.0', '0.0', '0.0', '0.0', '0', '0', 'base_footprint', 'base_link'],
        #     # namespace='amir',
        #     # remappings=[('/tf_static', '/amir/tf_static')],
        #     ),

        Node(
            package='my_utility',
            executable='odom_tf2_broadcaster',
            name='odom_tf2_broadcaster',
            output='screen',
            emulate_tty = True,
            # namespace='amir',
            # remappings=[('/tf', '/amir/tf')],
            ),

        # Nav2(slam_toolbox/amcl) 併用時は map→odom が競合するため既定で出さない。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            condition=IfCondition(publish_map_odom),
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'baselink']),    
        
        # Node(
        #     package = 'tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['-1.0','0.0','0.0','-0.05','0','0','map','ar_marker_0'], #2.0
        #     # namespace='mecanum2',
        #     # remappings=[('/tf_static','/mecanum2/tf_static')]
        #     ),
              
        # add_action(start_static_tf_node)       
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.0', '0.0', '0.0', '0.0', '0', '0', 'base_footprint', 'base_link'],
            # namespace='amir',
            # remappings=[('/tf_static', '/amir/tf_static')],
            ),        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.2', '0.0', '0.08', '3.14159', '3.14159', '0', 'base_link', 'horizontal_laser_link'],
            # namespace='amir',
            # remappings=[('/tf_static', '/amir/tf_static')],
            ),

        


        # Node(
        #     package = 'tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.2','0.0','0.22','0','0','0','base_link','camera_color_optical_frame'],
        #     # namespace='mecanum2',
        #     # remappings=[('/tf_static','/mecanum2/tf_static')]
        #     ),
        Node(
        package = 'tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0','0.0','0.0','0','0','0','base_link','camera_color_optical_frame'],
        # namespace='mecanum2',
        # remappings=[('/tf_static','/mecanum2/tf_static')]
        ),
        
    ])
