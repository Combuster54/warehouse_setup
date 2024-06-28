import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ## Logs for Patrol Behavior Server
    patrol_log_level_arg = DeclareLaunchArgument(
        'patrol_log_level',
        default_value='info',
        description='Logging level for each node inside Patrol Behavior Server'
    )

    ## Logs for Approach Server
    approach_log_level_arg = DeclareLaunchArgument(
        'approach_log_level',
        default_value='info',
        description='Logging level for each node inside Approach Server'
    )

    ## Logs for Shelf Position Server
    position_log_level_arg = DeclareLaunchArgument(
        'position_log_level',
        default_value='info',
        description='Logging level for each node inside Shelf Position Server'
    )

    cart_frame_log_level_arg = DeclareLaunchArgument(
        'cart_frame_log_level',
        default_value='info',
        description='Logging level for each node inside Cart Frame'
    )
    
    shelf_real_distance_arg = DeclareLaunchArgument(
        'shelf_real_distance',
        default_value= 'True',
        description='Variable to select real/sim shelf distance between legs'
    )

    return LaunchDescription([
        shelf_real_distance_arg,
        position_log_level_arg,
        approach_log_level_arg,
        patrol_log_level_arg,
        cart_frame_log_level_arg,

        #~~~~~~~~~~~~~~~~~~approach_server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_apps',
            executable='approach_srv',
            output='screen',
            parameters=[{'approach_log_level_arg': LaunchConfiguration('approach_log_level')}]
        ),
        #~~~~~~~~~~~~~~~~~~patrol_behavior_sever~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_apps',
            executable='patrol_behavior',
            output='screen',
            parameters=[{'patrol_log_level_arg': LaunchConfiguration('patrol_log_level')}]

        ),
        #~~~~~~~~~~~~~~~~~~shelf_position_server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_apps',
            executable='shelf_position_srv',
            output='screen',
            parameters=[{'position_log_level_arg': LaunchConfiguration('position_log_level')}]

        ),
        Node(
            package='nav2_apps',
            executable='cart_frame',
            output='screen',
            parameters=[{'cart_frame_log_level_arg': LaunchConfiguration('cart_frame_log_level'),
                         'shelf_real_distance_arg': LaunchConfiguration('shelf_real_distance')}]
        ),
        Node(
            package='nav2_apps',
            executable='auto_localization',
            output='screen',
        ),
    ])
