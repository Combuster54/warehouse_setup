import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():

    path_planner_server_pkg = 'path_planner_server'
    localization_server_pkg = 'localization_server'
    map_server_pkg = "map_server"
    use_sim_time = False


    #~~~~~~~~~~~~~~~~~~~~~~~~localization config files~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    amcl_file = os.path.join(get_package_share_directory(localization_server_pkg),'config','amcl_config.yaml')
    map_file_path = None

    if(use_sim_time):
        map_file_path = os.path.join(get_package_share_directory(map_server_pkg),'config','warehouse_map_sim.yaml')
        remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        common_params = {'use_sim_time': True}

    else:
        map_file_path = os.path.join(get_package_share_directory(localization_server_pkg),'config','real_warehouse_map.yaml')
        remappings = [('/cmd_vel', '/cmd_vel')]
        common_params = {'use_sim_time': False}

    #~~~~~~~~~~~~~~~~~~~~~~~~path_planner config files~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    controller_yaml = os.path.join(get_package_share_directory(path_planner_server_pkg), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(path_planner_server_pkg), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory(path_planner_server_pkg), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory(path_planner_server_pkg), 'config', 'recovery.yaml')

    #~~~~~~~~~~~~~~~~~~~~~~~~bt file path~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    rviz_file = os.path.join(get_package_share_directory('path_planner_server'),'rviz','pathplanning.rviz')

    bt_file_path = os.path.join(get_package_share_directory('path_planner_server'),'config','navigate_w_replanning_and_recovery.xml')
    # filters_yaml = os.path.join(get_package_share_directory(path_planner_server_pkg), 'config', 'filters.yaml')

    return LaunchDescription([    

        #~~~~~~~~~~~~~~~~~~provide map~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_file_path} 
                       ]),

        #~~~~~~~~~~~~~~~~~~amcl~~~~~~~~~~~~~~~~~~~~~~~~~~

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_file]

            ),

        #~~~~~~~~~~~~~~~~~~controller server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml,common_params],
            remappings = remappings
            ),

        #~~~~~~~~~~~~~~~~~~planner server~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, common_params]),

        #~~~~~~~~~~~~~~~~~~recoveries server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml,common_params],
            output='screen'),

        #~~~~~~~~~~~~~~~~~~~~~bt navigator~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'default_nav_to_pose_bt_xml': bt_file_path}]
        ),
        #~~~~~~~~~~~~~~~~~~~~~~~map server~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='filter_mask_server',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[filters_yaml]
        #     ),

        #~~~~~~~~~~~~~~~~~~~~~costmap filter~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Node(
        #     package='nav2_map_server',
        #     executable='costmap_filter_info_server',
        #     name='costmap_filter_info_server',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[filters_yaml]
        #     ),

        #~~~~~~~~~~~~~~~~~~~~lifecycle manager~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': [
                                    'map_server',
                                    'amcl',
                                    'planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    #'filter_mask_server',
                                    #'costmap_filter_info_server',
                        ]}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])