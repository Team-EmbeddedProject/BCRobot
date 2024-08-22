import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav2_params_file = os.path.join(
            get_package_share_directory('robot_package'),
            'params',
            'nav2_params.yaml'
    )

    lifecycle_nodes = ['map_server', 
                       'amcl', 
                       'controller_server',
                       'planner_server',
                       'bt_navigator',
                       'waypoint_follower']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        # Node(
        #     package='nav2_waypoint_follower',
        #     executable='waypoint_follower',
        #     name='waypoint_follower',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'autostart': True}, 
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])
