
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    ld = LaunchDescription()

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        # 'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    nav_params = os.path.join(
        get_package_share_directory('nav2_mobile_robot'),
        'nav_mppi.yaml'
    )

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                ],
            ),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                respawn_delay=2.0,
                parameters=[nav_params],
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                ],
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
            ),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
            ),

            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel'),
                ],
            ),

            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[nav_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[
                    {'autostart': True},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ],
    )

    ld.add_action(load_nodes)
    return ld

