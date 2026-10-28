from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def generate_launch_description():

    xacro_path = 'nav2_mobile_robot.xacro'
    

    robot_description = PathJoinSubstitution([
        get_package_share_directory('nav2_mobile_robot'),	
        xacro_path
    ])
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')
    

    ld = LaunchDescription()
    ld.add_action( robot_state_publisher_node )
    ld.add_action( joint_state_publisher_gui )
    ld.add_action( rviz2_node )

    return ld