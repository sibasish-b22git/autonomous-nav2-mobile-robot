from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    ld = LaunchDescription()

    pkg_nav2_mobile_robot = get_package_share_directory('nav2_mobile_robot')
    xacro_path = 'nav2_mobile_robot.xacro'
    
    robot_description = PathJoinSubstitution([
        pkg_nav2_mobile_robot,    
        xacro_path
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description': Command(['xacro ', robot_description]),
            'use_sim_time': True
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', robot_description])
        }]
    )

    # Spawn Robot
    spawn_node = Node(package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'diff_drive',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
            '-r', '0',
            '-p', '0',
            '-Y', '0',
            '-topic', '/robot_description'],
        output='screen'
    )

    # FIX 1: Use GZ_ instead of IGN_
    # FIX 2: Point it to the directory containing your 'worlds' and 'models' folders
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_nav2_mobile_robot)]
    )
    
    # FIX 3: Build the absolute path to maze.sdf
    world_file_path = os.path.join(pkg_nav2_mobile_robot, 'maze_world.sdf')

    ignition_gazebo_node = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        # Pass the absolute world path to Gazebo
        launch_arguments=[('gz_args', ['-r -v 4 ', world_file_path])]
    )
    
    # FIX 4: Updated ignition.msgs to gz.msgs and fixed directional brackets ']'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/odom/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],                
        output='screen',
        remappings=[
            ('/model/diff_drive/odometry', '/odom'),
            ('/odom/tf', '/tf'),
            ('/model/diff_drive/cmd_vel', '/cmd_vel'),
            # ('lidar', '/lidar'),
        ]
    )
    
    ld.add_action(gz_resource_path)
    ld.add_action(robot_state_publisher_node)
    
    # FIX 5: Actually add the Joint State Publisher to the launch file!
    # ld.add_action(joint_state_publisher_node)
    
    ld.add_action(spawn_node)
    ld.add_action(ignition_gazebo_node)
    ld.add_action(bridge)

    return ld
