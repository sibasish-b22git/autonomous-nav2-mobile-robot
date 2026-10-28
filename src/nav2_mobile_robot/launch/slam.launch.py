import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Create the placeholder to catch the terminal argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Declare the argument so ROS 2 knows to look for it
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    # 3. Get the path to your custom slam.yaml file
    slam_config_file = os.path.join(
        get_package_share_directory('nav2_mobile_robot'),
        'slam.yaml'
    )

    # 4. Get the path to the official slam_toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # 5. Include the official launch file and pass your YAML file as an argument
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'slam_params_file': slam_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 6. Add both the declaration and the node to the final description
    return LaunchDescription([
        declare_use_sim_time,
        start_slam_toolbox
    ])
