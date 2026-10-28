from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_mobile_robot'

# 1. Define the data_files list as a standalone variable FIRST
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['urdf/nav2_mobile_robot_macro.xacro']),
    ('share/' + package_name, ['urdf/nav2_mobile_robot.xacro']),
    ('share/' + package_name, ['launch/nav2_mobile_robot_gazebo.launch.py']),
    ('share/' + package_name, ['launch/slam.launch.py']),
    ('share/' + package_name, ['launch/display.launch.py']),
    ('share/' + package_name, ['launch/map_server.launch.py']),
    ('share/' + package_name, ['launch/navigation.launch.py']),
    ('share/' + package_name, ['launch/amcl.launch.py']),
    ('share/' + package_name, ['worlds/maze_1.world']),
    ('share/' + package_name, ['worlds/maze_2.world']),
    ('share/' + package_name, ['worlds/maze.sdf']),
    ('share/' + package_name, ['worlds/maze_world.sdf']),
    ('share/' + package_name, ['config/slam.yaml']),
    ('share/' + package_name, ['config/amcl.yaml']),
    ('share/' + package_name, ['config/nav.yaml']),
    ('share/' + package_name, ['config/nav_mppi.yaml']),
    ('share/' + package_name, ['map/map.yaml']),
    ('share/' + package_name, ['map/map.pgm']),
    

]

# 2. Run the loop to find all models and append them to the list
# for root, dirs, files in os.walk('models'):
#     data_files.append((os.path.join('share', package_name, root), [os.path.join(root, f) for f in dirs]))

# 3. Finally, call setup() and pass the completed list in
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,  # <--- Pass the completed variable here
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sibasish',
    maintainer_email='sonusibasish@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
