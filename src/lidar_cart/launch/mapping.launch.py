import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_lidar_cart = get_package_share_directory('lidar_cart')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Simulation Launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar_cart, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'gui': 'true',
            'world': 'square_room.world',
            'drive_mode': 'manual'
        }.items()
    )

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_slam_toolbox, 'config', 'mapper_params_online_async.yaml') # Use default or custom
        }.items()
    )

    # Rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_lidar_cart, 'rviz', 'mapping.rviz')], # We might need to create this or use default
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        slam_toolbox,
        # rviz # Rviz is handled by user or we can add it if we have a config
    ])
