import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    pkg_lidar_cart = get_package_share_directory('lidar_cart')
    pkg_nav2_map_server = get_package_share_directory('nav2_map_server')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_lidar_cart, 'maps', 'square_room.yaml'))

    # Simulation Launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar_cart, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'world': 'square_room.world',
            'drive_mode': 'manual' # Drive controlled by local planner
        }.items()
    )
    
    # Nav2 Map Server
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_yaml_file}]
    )

    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # Custom MCL Node
    mcl_node = Node(
        package='lidar_cart',
        executable='mcl_node',
        name='mcl_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Global Planner
    global_planner_node = Node(
        package='lidar_cart',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Local Planner
    local_planner_node = Node(
        package='lidar_cart',
        executable='local_planner',
        name='local_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_lidar_cart, 'rviz', 'mcl.rviz')], # Reuse or create new config
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        map_server_node,
        map_server_lifecycle_manager,
        mcl_node,
        global_planner_node,
        local_planner_node,
        rviz
    ])
