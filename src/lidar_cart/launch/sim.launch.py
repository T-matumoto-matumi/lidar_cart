import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    pkg_lidar_cart = get_package_share_directory('lidar_cart')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_gui = LaunchConfiguration('gui', default='true')
    world_path = os.path.join(pkg_lidar_cart, 'worlds', 'obstacle.world')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')
    
    # Process URDF
    xacro_file = os.path.join(pkg_lidar_cart, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Gazebo Server (gzserver)
    # Explicitly load gazebo_ros_factory plugin to enable spawn_entity service
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    # Gazebo Client (gzclient)
    from launch.conditions import IfCondition
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn Entity
    # We add a small delay to allow Gazebo to fully initialize the factory service
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'lidar_cart',
                   '-z', '0.1'],
        output='screen'
    )
    
    # Delay spawn_entity to ensure Gazebo is ready
    from launch.actions import TimerAction
    spawn_entity_delayed = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # Drive Node
    drive_node = Node(
        package='lidar_cart',
        executable='drive_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_gui_cmd,
        gzserver,
        gzclient,
        node_robot_state_publisher,
        spawn_entity_delayed,
        drive_node
    ])
