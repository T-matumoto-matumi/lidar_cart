import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    pkg_lidar_cart = get_package_share_directory('lidar_cart')
    
    robot_model = LaunchConfiguration('robot_model').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    if robot_model == 'four_ws':
        xacro_file = os.path.join(pkg_lidar_cart, 'urdf', 'four_ws_robot.urdf.xacro')
    else:
        xacro_file = os.path.join(pkg_lidar_cart, 'urdf', 'robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'lidar_cart',
                   '-z', '0.1'],
        output='screen'
    )
    
    spawn_entity_delayed = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return [node_robot_state_publisher, spawn_entity_delayed]

def generate_launch_description():
    pkg_lidar_cart = get_package_share_directory('lidar_cart')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_gui = LaunchConfiguration('gui', default='true')
    robot_model = LaunchConfiguration('robot_model', default='diff_drive')
    world_arg = LaunchConfiguration('world', default='obstacle.world')
    world_path = PythonExpression(["'", os.path.join(pkg_lidar_cart, 'worlds'), "/' + '", world_arg, "'"])

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='obstacle.world',
        description='World file name in worlds directory (e.g., obstacle.world, square_room.world)')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')

    declare_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='diff_drive',
        description='Choose robot model: "diff_drive" or "four_ws"')

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_drive_mode_cmd = DeclareLaunchArgument(
        'drive_mode',
        default_value='auto',
        description='Drive mode: "auto" (run drive_node) or "manual" (no drive_node)')

    # Gazebo Server (gzserver)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    # Gazebo Client (gzclient)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Drive Node (Only if drive_mode is auto)
    drive_node = Node(
        package='lidar_cart',
        executable='drive_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('drive_mode'), "' == 'auto'"]))
    )
    
    opaque_func = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        declare_world_cmd,
        declare_gui_cmd,
        declare_model_cmd,
        declare_sim_time_cmd,
        declare_drive_mode_cmd,
        gzserver,
        gzclient,
        drive_node,
        opaque_func
    ])
