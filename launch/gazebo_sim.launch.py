import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'example_package'

    # path to robot_state_publisher launch file
    rsp_path = os.path.join(
        get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
    )

    # Path to the controllers configuration file
    controllers_config_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'controllers.yaml'
    )

    # Launch robot_state_publisher with sim_time set to true
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Spawner nodes for controllers
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    position_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen',
    )

    # Launch description
    return LaunchDescription([        
        rsp,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        position_controller_node,
    ])
