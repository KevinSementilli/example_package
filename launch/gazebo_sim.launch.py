import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'example_package'

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')


    # path to robot_state_publisher launch file
    rsp_path = os.path.join(
        get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
    )

    # Path to the controllers configuration file
    controllers_config_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'controllers.yaml'
    )

    # Include robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Controller manager node
    ros2_controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/robot_description', '~/robot_description')]
    )

    # Spawner nodes for controllers
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    position_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller']
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        rsp,
        ros2_controller_node,
        joint_state_broadcaster_node,
        position_controller_node,
        gazebo,
        spawn_entity
    ])
