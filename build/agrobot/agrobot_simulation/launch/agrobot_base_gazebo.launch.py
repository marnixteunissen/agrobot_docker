#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

import xacro


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    package_name = 'agrobot_simulation'
    package_share = get_package_share_directory(package_name)
    desc_package = get_package_share_directory('agrobot_description')
    default_rviz_config_dir = os.path.join(desc_package, 'rviz')
    default_rviz_config = 'agrobot_rviz_settings.rviz'

    world_def = os.path.join(package_share, 'worlds', 'agrobot_world_1.model')
    robot_localization_file_path = os.path.join(desc_package, 'config', 'ekf_galactic.yaml')

    # Declare launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')


    urdf_file = os.path.join(desc_package, 'urdf', 'agrobot_base_gazebo.urdf')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Declaring processes
    # launch file launching the gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
        )
    # launch file launching gazebos graphical interface
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(headless)
        )
    # Node publishing the robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True},
                    params]
    )
    # node spawning the agrobot in gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'agrobot_base'],
        output='screen')

    # Node for the Kalman filter for odometry
    # (currently not working i.c.w cartographer):
    start_robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path,
                    {'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='launch RViz if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='path to rviz configuration file'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Option to disable gazebos graphical client'),
        DeclareLaunchArgument(
            'world',
            default_value=world_def,
            description='path to the world file to use in the simulation'
        ),

        # uncomment next lines to enable ros2_control differential drive controller:
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_diff_drive_controller],
        #     )
        # ),

        # Launch rviz once robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', [default_rviz_config_dir, '/', rviz_config]],
                    condition=IfCondition(use_rviz))],
            )
        ),

        gazebo_server,
        gazebo_client,
        node_robot_state_publisher,
        spawn_entity,
    ])
