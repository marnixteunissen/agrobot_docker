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
    cont_package = get_package_share_directory('agrobot_control')
    default_rviz_config_path = os.path.join(desc_package, 'rviz', 'agrobot_rviz_settings.rviz')

    world = os.path.join(package_share, 'worlds', 'agrobot_world_1.model')
    robot_localization_file_path = os.path.join(desc_package, 'config', 'ekf_gazebo.yaml')

    # Declare launch configurations
    use_rviz = LaunchConfiguration('use_rviz')

    urdf_file = os.path.join(desc_package, 'urdf', 'agrobot_gazebo.urdf')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    controllers = os.path.join(cont_package, 'config', 'controller_om.yaml')

    # Declaring processes
    # launch file launching the gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    # launch file launching gazebos graphical interface
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

    # Node publishing the robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True},
                    params])

    # node spawning the agrobot in gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'agrobot'],
        output='screen')

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # process to start joint state publisher for the ros2_control arm
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen')

    # process to load the joint trajectory controller for the arm
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen')

    # process to load differential drive controller from ros2_control
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_controller'],
        output='screen')

    # node to turn arm to home position:
    homing_node = Node(
        package='agrobot_control',
        executable='arm_home_pose.py')

    # Node for the Kalman filter for odometry:
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
            default_value='True',
            description='launch RViz if true'),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        # uncomment next lines to enable ros2_control differential drive controller
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_diff_drive_controller],
        #     )
        # ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', default_rviz_config_path],
                    condition=IfCondition(use_rviz))],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[homing_node],
            )
        ),

        gazebo_server,
        gazebo_client,
        node_robot_state_publisher,
        spawn_entity,
        # control_node,

    ])
