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


def generate_launch_description():
    sim_pkg = get_package_share_directory('agrobot_simulation')
    desc_pkg = get_package_share_directory('agrobot_description')
    cont_pkg = get_package_share_directory('agrobot_control')

    conf_dir = os.path.join(desc_pkg, 'config')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
                                                  default=conf_dir)
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    nav_2_params = LaunchConfiguration('nav_2_params',
                                       default=os.path.join(conf_dir, 'nav2_dynamic_point_follower.yaml'))
    headless = LaunchConfiguration('headless')

    # launching the simulation
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'agrobot_base_gazebo.launch.py')),
        launch_arguments={'use_rviz': use_rviz,
                          'rviz_config': rviz_config,
                          'headless': headless}.items()
        )
    # launch navigation stack:
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'params_file': nav_2_params,
                          # 'use_namespace': 'true',
                          # 'namespace': 'nav_2_stack'
                          }.items()
        )

    # Launch the node for (re)setting the rl environment:
    reset_node = Node(
        package='agrobot_rl',
        executable='world_reset_node.py',
        name='env_reset',
        parameters=[{'use_sim_time': True}]
    )

    # node for creating the input map:
    input_map_node = Node(
        package='agrobot_rl',
        executable='input_map_constructor.py',
        parameters=[{'use_sim_time': True}]
    )
    # node hosting the navigation goal service:
    nav_service_node = Node(
        package='agrobot_rl',
        executable='nav_goal_publisher.py',
        parameters=[{'use_sim_time': True}]
    )
    dyn_nav_service_node = Node(
        package='agrobot_rl',
        executable='dyn_nav_goal_updater.py',
        parameters=[{'use_sim_time': True}]
    )
    click_to_point = Node(
        package='nav2_test_utils',
        executable='clicked_point_to_pose',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='launch RViz if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='rviz_base_mapping.rviz',
            description='Name of rviz configuration file in share of agrobot_description package'),
        DeclareLaunchArgument(
            'slam_param',
            default_value=os.path.join(desc_pkg, 'config', 'SLAM_params.yaml'),
            description='Path to the yaml file containing the parameters for the slam_toolbox'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Option to disable gazebos graphical client'),

        launch_sim,
        # launch_nav2,
        # reset_node,
        # input_map_node,
        # nav_service_node,
        # dyn_nav_service_node,
        # click_to_point
    ])
