#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('agrobot_simulation')
    desc_pkg = get_package_share_directory('agrobot_description')
    cont_pkg = get_package_share_directory('agrobot_control')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
                                                  default=os.path.join(desc_pkg, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')
    params_file = os.path.join(desc_pkg, 'config', 'nav2_params_original.yaml')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]
    )
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', resolution,
                   '-publish_period_sec', publish_period_sec]
    )
    input_map_node = Node(
        package='agrobot_rl',
        executable='input_map_constructor.py',
        name='grid_map',
        output='screen',
        parameters=[{'show': True}],
    )
    # launch navigation stack:
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True',
                          'params_file': params_file}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='launch RViz if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(desc_pkg, 'rviz', 'rviz_base_mapping.rviz'),
            description='path to rviz configuration file'),
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


        cartographer_node,
        occupancy_grid_node,
        launch_nav2,
        input_map_node,

    ])
