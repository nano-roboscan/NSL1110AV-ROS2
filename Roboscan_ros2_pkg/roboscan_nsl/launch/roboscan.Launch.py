#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():


    launch_description = LaunchDescription()

    roboscan_node = Node(
        package='roboscan_nsl',
        executable='roboscan_node',
        output='screen')


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('roboscan_nsl'), 'rviz', 'config_cam660.rviz')],output='screen')

    launch_description.add_action(rviz_node)
    launch_description.add_action(roboscan_node)
   

    return launch_description
