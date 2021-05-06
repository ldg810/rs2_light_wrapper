#!/usr/bin/env python

import os.path as osp

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    rs2_node_param = [{'frame_name': 'camera_frame'},
                      {'depth_width': 320},
                      {'depth_height': 240},
                      {'depth_fps': 30}]

    realsense2_node = Node(package='rs2_light_wrapper',
            executable='rs2_light_node',
            parameters = rs2_node_param,
            output='screen',
            )

    return LaunchDescription([realsense2_node])
